module Planners.PRM
       ( Roadmap
       , buildRRG
       , buildKRRG
       , solve
       ) where

-- Motion Planning imports
import Data.StateSpace
import Data.MotionPlanningProblem

-- Graph library imports
import Data.Graph.Inductive hiding (dijkstra)
import Data.Graph.Dijkstra

-- Standard imports
import Data.List (sortBy, minimumBy, find)
import Data.Maybe (fromJust)
import Data.Function (on)
import Data.Monoid
import System.Random.Mersenne.Pure64 (PureMT, pureMT)
import qualified Control.Monad.Random as CMR

-- A roadmap is a graph with state type s and cost space c
data Roadmap s c = Roadmap
                   { _space :: StateSpace s
                   , _valid :: MotionValidity s
                   , _cost  :: MotionCost s c
                   , _graph :: Gr s c
                   }

distSqrd :: Roadmap s c -> s -> s -> Double
distSqrd = _stateDistanceSqrd . _space

dist :: Roadmap s c -> s -> s -> Double
dist = _stateDistance . _space

sample :: Roadmap s c -> StateSampler s
sample = _sampleUniform . _space

interp :: Roadmap s c -> s -> s -> Double -> s
interp = _interpolate . _space

-- In expandRoadmap, this function allows for specifying which graph
-- nodes to attempt to connect the new node to
type ConnectStrategy s c = Roadmap s c -> s -> [LNode s]

-- Try to connect a state to a roadmap using a given connection
-- strategy. If no collision-free connection can be made to the new
-- state, the state is not added to the roadmap.
expandRoadmap :: Roadmap s c -> ConnectStrategy s c -> s -> Roadmap s c
expandRoadmap rm cs newState =
  let nbrs = cs rm newState
      validNbrs = filter (_valid rm newState . snd) nbrs
  in
   case validNbrs of
     []      -> rm
     lnodes  -> let costs    = map (_cost rm newState . snd) lnodes
                    newAdjs  = zip costs (map fst lnodes)
                    newGraph = (newAdjs, head $ newNodes 1 (_graph rm), newState, newAdjs) &
                               _graph rm
                in  Roadmap (_space rm) (_valid rm) (_cost rm) newGraph

-- A strategy for generating all states in a roadmap within a given
-- radius of a given state.
withinRadiusStrategy :: Double -> ConnectStrategy s c
withinRadiusStrategy r rm s = filter withinRadius $ labNodes (_graph rm)
  where withinRadius (_, nodeState) = sqrdDist s nodeState <= r*r
        sqrdDist = distSqrd rm

-- A strategy for generating the k nearest states to the given state
-- in a roadmap
kNearestStrategy :: Int -> ConnectStrategy s c
kNearestStrategy k rm s = take k $ sortBy near $ labNodes $ _graph rm
  where near = compare `on` (distSqrd rm s . snd)

-- Strategy used to generate new states to add to the roadmap
type SamplingStrategy s c = Roadmap s c -> StateSampler s

-- Given a roadmap, generate a random state that's at most d units
-- away from the nearest state in the roadmap. This sampling strategy
-- creates a Voronoi-biased sampling.
voronoiSamplingStrategy :: Double -> SamplingStrategy s c
voronoiSamplingStrategy stepSize rm = do
  sampleState <- sample rm
  let nearState = minimumBy (near sampleState) $ (map snd . labNodes . _graph) rm
      d = dist rm nearState sampleState
  return $ if d <= stepSize
           then nearState
           else interp rm nearState sampleState (stepSize / d)

  where near s = compare `on` distSqrd rm s

sampleAndExpand ::
  SamplingStrategy s c -> ConnectStrategy s c -> Roadmap s c -> CMR.Rand PureMT (Roadmap s c)
sampleAndExpand ss cs rm = fmap (expandRoadmap rm cs) (ss rm)

iterateExpand ::
  SamplingStrategy s c -> ConnectStrategy s c -> Int -> Roadmap s c -> CMR.Rand PureMT (Roadmap s c)
iterateExpand ss cs n rm = go (return rm) 0
  where go randomRM i
          | i >= n = randomRM
          | otherwise = do
            roadmap <- randomRM
            go (sampleAndExpand ss cs roadmap) (i+1)

mkInitialRoadmap :: StateSpace s -> MotionValidity s -> MotionCost s c -> s -> Roadmap s c
mkInitialRoadmap space valid cost s = Roadmap { _space = space
                                              , _valid = valid
                                              , _cost = cost
                                              , _graph = ([], 1, s, []) & empty }

buildKRRG ::
  StateSpace s -> MotionValidity s -> MotionCost s c -> Double -> Int -> Int -> s -> CMR.Rand PureMT (Roadmap s c)
buildKRRG space valid cost d k n s =
  let initialRoadmap = mkInitialRoadmap space valid cost s
  in  iterateExpand (voronoiSamplingStrategy d) (kNearestStrategy k) n $ initialRoadmap

buildRRG ::
  StateSpace s -> MotionValidity s -> MotionCost s c -> Double -> Double -> Int -> s -> CMR.Rand PureMT (Roadmap s c)
buildRRG space valid cost d r n s =
  let initialRoadmap = mkInitialRoadmap space valid cost s
  in  iterateExpand (voronoiSamplingStrategy d) (withinRadiusStrategy r) n $ initialRoadmap

-- Given a precomputed roadmap and a motion planning problem, returns
-- the path solving the problem. This function returns [] if:
--
-- (a) The problem's start state cannot be connected to the roadmap
-- with a collision-free path
--
-- (b) The start state is not connected to a goal state on the roadmap
solve :: (Ord c, Monoid c) => Roadmap s c -> MotionPlanningProblem s -> [s]
solve rm p = let sortedLNodes = sortBy near $ labNodes $ _graph rm
                 nearLNode    = find (_valid rm (_startState p) . snd) sortedLNodes
             in
              case nearLNode of
                Nothing -> []
                Just (nearNode,_) ->
                 let paths = dijkstra (_graph rm) nearNode
                     goalPath = find (_goalSatisfied p . fromJust . lab (_graph rm) . head) paths
                 in
                  case goalPath of
                    Nothing -> []
                    Just path -> reverse $ map (fromJust . lab (_graph rm)) path
  where near = compare `on` (distSqrd rm (_startState p) . snd)

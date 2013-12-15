module Planners.PRM
       ( Roadmap
       , buildRRG
       --, buildRRGDefaultSeed
       --, buildKRRGDefaultSeed
       --, buildPRMStarDefaultSeed
       -- , buildKRRG
       , solve
       , printEdges
       , printStates
       ) where

-- Motion Planning imports
import Data.MotionPlanningProblem
import Data.NearestNeighbors

-- Graph library imports
import Data.Graph.Inductive hiding (dijkstra)
import Data.Graph.Dijkstra

-- Standard imports
import Data.List (sortBy, minimumBy, find, intercalate)
import Data.Tuple (swap)
import Data.Maybe (fromJust)
import Data.Function (on)
import Data.Monoid
import System.Random.Mersenne.Pure64 (PureMT, pureMT)
import qualified Control.Monad.Random as CMR

-- Debug import
import Debug.Trace

-- A roadmap is a graph with state type s and cost space c
data Roadmap s c n = Roadmap
                     { _space :: StateSpace s
                     , _valid :: MotionValidity s
                     , _cost  :: MotionCost s c
                     , _graph :: Gr s c
                     , _nn    :: n
                     }

-- type GrNN n s = n s Node

distSqrd :: Roadmap s c n -> s -> s -> Double
distSqrd = _stateDistanceSqrd . _space

dist :: Roadmap s c n -> s -> s -> Double
dist = _stateDistance . _space

sample :: Roadmap s c n -> StateSampler s
sample = _sampleUniform . _space

interp :: Roadmap s c n -> s -> s -> Double -> s
interp = _interpolate . _space

-- In expandRoadmap, this function allows for specifying which graph
-- nodes to attempt to connect the new node to
type ConnectStrategy s c n = Roadmap s c n -> s -> [LNode s]

-- Try to connect a state to a roadmap using a given connection
-- strategy. If no collision-free connection can be made to the new
-- state, the state is not added to the roadmap.
expandRoadmap :: NN n => Roadmap s c (n s Node) -> ConnectStrategy s c (n s Node) -> s -> Roadmap s c (n s Node)
expandRoadmap rm cs newState =
  let nbrs = cs rm newState
      validNbrs = filter (flip (_valid rm) newState . snd) nbrs
  in
   case validNbrs of
     []      -> rm
     lnodes  -> let costs    = map (_cost rm newState . snd) lnodes
                    newAdjs  = zip costs (map fst lnodes)
                    newNode  = head $ newNodes 1 (_graph rm)
                    newGraph = (newAdjs, newNode, newState, newAdjs) &
                               _graph rm
                    newNN    = insert (_nn rm) newState newNode
                in  Roadmap (_space rm) (_valid rm) (_cost rm) newGraph newNN

-- A strategy for generating all states in a roadmap within a given
-- radius of a given state.
withinRadiusStrategy :: NN n => Double -> ConnectStrategy s c (n s Node)
withinRadiusStrategy r rm s = map swap $ nearestR (_nn rm) r s

-- A strategy for generating the k nearest states to the given state
-- in a roadmap
kNearestStrategy :: NN n => Int -> ConnectStrategy s c (n s Node)
kNearestStrategy k rm s = map swap $ nearestK (_nn rm) k s

-- Strategy used to generate new states to add to the roadmap
type SamplingStrategy s c n = Roadmap s c n -> StateSampler s

-- Given a roadmap, generate a random state that's at most d units
-- away from the nearest state in the roadmap. This sampling strategy
-- creates a Voronoi-biased sampling.
voronoiSamplingStrategy :: NN n => Double -> SamplingStrategy s c (n s Node)
voronoiSamplingStrategy stepSize rm
  | (isEmpty . _graph) rm = error "Voronoi sampling requires at least one state in the roadmap!"
  | otherwise = do
    sampleState <- sample rm
    let nearState = fst $ nearest (_nn rm) sampleState
        d = dist rm nearState sampleState
    return $ if d <= stepSize
             then sampleState
             else interp rm nearState sampleState (stepSize / d)

sampleAndExpand :: NN n =>
  SamplingStrategy s c (n s Node) -> ConnectStrategy s c (n s Node) -> Roadmap s c (n s Node) -> CMR.Rand PureMT (Roadmap s c (n s Node))
sampleAndExpand ss cs rm = fmap (expandRoadmap rm cs) (ss rm)

iterateExpand :: NN n =>
  SamplingStrategy s c (n s Node) -> ConnectStrategy s c (n s Node) -> Int -> Roadmap s c (n s Node) -> CMR.Rand PureMT (Roadmap s c (n s Node))
iterateExpand ss cs n rm = go (return rm) 0
  where go randomRM i
          | i >= n = randomRM
          | otherwise = do
            roadmap <- randomRM
            go (sampleAndExpand ss cs roadmap) (i+1)

mkInitialRoadmap :: NN n => StateSpace s -> MotionValidity s -> MotionCost s c -> n s Node -> s -> Roadmap s c (n s Node)
mkInitialRoadmap space valid cost nn s = Roadmap { _space = space
                                                 , _valid = valid
                                                 , _cost  = cost
                                                 , _graph = ([], 1, s, []) & empty
                                                 , _nn    = insert nn s 1
                                                 }

-- Performs n iterations of roadmap expansion using a voronoi sampling
-- strategy and k-nearest neighbor connection strategy
buildKRRGWithNN :: NN n =>
  StateSpace s -> MotionValidity s -> MotionCost s c -> n s Node -> Double -> Int -> Int -> s -> CMR.Rand PureMT (Roadmap s c (n s Node))
buildKRRGWithNN space valid cost nn d k n s =
  let initialRoadmap = mkInitialRoadmap space valid cost nn s
  in  iterateExpand (voronoiSamplingStrategy d) (kNearestStrategy k) n $ initialRoadmap

-- Performs n iterations of roadmap expansion using a voronoi sampling
-- strategy and connecting neighbors within a given radius of each new
-- node
buildRRGWithNN :: NN n =>
  StateSpace s -> MotionValidity s -> MotionCost s c -> n s Node -> Double -> Double -> Int -> s -> CMR.Rand PureMT (Roadmap s c (n s Node))
buildRRGWithNN space valid cost nn d r n s =
  let initialRoadmap = mkInitialRoadmap space valid cost nn s
  in  iterateExpand (voronoiSamplingStrategy d) (withinRadiusStrategy r) n $ initialRoadmap

buildPRMStarWithNN :: NN n =>
  StateSpace s -> MotionValidity s -> MotionCost s c -> n s Node -> Double -> Int -> CMR.Rand PureMT (Roadmap s c (n s Node))
buildPRMStarWithNN space valid cost nn r n =
  let initialRoadmap = mkInitialRoadmap space valid cost nn `fmap` _sampleUniform space
  in  iterateExpand (_sampleUniform space `const`) (withinRadiusStrategy r) n =<< initialRoadmap

buildRRG :: StateSpace s -> MotionValidity s -> MotionCost s c -> Double -> Double -> Int -> s -> CMR.Rand PureMT (Roadmap s c (LinearNN s Node))
buildRRG space valid cost = buildRRGWithNN space valid cost (mkLinearNN space)

-- buildPRMStarDefaultSeed :: StateSpace s -> MotionValidity s -> MotionCost s c -> Double -> Int -> Roadmap s c
-- buildPRMStarDefaultSeed space valid cost r n =
--   CMR.evalRand (buildPRMStar space valid cost r n) (pureMT 1)

-- buildRRGDefaultSeed ::
--   StateSpace s -> MotionValidity s -> MotionCost s c -> Double -> Double -> Int -> s -> Roadmap s c
-- buildRRGDefaultSeed space valid cost d r n s =
--   CMR.evalRand (buildRRG space valid cost d r n s) (pureMT 2)

-- buildKRRGDefaultSeed ::
--   StateSpace s -> MotionValidity s -> MotionCost s c -> Double -> Int -> Int -> s -> Roadmap s c
-- buildKRRGDefaultSeed space valid cost d k n s =
--   CMR.evalRand (buildKRRG space valid cost d k n s) (pureMT 2)

-- Given a precomputed roadmap and a motion planning query, returns
-- the path solving the query. This function returns [] if:
--
-- (a) The problem's start state cannot be connected to the roadmap
-- with a collision-free path
--
-- (b) The start state is not connected to a goal state on the roadmap
solve :: (Ord c, Monoid c, NN n) => Roadmap s c (n s Node) -> MotionPlanningQuery s -> [s]
solve rm q = let sortedLNodes = sortBy near $ labNodes $ _graph rm
                 nearLNode    = find (_valid rm (_startState q) . snd) sortedLNodes
             in
              case nearLNode of
                Nothing -> trace "NO NEAR NODES?" []
                Just (nearNode,_) ->
                 let paths = dijkstra (_graph rm) nearNode
                     goalPath = find (_goalSatisfied q . fromJust . lab (_graph rm) . head) paths
                 in
                  case goalPath of
                    Nothing -> trace "NO GOAL NODES?" []
                    Just path -> reverse $ map (fromJust . lab (_graph rm)) path
  where near = compare `on` (distSqrd rm (_startState q) . snd)

printEdges :: (Show s) => Roadmap s c n -> String
printEdges rm = intercalate "\n" $ map edgeToStr $ (edges . _graph) rm
  where
    edgeToStr :: (Node, Node) -> String
    edgeToStr (n1, n2) = nodeToStr n1 ++ " " ++ nodeToStr n2
    nodeToStr = (show . fromJust . lab (_graph rm))

printStates :: (Show s) => Roadmap s c n -> String
printStates rm = intercalate "\n" $ map (show . snd) $ (labNodes . _graph) rm

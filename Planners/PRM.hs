module Planners.PRM
       ( Roadmap
       , expandRoadmapR
       , expandRoadmapK ) where

-- Motion Planning imports
import Data.StateSpace
import Data.MotionPlanningProblem

-- Graph library imports
import Data.Graph.Inductive hiding (dijkstra)
import Data.Graph.Dijkstra

-- Standard imports
import Data.List (sortBy, minimumBy)
import Data.Function (on)
import Data.Monoid

type MotionCost s c = s -> s -> c

-- A roadmap is a graph with state type s and cost space c
data Roadmap s c = Roadmap
                   { _problem :: MotionPlanningProblem s
                   , _cost  :: MotionCost s c
                   , _graph   :: Gr s c
                   }

getSpace :: Roadmap s c -> StateSpace s
getSpace = _stateSpace . _problem

valid :: Roadmap s c -> MotionValidityFn s
valid = _motionValidity . _problem

distSqrd :: Roadmap s c -> s -> s -> Double
distSqrd = _stateDistanceSqrd . getSpace

dist :: Roadmap s c -> s -> s -> Double
dist = _stateDistance . getSpace

sample :: Roadmap s c -> StateSampler s
sample = _sampleUniform . getSpace

interp :: Roadmap s c -> s -> s -> Double -> s
interp = _interpolate . getSpace

-- In expandRoadmap, this function allows for specifying which graph
-- nodes to attempt to connect the new node to
type ConnectStrategy s c = Roadmap s c -> [LNode s]

-- Try to connect a state to a roadmap using a given connection
-- strategy. If no collision-free connection can be made to the new
-- state, the state is not added to the roadmap.
expandRoadmap :: Roadmap s c -> ConnectStrategy s c -> s -> Roadmap s c
expandRoadmap rm cs newState =
  let nbrs = cs rm
      validNbrs = filter (valid rm newState . snd) nbrs
  in
   case validNbrs of
     []      -> rm
     lnodes  -> let costs    = map (_cost rm newState . snd) lnodes
                    newAdjs  = zip costs (map fst lnodes)
                    newGraph = (newAdjs, head $ newNodes 1 (_graph rm), newState, newAdjs) &
                               _graph rm
                in  Roadmap (_problem rm) (_cost rm) newGraph

-- A strategy for generating all states in a roadmap within a given
-- radius of a given state.
withinRadiusStrategy :: s -> Double -> ConnectStrategy s c
withinRadiusStrategy s r rm = filter withinRadius $ labNodes (_graph rm)
  where withinRadius (_, nodeState) = sqrdDist s nodeState <= r*r
        sqrdDist = distSqrd rm

-- A strategy for generating the k nearest states to the given state
-- in a roadmap
kNearestStrategy :: s -> Int -> ConnectStrategy s c
kNearestStrategy s k rm = take k $ sortBy near $ labNodes $ _graph rm
  where near = compare `on` (distSqrd rm s . snd)

-- Expand the roadmap using the withinRadiusStrategy
expandRoadmapR :: Roadmap s c -> Double -> s -> Roadmap s c
expandRoadmapR rm radius newState =
  expandRoadmap rm (withinRadiusStrategy newState radius) newState

-- Expand the roadmap using the kNearestStrategy
expandRoadmapK :: Roadmap s c -> Int -> s -> Roadmap s c
expandRoadmapK rm k newState =
  expandRoadmap rm (kNearestStrategy newState k) newState

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

-- Given a roadmap, return the shortest path on the roadmap between
-- two given nodes.
shortestPath :: (Ord c, Monoid c) => Roadmap s c -> Node -> Node -> [LNode s]
shortestPath rm = dijkstra (_graph rm)

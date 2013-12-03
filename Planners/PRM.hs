module Planners.PRM
       ( Roadmap
       , expandRoadmapR ) where

-- Motion Planning imports
import Data.StateSpace
import Data.Cost
import Data.MotionPlanningProblem

-- Graph library imports
import Data.Graph.Inductive

-- Standard imports
import Data.Maybe (fromJust)

-- A roadmap is a graph with state type s and cost space c
data Roadmap s c = Roadmap
                   { _problem :: MotionPlanningProblem s
                   , _cSpace  :: CostSpace c s
                   , _graph   :: Gr s c
                   }

stateFilter :: Roadmap s c -> (s -> Bool) -> [LNode s]
stateFilter rm f = filter (f . snd) $ labNodes (_graph rm)

getSpace :: Roadmap s c -> StateSpace s
getSpace = _stateSpace . _problem

valid :: Roadmap s c -> MotionValidityFn s
valid = _motionValidity . _problem

cost :: Roadmap s c -> (s -> s -> c)
cost = _motionCost . _cSpace

-- in expandRoadmap, this function allows for specifying which graph
-- nodes to attempt to connect the new node to
type ConnectStrategy s c = Roadmap s c -> [LNode s]

-- Try to connect a state to a roadmap using a given connection
-- strategy. If no collision-free connection can be made to the new
-- state, the state is not added to the roadmap.
expandRoadmap :: Roadmap s c -> ConnectStrategy s c -> s -> Roadmap s c
expandRoadmap rm cs newState =
  let nbrs = cs rm
      validNbrs = filter ((valid rm $ newState) . snd) nbrs
  in
   case validNbrs of
     []      -> rm
     lnodes  -> let costs    = map ((cost rm $ newState) . snd) lnodes
                    newAdjs  = zip costs (map fst lnodes)
                    newGraph = (newAdjs, head $ newNodes 1 (_graph rm), newState, newAdjs) &
                               (_graph rm)
                in  Roadmap (_problem rm) (_cSpace rm) newGraph

-- A strategy for generating all states in a roadmap within a given
-- radius of a given state.
withinRadiusStrategy :: s -> Double -> ConnectStrategy s c
withinRadiusStrategy s r rm = filter withinRadius $ labNodes (_graph rm)
  where withinRadius (_, nodeState) = sqrdDist s nodeState <= r*r
        sqrdDist = (_fastNonMetricDistance . _stateSpace . _problem) rm

expandRoadmapR :: Roadmap s c -> s -> Double -> Roadmap s c
expandRoadmapR rm newState radius =
  expandRoadmap rm (withinRadiusStrategy newState radius) newState

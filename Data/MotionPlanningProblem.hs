module Data.MotionPlanningProblem
       ( MotionPlanningProblem(..)
       , goalStateSatisfied
       ) where

import Data.StateSpace

data MotionPlanningProblem s = MotionPlanningProblem
                               { _stateSpace     :: StateSpace s
                               , _startState     :: s
                               , _goalSatisfied  :: s -> Bool
                               }

goalStateSatisfied :: StateSpace s -> Double -> s -> s -> Bool
goalStateSatisfied ss tol goalState s = _stateDistanceSqrd ss s goalState <= tol*tol

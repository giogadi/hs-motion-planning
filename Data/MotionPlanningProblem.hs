module Data.MotionPlanningProblem
       ( StateSpace(..)
       , DistFn
       , StateSampler
       , MotionValidity
       , discreteMotionValid
       , MotionPlanningQuery(..)
       , goalStateSatisfied
       , MotionCost
       , pathCost
       , evalDefaultSeed
       ) where

import qualified Control.Monad.Random as CMR
import System.Random.Mersenne.Pure64 (PureMT, pureMT)
import Data.Monoid

-- |A distance function that maps two states to a real number. In most
-- cases, this function satisfies the properties of a metric.
type DistFn s = s -> s -> Double

-- |Generates random states in a state space.
type StateSampler s = CMR.Rand PureMT s

-- |An interpolator takes two states, q(0) and q(1), as well as an
-- interpolation parameter t in [0,1] and returns q(t).
type Interpolator s = s -> s -> Double -> s

-- |Encapsulates all vital information about a state space.
data StateSpace s = StateSpace
                    { _stateDistance     :: DistFn s
                    , _stateDistanceSqrd :: DistFn s
                    , _interpolate       :: Interpolator s
                    , _sampleUniform     :: StateSampler s
                    }

-- |Returns true if the motion connecting two states is valid. If the
-- two states are q(0) and q(1), the motion between the two states is
-- expected to be all states generated by the StateSpace's
-- interpolator function with interpolation parameter in [0,1].
type MotionValidity s = s -> s -> Bool

-- |Returns true if the given state is valid. Useful for
-- discreteMotionValid.
type StateValidity s = s -> Bool

-- |A useful MotionValidity function. Given a StateValidity function
-- and a step size h, approximates the motion validity computation by
-- taking discrete steps of size h from s1 and checking state
-- validity. State s1 does NOT get checked for validity (assumed
-- valid) and s2 is ALWAYS checked. Step size is in the same units as
-- the StateSpace's _stateDistance function.
discreteMotionValid :: StateSpace s -> StateValidity s -> Double -> s -> s -> Bool
discreteMotionValid ss f h s1 s2
  | h <= 0.0  = error "Data.StateSpace.discreteMotionValid must have a positive step size"
  | otherwise = let d = _stateDistance ss s1 s2
                    n = (floor $ d / h) :: Int
                    samplePts = scanl1 (+) (replicate n (h / d))
                    innerValid = all f $ map (_interpolate ss s1 s2) samplePts
                in  innerValid && f s2

evalDefaultSeed :: CMR.Rand PureMT s -> s
evalDefaultSeed s = CMR.evalRand s (pureMT 1)

-- |A predicate that decides whether a given state lies in the goal
-- (i.e., a motion plan's endpoint).
type GoalPredicate s = s -> Bool

-- |An actual motion planning problem to be solved.
data MotionPlanningQuery s = MotionPlanningQuery
                             { _startState     :: s
                             , _goalSatisfied  :: GoalPredicate s
                             }

goalStateSatisfied :: StateSpace s -> Double -> s -> GoalPredicate s
goalStateSatisfied ss tol goalState s = _stateDistanceSqrd ss s goalState <= tol*tol

type MotionCost s c = s -> s -> c

pathCost :: (Monoid c) => MotionCost s c -> [s] -> c
pathCost _ [] = mempty
pathCost cost states@(_:ss) = mconcat $ zipWith cost states ss

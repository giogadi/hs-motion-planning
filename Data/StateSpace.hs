module Data.StateSpace
       ( StateSpace(..)
       , StateSampler
       , MotionValidity
       , discreteMotionValid
       , MotionPlanningQuery(..)
       , goalStateSatisfied
       , MotionCost
       ) where

import qualified Control.Monad.Random as CMR
import System.Random.Mersenne.Pure64 (PureMT)

type StateSampler s = CMR.Rand PureMT s

data StateSpace s = StateSpace
                    { _stateDistance     :: s -> s -> Double
                    , _stateDistanceSqrd :: s -> s -> Double
                    , _interpolate       :: s -> s -> Double -> s
                    , _sampleUniform     :: StateSampler s
                    }

type MotionValidity s = s -> s -> Bool

discreteMotionValid :: StateSpace s -> (s -> Bool) -> Double -> s -> s -> Bool
discreteMotionValid ss f h s1 s2
  | h <= 0.0  = error "Data.StateSpace.discreteMotionValid must have a positive step size"
  | otherwise = let d = _stateDistance ss s1 s2
                    n = (floor $ d / h) :: Int
                    samplePts = scanl1 (+) (replicate n (h / d))
                    innerValid = all f $ map (_interpolate ss s1 s2) samplePts
                in  innerValid && f s2

data MotionPlanningQuery s = MotionPlanningQuery
                             { _startState     :: s
                             , _goalSatisfied  :: s -> Bool
                             }

goalStateSatisfied :: StateSpace s -> Double -> s -> s -> Bool
goalStateSatisfied ss tol goalState s = _stateDistanceSqrd ss s goalState <= tol*tol

type MotionCost s c = s -> s -> c

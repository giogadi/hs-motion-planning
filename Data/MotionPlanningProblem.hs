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
       ) where

import qualified Control.Monad.Random as CMR
import System.Random.Mersenne.Pure64 (PureMT)
import Data.Monoid

type DistFn s = s -> s -> Double
type StateSampler s = CMR.Rand PureMT s

data StateSpace s = StateSpace
                    { _stateDistance     :: DistFn s
                    , _stateDistanceSqrd :: DistFn s
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

pathCost :: (Monoid c) => MotionCost s c -> [s] -> c
pathCost _ [] = mempty
pathCost cost states@(_:ss) = mconcat $ zipWith cost states ss

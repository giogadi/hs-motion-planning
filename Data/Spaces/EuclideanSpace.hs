module Data.Spaces.EuclideanSpace
       ( mkEuclideanSpace
       ) where

import qualified Data.FixedList as FL
import Control.Applicative
import Data.Foldable (sum)
import qualified Data.MotionPlanningProblem as MP
import qualified Control.Monad.Random as CMR
import Data.Traversable

addV :: (FL.FixedList f) => f Double -> f Double -> f Double
v1 `addV` v2 = pure (+) <*> v1 <*> v2

minusV :: (FL.FixedList f) => f Double -> f Double -> f Double
v1 `minusV` v2 = pure (-) <*> v1 <*> v2

scaleV :: (FL.FixedList f) => f Double -> Double -> f Double
scaleV v a = fmap (*a) v

dotV :: (FL.FixedList f) => f Double -> f Double -> Double
v1 `dotV` v2 = Data.Foldable.sum $ pure (*) <*> v1 <*> v2

lengthV :: (FL.FixedList f) => f Double -> Double
lengthV v = sqrt $ v `dotV` v

normalizeV :: (FL.FixedList f) => f Double -> f Double
normalizeV v = let scale = 1.0 / lengthV v
               in  scaleV v scale

stateDistance :: (FL.FixedList f) => f Double -> f Double -> Double
stateDistance s1 s2 = lengthV $ s2 `minusV` s1

stateDistanceSqrd :: (FL.FixedList f) => f Double -> f Double -> Double
stateDistanceSqrd s1 s2 = let dv = s2 `minusV` s1
                          in  dv `dotV` dv

interpolate :: (FL.FixedList f) => f Double -> f Double -> Double -> f Double
interpolate s1 s2 d
  | d < 0.0 || d > (1.0 + 1e-8) =
    error $ "Data.EuclideanSpace.interpolate's parameter must be in [0,1]" ++ show d
  | otherwise = let v = s2 `minusV` s1
                in  s1 `addV` scaleV v d

getUniformSampler :: FL.FixedList f =>
                     f Double -> f Double -> MP.StateSampler (f Double)
getUniformSampler minState maxState = let bounds = pure (,) <*> minState <*> maxState
                                      in  sequenceA $ fmap CMR.getRandomR bounds

mkEuclideanSpace :: FL.FixedList f =>
                    f Double -> f Double -> MP.StateSpace (f Double)
mkEuclideanSpace minState maxState = MP.StateSpace
                                     stateDistance
                                     stateDistanceSqrd
                                     interpolate
                                     (getUniformSampler minState maxState)

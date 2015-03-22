{-# LANGUAGE CPP #-}

module Data.Spaces.EuclideanSpace
       ( mkEuclideanSpace
       ) where

#if MIN_VERSION_base(4,8,0)
#else
import Control.Applicative
import Data.Traversable
#endif

import qualified Data.MotionPlanningProblem as MP
import qualified Control.Monad.Random as CMR

lengthSqrd :: [Double] -> Double
lengthSqrd s = sum $ zipWith (*) s s

stateDistanceSqrd :: [Double] -> [Double] -> Double
stateDistanceSqrd s1 s2 = lengthSqrd $ zipWith (-) s2 s1

stateDistance :: [Double] -> [Double] -> Double
stateDistance s1 s2 = sqrt $ stateDistanceSqrd s1 s2

interpolate :: [Double] -> [Double] -> Double -> [Double]
interpolate s1 s2 d
  | d < 0.0 || d > (1.0 + 1e-8) =
    error $ "Data.EuclideanSpace.interpolate's parameter must be in [0,1]" ++ show d
  | otherwise = let v = zipWith (-) s2 s1
                in  zipWith (+) s1 $ map (d*) v

getUniformSampler :: [Double] -> [Double] -> MP.StateSampler [Double]
getUniformSampler minState maxState =
  mapM CMR.getRandomR $ zip minState maxState

mkEuclideanSpace :: [Double] -> [Double] -> MP.StateSpace [Double]
mkEuclideanSpace minState maxState = MP.StateSpace
                                     stateDistance
                                     stateDistanceSqrd
                                     interpolate
                                     (getUniformSampler minState maxState)

{-# LANGUAGE CPP #-}

module Data.Spaces.EuclideanSpace
       ( mkEuclideanSpace
       , stateAsList
       , stateFromList
       ) where

import qualified Data.MotionPlanningProblem as MP
import qualified Data.Vector.Unboxed as V
import qualified Control.Monad.Random as CMR

lengthSqrd :: V.Vector Double -> Double
lengthSqrd s = V.sum $ V.zipWith (*) s s

stateDistanceSqrd ::  V.Vector Double -> V.Vector Double -> Double
stateDistanceSqrd s1 s2 = lengthSqrd $ V.zipWith (-) s2 s1

stateDistance :: V.Vector Double -> V.Vector Double -> Double
stateDistance s1 s2 = sqrt $ stateDistanceSqrd s1 s2

interpolate :: V.Vector Double -> V.Vector Double -> Double -> V.Vector Double
interpolate s1 s2 d
  | d < 0.0 || d > (1.0 + 1e-8) =
    error $ "Data.EuclideanSpace.interpolate's parameter must be in [0,1]" ++ show d
  | otherwise = let v = V.zipWith (-) s2 s1
                in  V.zipWith (+) s1 $ V.map (d*) v

getUniformSampler :: V.Vector Double
                  -> V.Vector Double
                  -> MP.StateSampler (V.Vector Double)
getUniformSampler minState maxState =
  V.mapM CMR.getRandomR $ V.zip minState maxState

mkEuclideanSpace :: [Double] -> [Double] -> MP.StateSpace (V.Vector Double)
mkEuclideanSpace minState maxState =
  MP.StateSpace
    stateDistance
    stateDistanceSqrd
    interpolate
    (getUniformSampler (V.fromList minState) (V.fromList maxState))

stateAsList :: V.Vector Double -> [Double]
stateAsList = V.toList

stateFromList :: [Double] -> V.Vector Double
stateFromList = V.fromList

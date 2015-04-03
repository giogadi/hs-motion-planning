module Data.Spaces.StandardSpace
       ( StandardSpace (..)
       , State (..)
       , stateDistance
       , stateDistanceSqrd
       , interpolate
       , getUniformSampler
       , mkStandardSpace
       ) where

import Control.Monad
import qualified Control.Monad.Random as CMR

import qualified Data.MotionPlanningProblem as MP

data StandardSpace = RealStateSpace Double Double
                   | SO2StateSpace
                   | CompoundStateSpace [(StandardSpace, Double)]

data State = RealState Double |
             SO2State Double |
             CompoundState [State] -- TODO try vector here
  deriving (Show)

validSO2 :: Double -> Bool
validSO2 x = x < pi && x >= -pi

stateDistanceSqrd :: StandardSpace -> MP.DistFn State
stateDistanceSqrd (RealStateSpace _ _) (RealState x) (RealState x') =
  let d = x - x'
  in  d * d
stateDistanceSqrd SO2StateSpace (SO2State x) (SO2State x')
  | validSO2 x && validSO2 x =
      let d = let dx = abs $ x - x'
              in  if dx > pi
                  then 2 * pi - dx
                  else dx
      in  d * d
  | otherwise = error "StandardSpace.stateDistance: invalid SO2 value!"
stateDistanceSqrd (CompoundStateSpace spacesAndWeights)
                  (CompoundState s) (CompoundState s') =
  let (spaces, weights) = unzip spacesAndWeights
  in  sum $
        zipWith (*) weights $ zipWith3 stateDistanceSqrd spaces s s'
stateDistanceSqrd _ _ _ =
  error "StandardSpace.stateDistanceSqrd: spaces and states don't line up!"

stateDistance :: StandardSpace -> MP.DistFn State
stateDistance space s s' = sqrt $ stateDistanceSqrd space s s'

interpolate :: MP.Interpolator State
interpolate _ _ d
  | d < 0 || d > 1.0 =
      error $ "StandardSpace.interpolate's parameter must be in [0,1]: " ++
        show d
interpolate (RealState x1) (RealState x2) d = RealState $ x1 + d * (x2 - x1)
interpolate (SO2State x1) (SO2State x2) d =
  let diff = x2 - x1
  in  if abs diff <= pi
      then SO2State $ x1 + d * diff
      else let diff' = signum diff * 2 * pi - diff
               v     = x1 - diff' * d
           in  if abs v > pi
               then SO2State $ v - signum v * 2 * pi
               else SO2State v
interpolate (CompoundState s1) (CompoundState s2) d =
  CompoundState $ zipWith3 interpolate s1 s2 $ repeat d
interpolate _ _ _ = error "StandardSpace.interpolate: mismatched state types!"

getUniformSampler :: StandardSpace -> MP.StateSampler State
getUniformSampler (RealStateSpace minVal maxVal) =
  liftM RealState $ CMR.getRandomR (minVal, maxVal)
getUniformSampler SO2StateSpace = do
  r <- CMR.getRandom
  return $ SO2State $ (-pi) + 2 * pi * r
getUniformSampler (CompoundStateSpace components) =
  let (spaces, _) = unzip components
  in  fmap CompoundState $ mapM getUniformSampler spaces

mkStandardSpace :: StandardSpace -> MP.StateSpace State
mkStandardSpace space =
  MP.StateSpace
    (stateDistance space)
    (stateDistanceSqrd space)
    interpolate
    (getUniformSampler space)

module Data.Spaces.Point2DSpace
       ( Point2D(..)
       , mkPoint2DSpace
       , stateDistance
       , stateDistanceSqrd
       , interpolate
       ) where

import qualified Data.MotionPlanningProblem as MP

import qualified Control.Monad.Random as CMR
import Control.Monad (liftM2)
import Control.DeepSeq

data Point2D = Point2D
               {-# UNPACK #-} !Double
               {-# UNPACK #-} !Double

instance Show Point2D where
    show (Point2D x y) = show x ++ " " ++ show y

-- For Criterion profiling
instance NFData Point2D where
  rnf (Point2D x y) = seq x $ seq y ()

stateDistanceSqrd :: Point2D -> Point2D -> Double
{-# INLINE stateDistanceSqrd #-}
stateDistanceSqrd (Point2D x1 y1) (Point2D x2 y2) =
    let v1 = x2 - x1
        v2 = y2 - y1
    in  v1*v1 + v2*v2

stateDistance :: Point2D -> Point2D -> Double
stateDistance p1 p2 = sqrt $ stateDistanceSqrd p1 p2

interpolate :: Point2D -> Point2D -> Double -> Point2D
interpolate (Point2D x1 y1) (Point2D x2 y2) d
  | d < 0.0 || d > (1.0 + 1e-8) =
    error $ "Data.Point2DSpace.interpolate's parameter must be in [0,1]" ++ show d
  | otherwise = let v1 = x2 - x1
                    v2 = y2 - y1
                in  Point2D (x1 + d*v1) (y1 + d*v2)

getUniformSampler :: Point2D -> Point2D -> MP.StateSampler Point2D
getUniformSampler (Point2D xmin ymin) (Point2D xmax ymax) =
    liftM2 Point2D
    (CMR.getRandomR (xmin, xmax))
    (CMR.getRandomR (ymin, ymax))

mkPoint2DSpace :: Point2D -> Point2D -> MP.StateSpace Point2D
mkPoint2DSpace pmin pmax = MP.StateSpace
                           stateDistance
                           stateDistanceSqrd
                           interpolate
                           (getUniformSampler pmin pmax)

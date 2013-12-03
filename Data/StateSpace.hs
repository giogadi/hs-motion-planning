module Data.StateSpace
       ( StateSpace(..)
       , UniformSampler
       ) where

import qualified Control.Monad.Random as CMR
import System.Random.Mersenne.Pure64 (PureMT)

type UniformSampler s = CMR.Rand PureMT s

data StateSpace s = StateSpace
                    { _stateDistance :: s -> s -> Double
                    , _fastNonMetricDistance :: s -> s -> Double
                    , _interpolate :: s -> s -> Double -> s
                    , _sampleUniform :: UniformSampler s
                    }

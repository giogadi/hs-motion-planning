module Data.StateSpace
       ( StateSpace(..)
       , StateSampler
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

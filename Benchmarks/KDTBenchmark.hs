import Data.Trees.KdTree

import Control.Monad
import qualified Control.Monad.Random as CMR
import Criterion.Main
import System.Random.Mersenne.Pure64

zeroOnePointSampler :: CMR.Rand PureMT Point2d
zeroOnePointSampler =
  liftM2 Point2d
    (CMR.getRandomR (0.0, 1.0))
    (CMR.getRandomR (0.0, 1.0))

main :: IO ()
main =
  let seed = 1
      numPoints = 100000
      randomPoints = CMR.evalRand (replicateM numPoints zeroOnePointSampler) $ pureMT seed
  in  defaultMain [
      bgroup "kdtree" [ bench "build-100000" $ nf
                          (buildKdTree mk2DEuclideanSpace) (zip randomPoints $ repeat ())
                      ]
      ]

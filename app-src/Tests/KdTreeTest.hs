import Data.Trees.KdTree as KDT
import Data.Trees.DynamicKdTree as DKDT

import Control.Monad
import System.Exit

main :: IO ()
main = do
  success <- liftM2 (&&) KDT.runTests DKDT.runTests
  when (not success) exitFailure

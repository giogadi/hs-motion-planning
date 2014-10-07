import Data.Trees.KdTree

import Control.Monad
import System.Exit

main :: IO ()
main = do
  success <- runTests
  when (not success) exitFailure

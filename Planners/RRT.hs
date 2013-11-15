module Planners.RRT
       ( RRT
       , solveRRT
       , solveRRTDefaultSeed
       , buildRRT
       , buildRRTDefaultSeed
       , getPathToGoal
       , getNumStates
       , writeRRT
         -- , rrtTests
       ) where

import Data.StateSpace
import Data.MotionPlanningProblem

import Data.List (minimumBy)
import Debug.Trace (trace)
import Data.Maybe (isJust, fromJust)
import System.Random (RandomGen, mkStdGen, StdGen)
import qualified Control.Monad.Random as CMR
import Data.List (foldl1', intercalate)
import Data.Foldable (foldr', sum, toList)
import Data.Function (on)
-- import qualified Test.QuickCheck as QC
-- import Test.Framework (testGroup)
-- import Test.Framework.Providers.QuickCheck2 (testProperty)

data Node s = Root s | Node s (Node s)

nodeState :: Node s -> s
nodeState (Root s) = s
nodeState (Node s _) = s

data RRT s g = RRT
    { _problem  :: MotionPlanningProblem s g
    , _stepSize :: Double
    , _nodes    :: [Node s]
    , _solution :: Maybe (Node s)}

getSpace :: RRT s g -> StateSpace s g
getSpace = _stateSpace . _problem

getNonMetricDist :: RRT s g -> (s -> s -> Double)
getNonMetricDist rrt = _fastNonMetricDistance $ getSpace rrt

getDist :: RRT s g -> (s -> s -> Double)
getDist rrt = _stateDistance $ getSpace rrt

getValidityFn :: RRT s g -> MotionValidityFn s
getValidityFn rrt = _motionValidity $ _problem rrt

getInterp :: RRT s g -> (s -> s -> Double -> s)
getInterp rrt = _interpolate $ getSpace rrt

getNumStates :: RRT s g -> Int
getNumStates = length . _nodes

writeRRT :: Show s => RRT s g -> String -> IO ()
writeRRT rrt fileName = writeFile fileName $ intercalate "\n" edgeStrings
    where edgeStrings = map stringFromEdge $ _nodes rrt
          stringFromEdge (Root _) = ""
          stringFromEdge (Node s p) = (show s) ++ " " ++ (show $ nodeState p)

-- A strict implementation of minimumBy
-- minimumBy' :: (a -> a -> Ordering) -> [a] -> a
-- minimumBy' cmp = foldl1' min'
--     where min' x y = case cmp x y of
--                        GT -> y
--                        _  -> x

nearestNode :: RRT s g -> s -> Node s
nearestNode rrt sample = let compareFn = compare `on` ((getNonMetricDist rrt $ sample) . nodeState)
                         in  minimumBy compareFn (_nodes rrt)

extendRRT :: RRT s g -> s -> RRT s g
extendRRT rrt sample =
    let near = nearestNode rrt sample
        nearState = nodeState near
        newState = let d = (getDist rrt) nearState sample
                   in  if d <= _stepSize rrt
                       then nearState
                       else (getInterp rrt) nearState sample $ ((_stepSize rrt) / d)
    in  if (getValidityFn rrt) nearState newState
        then let newNode = Node newState near
                 solution = if (_goalSatisfied $ _problem rrt) newState
                            then Just newNode
                            else Nothing
             in  RRT (_problem rrt) (_stepSize rrt) (newNode : (_nodes rrt)) solution
        else rrt

buildRRT :: RandomGen g => MotionPlanningProblem s g -> Double -> Int -> CMR.Rand g (RRT s g)
buildRRT problem stepSize numIterations =
    let start = _startState problem
        beginRRT = RRT problem stepSize [(Root start)] Nothing
    in  go beginRRT 0
    where
      go rrt iteration
        | iteration >= numIterations = return rrt
        | isJust $ _solution rrt = return rrt
        | otherwise = do
          newRRT <- (extendRRT rrt) `fmap` sample
          go newRRT (iteration + 1)
        where sample = _sampleUniform $ _stateSpace problem

getPathToGoal :: RandomGen g => RRT s g -> [s]
getPathToGoal rrt =
  case _solution rrt of
    Nothing -> []
    Just goalNode -> reverse $ go goalNode
  where go (Root s) = [s]
        go (Node s p) = s : (go p)

solveRRT :: RandomGen g => MotionPlanningProblem s g -> Double -> Int -> CMR.Rand g [s]
solveRRT problem stepSize numIterations =
  fmap getPathToGoal $ buildRRT problem stepSize numIterations

buildRRTDefaultSeed :: MotionPlanningProblem s StdGen -> Double -> Int -> RRT s StdGen
buildRRTDefaultSeed problem stepSize numIterations =
  CMR.evalRand (buildRRT problem stepSize numIterations) (mkStdGen 1)

solveRRTDefaultSeed :: MotionPlanningProblem s StdGen -> Double -> Int -> [s]
solveRRTDefaultSeed problem stepSize numIterations =
  CMR.evalRand (solveRRT problem stepSize numIterations) (mkStdGen 1)

-- --------------------------------------------------
-- -- Tests
-- --------------------------------------------------
-- prop_nonnegDist :: State -> State -> Bool
-- prop_nonnegDist s1 s2 = stateDistance s1 s2 >= 0.0

-- prop_squaredDist :: State -> State -> Bool
-- prop_squaredDist s1 s2 = abs ((stateDistance s1 s2)^2 - (stateDistanceSqrd s1 s2)) < 1e-5

-- prop_extendLimit :: State -> State -> QC.Positive Double -> Bool
-- prop_extendLimit s1 s2 (QC.Positive d) = let newState = extendTowardState s1 s2 d
--                                          in  stateDistance s1 newState <= d + 1e-7

-- rrtTests = testGroup "RRT tests" [
--             testProperty "Nonnegative distance" prop_nonnegDist,
--             testProperty "Squared distance" prop_squaredDist,
--             testProperty "Extend limit" prop_extendLimit]

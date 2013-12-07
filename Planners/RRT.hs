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

-- Moton planning imports
import Data.StateSpace
import Data.MotionPlanningProblem

-- Standard imports
import Data.Maybe (isJust)
import System.Random.Mersenne.Pure64 (PureMT, pureMT)
import qualified Control.Monad.Random as CMR
import Data.List (foldl1', intercalate)
import Data.Function (on)

data Node s = Root s | Node s (Node s)

nodeState :: Node s -> s
{-# INLINE nodeState #-}
nodeState (Root s) = s
nodeState (Node s _) = s

data RRT s = RRT
             { _problem  :: MotionPlanningProblem s
             , _stepSize :: Double
             , _nodes    :: [Node s]
             , _solution :: Maybe (Node s)
             }

getSpace :: RRT s -> StateSpace s
{-# INLINE getSpace #-}
getSpace = _stateSpace . _problem

getDistSqrd :: RRT s -> s -> s -> Double
{-# INLINE getDistSqrd #-}
getDistSqrd = _stateDistanceSqrd . getSpace

getDist :: RRT s -> s -> s -> Double
{-# INLINE getDist #-}
getDist = _stateDistance . getSpace

getValidityFn :: RRT s -> MotionValidityFn s
getValidityFn = _motionValidity . _problem

getInterp :: RRT s -> s -> s -> Double -> s
{-# INLINE getInterp #-}
getInterp = _interpolate . getSpace

getNumStates :: RRT s -> Int
getNumStates = length . _nodes

writeRRT :: Show s => RRT s -> String -> IO ()
writeRRT rrt fileName = writeFile fileName $ intercalate "\n" edgeStrings
    where edgeStrings = map stringFromEdge $ _nodes rrt
          stringFromEdge (Root _) = ""
          stringFromEdge (Node s p) = show s ++ " " ++ show (nodeState p)

-- A strict implementation of minimumBy
minimumBy' :: (a -> a -> Ordering) -> [a] -> a
minimumBy' cmp = foldl1' min'
    where
      min' x y = case cmp x y of
                       GT -> y
                       _  -> x

nearestNode :: RRT s -> s -> Node s
nearestNode rrt sample = let compareFn = compare `on` (getDistSqrd rrt sample . nodeState)
                         in  minimumBy' compareFn (_nodes rrt)

extendRRT :: RRT s -> s -> RRT s
extendRRT rrt sample =
    let near = nearestNode rrt sample
        nearState = nodeState near
        newState = let d = getDist rrt nearState sample
                   in  if d <= _stepSize rrt
                       then nearState
                       else getInterp rrt nearState sample $ _stepSize rrt / d
    in  if getValidityFn rrt nearState newState
        then let newNode = Node newState near
                 solution = if (_goalSatisfied $ _problem rrt) newState
                            then Just newNode
                            else Nothing
             in  RRT (_problem rrt) (_stepSize rrt) (newNode : _nodes rrt) solution
        else rrt

buildRRT :: MotionPlanningProblem s -> Double -> Int -> CMR.Rand PureMT (RRT s)
buildRRT problem stepSize numIterations =
    let start = _startState problem
        beginRRT = RRT problem stepSize [Root start] Nothing
    in  go beginRRT 0
    where
      go rrt iteration
        | iteration >= numIterations = return rrt
        | isJust $ _solution rrt = return rrt
        | otherwise = do
          newRRT <- extendRRT rrt `fmap` sample
          go newRRT (iteration + 1)
        where sample = _sampleUniform $ _stateSpace problem

getPathToGoal :: RRT s -> [s]
getPathToGoal rrt =
  case _solution rrt of
    Nothing -> []
    Just goalNode -> go goalNode []
  where go (Root s) path = s:path
        go (Node s p) path = go p $ s:path

solveRRT :: MotionPlanningProblem s -> Double -> Int -> CMR.Rand PureMT [s]
solveRRT problem stepSize numIterations =
  fmap getPathToGoal $ buildRRT problem stepSize numIterations

buildRRTDefaultSeed :: MotionPlanningProblem s -> Double -> Int -> RRT s
buildRRTDefaultSeed problem stepSize numIterations =
  CMR.evalRand (buildRRT problem stepSize numIterations) (pureMT 1)

solveRRTDefaultSeed :: MotionPlanningProblem s -> Double -> Int -> [s]
solveRRTDefaultSeed problem stepSize numIterations =
  CMR.evalRand (solveRRT problem stepSize numIterations) (pureMT 1)

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

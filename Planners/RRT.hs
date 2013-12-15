module Planners.RRT
       ( RRT
       , solveWithNN
       , solve
       -- , solveRRTDefaultSeed
       , buildRRTWithNN
       , buildRRT
       -- , buildRRTDefaultSeed
       , getPathToGoal
       , getNumStates
       , writeRRT
         -- , rrtTests
       ) where

-- Moton planning imports
import Data.MotionPlanningProblem
import Data.NearestNeighbors

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

data RRT s n = RRT
               { _space    :: StateSpace s
               , _query    :: MotionPlanningQuery s
               , _valid    :: MotionValidity s
               , _stepSize :: Double
               , _nodes    :: n
               , _solution :: Maybe (Node s)
               }

getDistSqrd :: RRT s n -> s -> s -> Double
{-# INLINE getDistSqrd #-}
getDistSqrd = _stateDistanceSqrd . _space

getDist :: RRT s n -> s -> s -> Double
{-# INLINE getDist #-}
getDist = _stateDistance . _space

getInterp :: RRT s n -> s -> s -> Double -> s
{-# INLINE getInterp #-}
getInterp = _interpolate . _space

getNumStates :: (NN n) => RRT s (n s (Node s)) -> Int
getNumStates = size . _nodes

writeRRT :: (NN n, Show s) => RRT s (n s (Node s)) -> String -> IO ()
writeRRT rrt fileName = writeFile fileName $ intercalate "\n" edgeStrings
    where edgeStrings = map (stringFromEdge . snd) $ toList $ _nodes rrt
          stringFromEdge (Root _) = ""
          stringFromEdge (Node s p) = show s ++ " " ++ show (nodeState p)

extendRRT :: NN n => RRT s (n s (Node s)) -> s -> RRT s (n s (Node s))
extendRRT rrt sample =
    let near = snd $ nearest (_nodes rrt) sample
        nearState = nodeState near
        newState = let d = getDist rrt nearState sample
                   in  if d <= _stepSize rrt
                       then nearState
                       else getInterp rrt nearState sample $ _stepSize rrt / d
    in  if _valid rrt nearState newState
        then let newNode = Node newState near
                 solution = if (_goalSatisfied $ _query rrt) newState
                            then Just newNode
                            else Nothing
             in  RRT { _space = _space rrt
                     , _query = _query rrt
                     , _valid = _valid rrt
                     , _stepSize = _stepSize rrt
                     , _nodes = insert (_nodes rrt) newState newNode
                     , _solution = solution
                     }
        else rrt

buildRRTWithNN :: NN n => StateSpace s -> MotionPlanningQuery s -> MotionValidity s -> n s (Node s) -> Double -> Int -> CMR.Rand PureMT (RRT s (n s (Node s)))
buildRRTWithNN space query valid emptyNN stepSize numIterations =
    let start = _startState query
        beginRRT = RRT space query valid stepSize (insert emptyNN start (Root start)) Nothing
    in  go beginRRT 0
    where
      go rrt iteration -- TODO try having rrt argument be (m rrt)?
        | iteration >= numIterations = return rrt
        | isJust $ _solution rrt = return rrt
        | otherwise = do
          newRRT <- extendRRT rrt `fmap` sample
          go newRRT (iteration + 1)
        where sample = _sampleUniform space

buildRRT :: StateSpace s -> MotionPlanningQuery s -> MotionValidity s -> Double -> Int -> CMR.Rand PureMT (RRT s (LinearNN s (Node s)))
buildRRT space query valid stepSize numIterations =
  buildRRTWithNN space query valid (mkLinearNN space) stepSize numIterations

getPathToGoal :: RRT s n-> [s]
getPathToGoal rrt =
  case _solution rrt of
    Nothing -> []
    Just goalNode -> go goalNode []
  where go (Root s) path = s:path
        go (Node s p) path = go p $ s:path

solveWithNN :: NN n => StateSpace s -> MotionPlanningQuery s -> MotionValidity s -> n s (Node s) -> Double -> Int -> CMR.Rand PureMT [s]
solveWithNN space query motionValidity nn stepSize numIterations =
  fmap getPathToGoal $ buildRRTWithNN space query motionValidity nn stepSize numIterations

solve :: StateSpace s -> MotionPlanningQuery s -> MotionValidity s -> Double -> Int -> CMR.Rand PureMT [s]
solve space query motionValidity stepSize numIterations =
  fmap getPathToGoal $ buildRRT space query motionValidity stepSize numIterations

-- buildRRTDefaultSeed :: StateSpace s -> MotionPlanningQuery s -> MotionValidity s -> Double -> Int -> RRT s
-- buildRRTDefaultSeed space query motionValidity stepSize numIterations =
--   CMR.evalRand (buildRRT space query motionValidity stepSize numIterations) (pureMT 1)

-- solveRRTDefaultSeed :: StateSpace s -> MotionPlanningQuery s -> MotionValidity s -> Double -> Int -> [s]
-- solveRRTDefaultSeed space query motionValidity stepSize numIterations =
--   CMR.evalRand (solveRRT space query motionValidity stepSize numIterations) (pureMT 1)

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

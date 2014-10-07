module Planners.RRT
       ( RRT
       , solve
       , buildRRT
       , buildRRTDefault
       , getPathToGoal
       , getNumStates
       , getStates
       , writeRRT
       ) where

import Data.MotionPlanningProblem
import Data.NearestNeighbors

import Data.Maybe (isJust)
import System.Random.Mersenne.Pure64 (PureMT)
import qualified Control.Monad.Random as CMR
import Data.List (intercalate)

data Node s = Root s | Node s (Node s)

nodeState :: Node s -> s
nodeState (Root s) = s
nodeState (Node s _) = s

type RRTNN n s = n s (Node s)

data RRT s n = RRT
               { _space    :: StateSpace s
               , _query    :: MotionPlanningQuery s
               , _valid    :: MotionValidity s
               , _stepSize :: Double
               , _nodes    :: RRTNN n s
               , _solution :: Maybe (Node s)
               }

getDist :: RRT s n -> s -> s -> Double
getDist = _stateDistance . _space

getInterp :: RRT s n -> s -> s -> Double -> s
getInterp = _interpolate . _space

getNumStates :: (NN n) => RRT s n -> Int
getNumStates = size . _nodes

getStates :: (NN n) => RRT s n -> [s]
getStates = map fst . toList . _nodes

writeRRT :: (NN n, Show s) => RRT s n -> String -> IO ()
writeRRT rrt fileName = writeFile fileName $ intercalate "\n" edgeStrings
    where edgeStrings = map (stringFromEdge . snd) $ toList $ _nodes rrt
          stringFromEdge (Root _) = ""
          stringFromEdge (Node s p) = show s ++ " " ++ show (nodeState p)

extendRRT :: NN n => RRT s n -> s -> RRT s n
extendRRT rrt sample =
    let nearNode = snd $ nearest (_nodes rrt) sample
        nearState = nodeState nearNode
        newState = let d = getDist rrt nearState sample
                   in  if d <= _stepSize rrt
                       then sample
                       else getInterp rrt nearState sample $ _stepSize rrt / d
    in  if _valid rrt nearState newState
        then let newNode = Node newState nearNode
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

buildRRT :: NN n => StateSpace s ->
                    MotionPlanningQuery s ->
                    MotionValidity s ->
                    RRTNN n s ->
                    Double ->
                    Int ->
                    CMR.Rand PureMT (RRT s n)
buildRRT space query valid emptyNN stepSize numIterations =
    let start = _startState query
        beginRRT = RRT space query valid stepSize (insert emptyNN start (Root start)) Nothing
    in  go beginRRT 0
    where
      go rrt iteration
        | iteration >= numIterations = return rrt
        | isJust $ _solution rrt = return rrt
        | otherwise = do
          newRRT <- extendRRT rrt `fmap` sample
          go newRRT (iteration + 1)
      sample = _sampleUniform space

buildRRTDefault :: StateSpace s ->
                   MotionPlanningQuery s ->
                   MotionValidity s ->
                   Double ->
                   Int ->
                   RRT s LinearNN
buildRRTDefault space query valid stepSize numIterations =
  evalDefaultSeed $ buildRRT space query valid (mkLinearNN space) stepSize numIterations

getPathToGoal :: RRT s n -> [s]
getPathToGoal rrt =
  case _solution rrt of
    Nothing -> []
    Just goalNode -> go goalNode []
  where go (Root s) path = s:path
        go (Node s p) path = go p $ s:path

solve :: StateSpace s -> MotionPlanningQuery s -> MotionValidity s -> Double -> Int -> [s]
solve space query motionValidity stepSize numIterations =
  getPathToGoal $ buildRRTDefault space query motionValidity stepSize numIterations

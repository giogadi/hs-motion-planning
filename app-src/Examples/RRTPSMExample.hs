import Data.MotionPlanningProblem
import Data.Spaces.StandardSpace
import Planners.RRT

fk :: [(Double, Double)] -> [(Double, Double)]
fk links =
  let (jointAngles, linkLengths) = unzip links
      angleSums = scanl1 (+) jointAngles
      xs = scanl1 (+) $ zipWith (*) linkLengths $ map cos angleSums
      ys = scanl1 (+) $ zipWith (*) linkLengths $ map sin angleSums
  in  zip xs ys

isCollisionFree :: [(Double, Double)] -> Bool
isCollisionFree links =
  let jointPositions = fk links
      jointInBounds (_, y) = y < 1.0 && y > -1.0
  in  all jointInBounds jointPositions

goalReached :: [(Double, Double)] -> (Double, Double) -> Double -> Bool
goalReached links (gx, gy) goalRadius =
  let (x, y) = last $ fk links
      dx = x - gx
      dy = y - gy
  in  dx*dx + dy*dy <= goalRadius*goalRadius

toState :: [Double] -> State
toState = CompoundState . map SO2State

fromState :: State -> [Double]
fromState (CompoundState so2States) =
  map (\(SO2State a) -> a) so2States

main :: IO ()
main =
  let linkLengths = [0.8, 0.8]
      ss = mkRotationalVectorSpace $ length linkLengths
      goalSatisfied s =
        goalReached (zip (fromState s) linkLengths) (-1.6, 0.0) 0.01
      q = MotionPlanningQuery
          { _startState = toState [0.0, 0.0]
          , _goalSatisfied = goalSatisfied
          }
      stateValid s = isCollisionFree $ zip (fromState s) linkLengths
      motionValid = discreteMotionValid ss stateValid 0.03
      rrt = buildRRTDefault ss q motionValid 0.3 10000
      motionPlan = getPathToGoal rrt
  in  do
    print $ fk (zip [0.0, 0.0] linkLengths)
    putStrLn $ "Computed a motion plan with " ++ show (Prelude.length motionPlan) ++ " states."
    putStrLn $ "Num states in tree: " ++ show (getNumStates rrt)
    putStrLn "Plan:"
    mapM_ print motionPlan

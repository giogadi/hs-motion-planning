import Data.MotionPlanningProblem
import Data.NearestNeighbors
import Data.Spaces.EuclideanSpace
import Planners.RRT

main :: IO ()
main = let minState = [0, 0, 0]
           maxState = [1, 1, 1]
           ss = mkEuclideanSpace minState maxState
           q = MotionPlanningQuery
               { _startState = stateFromList minState
               , _goalSatisfied = goalStateSatisfied ss 0.0 (stateFromList maxState)
               }
           valid _ _ = True
           kdt = mkKdTreeNN ss stateAsList
           -- rrt = buildRRTDefault ss q valid 0.1 1000
           rrt = evalDefaultSeed $ buildRRT ss q valid kdt 0.01 100000
           motionPlan = getPathToGoal rrt
       in do
         putStrLn $ "Computed a motion plan with " ++ show (Prelude.length motionPlan) ++ " states."
         putStrLn $ "Num states in tree: " ++ show (getNumStates rrt)
         putStrLn "Plan:"
         mapM_ print motionPlan

import Data.MotionPlanningProblem
import Data.Spaces.EuclideanSpace (mkEuclideanSpace)
import Planners.RRT

import Data.FixedList

main :: IO ()
main = let minState = 0.0 :. 0.0 :. 0.0 :. Nil
           maxState = 1.0 :. 1.0 :. 1.0 :. Nil
           ss = mkEuclideanSpace minState maxState
           q = MotionPlanningQuery
               { _startState = minState
               , _goalSatisfied = goalStateSatisfied ss 0.2 maxState
               }
           valid _ _ = True
           rrt = evalDefaultSeed $ buildRRT ss q valid 0.1 1000
           motionPlan = getPathToGoal rrt
       in do
         putStrLn $ "Computed a motion plan with " ++ show (Prelude.length motionPlan) ++ " states."
         putStrLn $ "Num states in tree: " ++ show (getNumStates rrt)
         putStrLn "Plan:"
         mapM_ print motionPlan

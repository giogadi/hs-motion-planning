import Data.MotionPlanningProblem
import Data.Spaces.Point2DSpace
import Planners.RRT

data Circle2D = Circle2D
  { _center :: Point2D
  , _radius :: Double
  }

pointOutsideCircle :: Circle2D -> Point2D -> Bool
pointOutsideCircle c p = stateDistanceSqrd p (_center c) > (_radius c)*(_radius c)

main :: IO ()
main = let minState = Point2D 0.0 0.0
           maxState = Point2D 1.0 1.0
           circleObs = Circle2D (Point2D 0.5 0.5) 0.25
           ss = mkPoint2DSpace minState maxState
           q = MotionPlanningQuery
               { _startState = minState
               , _goalSatisfied = goalStateSatisfied ss 0.1 maxState
               }
           valid = discreteMotionValid ss (pointOutsideCircle circleObs) 0.002
           rrt = evalDefaultSeed $ buildRRT ss q valid mkPoint2DKdTree 0.01 5000
           motionPlan = getPathToGoal rrt
       in do
         putStrLn $ "Computed a motion plan with " ++ show (Prelude.length motionPlan) ++ " states."
         putStrLn $ "Num states in tree: " ++ show (getNumStates rrt)
         putStrLn "Plan:"
         mapM_ print motionPlan

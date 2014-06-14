import Data.MotionPlanningProblem
import Data.Spaces.Point2DSpace
import Planners.RRT

import Criterion.Main

main :: IO ()
main = let minState = Point2D 0.0 0.0
           maxState = Point2D 1.0 1.0
           ss = mkPoint2DSpace minState maxState
           q = MotionPlanningQuery
               { _startState = minState
               , _goalSatisfied = goalStateSatisfied ss 0.0 maxState
               }
           valid _ _ = True
       in  defaultMain [bench "rrt" $ nf (solve ss q valid 0.01) 5000]

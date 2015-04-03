import Data.MotionPlanningProblem
--import Data.Spaces.Point2DSpace
import Data.Spaces.StandardSpace
import Planners.RRT

mkPoint2DSpace :: (Double, Double) -> (Double, Double) -> StateSpace State
mkPoint2DSpace (minX, minY) (maxX, maxY) =
  let pointSpace = CompoundStateSpace [
                     (RealStateSpace minX maxX, 1.0),
                     (RealStateSpace minY maxY, 1.0)
                   ]
  in  mkStandardSpace pointSpace

fromPoint2DToPair :: State -> (Double, Double)
fromPoint2DToPair (CompoundState [(RealState x), (RealState y)]) =
  (x, y)

fromPairToPoint2D :: (Double, Double) -> State
fromPairToPoint2D (x, y) = CompoundState [(RealState x), (RealState y)]

data Circle2D = Circle2D
  { _center :: (Double, Double)
  , _radius :: Double
  }

pointOutsideCircle :: Circle2D -> State -> Bool
pointOutsideCircle c s =
  let (x, y) = fromPoint2DToPair s
      (cx, cy) = _center c
      dx = x - cx
      dy = y - cy
      r = _radius c
  in  dx*dx + dy*dy > r*r

main :: IO ()
main = let minState = (0.0, 0.0)
           maxState = (1.0, 1.0)
           circleObs = Circle2D (0.5, 0.5) 0.25
           ss = mkPoint2DSpace minState maxState
           q = MotionPlanningQuery
               { _startState = fromPairToPoint2D minState
               , _goalSatisfied =
                   goalStateSatisfied ss 0.1 $ fromPairToPoint2D maxState
               }
           valid = discreteMotionValid ss (pointOutsideCircle circleObs) 0.002
           rrt = buildRRTDefault ss q valid 0.01 5000
           motionPlan = getPathToGoal rrt
       in do
         putStrLn $ "Computed a motion plan with " ++ show (Prelude.length motionPlan) ++ " states."
         putStrLn $ "Num states in tree: " ++ show (getNumStates rrt)
         putStrLn "Plan:"
         mapM_ print motionPlan

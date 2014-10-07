import Data.MotionPlanningProblem
import Data.Spaces.Point2DSpace
import Planners.PRM
import Data.Monoid

data Circle2D = Circle2D
  { _center :: Point2D
  , _radius :: Double
  }

pointOutsideCircle :: Circle2D -> Point2D -> Bool
pointOutsideCircle c p = stateDistanceSqrd p (_center c) > (_radius c)*(_radius c)

pathLengthMotionCost :: StateSpace s -> MotionCost s (Sum Double)
pathLengthMotionCost ss s1 s2 = Sum $ _stateDistance ss s1 s2

main :: IO ()
main = let minState = Point2D 0.0 0.0
           maxState = Point2D 1.0 1.0
           circleObs = Circle2D (Point2D 0.5 0.5) 0.25
           ss = mkPoint2DSpace minState maxState
           q = MotionPlanningQuery
               { _startState = minState
               , _goalSatisfied = goalStateSatisfied ss 0.3 maxState
               }
           valid = discreteMotionValid ss (pointOutsideCircle circleObs) 0.002
           rrg = evalDefaultSeed $ buildRRG ss valid (pathLengthMotionCost ss) 0.05 0.1 500 minState
           -- rrg = buildKRRGDefaultSeed ss valid (pathLengthMotionCost ss) 0.05 1 300 minState
           -- rrg = buildPRMStarDefaultSeed ss valid (pathLengthMotionCost ss) 0.1 300
           motionPlan = solve rrg q
       in do
         putStrLn $ "Computed a motion plan with " ++ show (Prelude.length motionPlan) ++ " states."
         putStrLn $ "Plan cost: " ++ show (getSum $ pathCost (pathLengthMotionCost ss) motionPlan)
         putStrLn "Plan:"
         mapM_ print motionPlan
         writeFile "/Users/luis/Desktop/rrg-test.txt" $ printEdges rrg

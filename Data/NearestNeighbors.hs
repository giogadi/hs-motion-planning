module Data.NearestNeighbors
       ( NN(..)
       , LinearNN
       , mkLinearNN
       ) where

-- Motion planning imports
import Data.MotionPlanningProblem

-- Standard imports
import Data.List (minimumBy, sortBy)
import Data.Function (on)

class NN n where
  insert   :: n k d -> k -> d -> n k d
  nearest  :: n k d -> k -> (k,d)
  nearestK :: n k d -> Int -> k -> [(k,d)]
  nearestR :: n k d -> Double -> k -> [(k,d)]
  size     :: n k d -> Int
  toList   :: n k d -> [(k,d)]

data LinearNN k d = LinearNN (DistFn k) [(k,d)]

instance NN LinearNN where
  insert (LinearNN dist elems) k d = LinearNN dist $ (k,d) : elems

  nearest (LinearNN dist elems) k = minimumBy near elems
    where near = compare `on` (dist k . fst)

  nearestK (LinearNN dist elems) k q = take k $ sortBy near elems
    where near = compare `on` (dist q . fst)

  nearestR (LinearNN dist elems) r q = filter withinRadius elems
    where withinRadius (e,_) = (dist q e) <= r*r

  size (LinearNN _ elems) = length elems

  toList (LinearNN _ elems) = elems

mkLinearNN :: StateSpace s -> LinearNN s d
mkLinearNN ss = LinearNN (_stateDistanceSqrd ss) []

-- A strict implementation of minimumBy
-- minimumBy' :: (a -> a -> Ordering) -> [a] -> a
-- minimumBy' cmp = foldl1' min'
--     where
--       min' x y = case cmp x y of
--                        GT -> y
--                        _  -> x

module Data.NearestNeighbors
       ( NN(..)
       , LinearNN(..)
       , mkLinearNN
       ) where

-- Motion planning imports
import Data.MotionPlanningProblem

-- Standard imports
import Data.List (minimumBy)
import Data.Function (on)

class NN n where
  insert  :: n k d -> k -> d -> n k d
  nearest :: n k d -> k -> (k,d)
  size    :: n k d -> Int
  toList  :: n k d -> [(k,d)]

data LinearNN k d = LinearNN (DistFn k) [(k,d)]

instance NN LinearNN where
  insert (LinearNN dist elems) k d = LinearNN dist $ (k,d) : elems

  nearest (LinearNN dist elems) k = minimumBy near elems
    where near = compare `on` (dist k . fst)

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

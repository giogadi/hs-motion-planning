module Data.NearestNeighbors
       ( NN(..)
       , LinearNN
       , mkLinearNN
       ) where

import Data.MotionPlanningProblem

import Data.List (sortBy, foldl')
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

  -- TODO why is this nice way just so much slower? :(
  -- nearest (LinearNN dist elems) k = minimumBy near elems
  --   where near = compare `on` (dist k . fst)
  nearest (LinearNN dist (e : es)) k = fst $ foldl' f (e, dist k $ fst e) es
    where {-# INLINE f #-}
          f b@(_, dBest) x
            | d < dBest = (x, d)
            | otherwise = b
            where d = dist k $ fst x
  nearest (LinearNN _ []) _ = error "LinearNN.nearest was called on an structure!"

  nearestK (LinearNN dist elems) k q = take k $ sortBy near elems
    where near = compare `on` (dist q . fst)

  nearestR (LinearNN dist elems) r q = filter withinRadius elems
    where withinRadius (e,_) = dist q e <= r*r

  size (LinearNN _ elems) = length elems

  toList (LinearNN _ elems) = elems

mkLinearNN :: StateSpace s -> LinearNN s d
mkLinearNN ss = LinearNN (_stateDistanceSqrd ss) []
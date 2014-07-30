-- Copyright (c)2011, Issac Trotts
--
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
--     * Redistributions of source code must retain the above copyright
--       notice, this list of conditions and the following disclaimer.
--
--     * Redistributions in binary form must reproduce the above
--       copyright notice, this list of conditions and the following
--       disclaimer in the documentation and/or other materials provided
--       with the distribution.
--
--     * Neither the name of Issac Trotts nor the names of other
--       contributors may be used to endorse or promote products derived
--       from this software without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
-- "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
-- LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
-- A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
-- OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
-- SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
-- LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
-- DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
-- THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
-- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
-- OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

-- Modifications made by Luis G. Torres.

module Data.Trees.KdTree
       ( EuclideanSpace (..)
       , compareDistance
       , KdTree
       , fromList
       , toList
       , nearestNeighbor
       , nearNeighbors
       , kNearestNeighbors
       , allSubtreesAreValid
       , mk2DEuclideanSpace
       ) where

import Data.Maybe

import qualified Data.List as L
import Test.QuickCheck

data EuclideanSpace p = EuclideanSpace
                        { _dimension :: Int
                        , _coord     :: Int -> p -> Double
                        , _dist2     :: p -> p -> Double
                        }

compareDistance :: EuclideanSpace p -> p -> p -> p -> Ordering
compareDistance s p a b = (_dist2 s) p a `compare` (_dist2 s) p b

data Tree p d = TreeNode { kdLeft :: Tree p d,
                           kdPoint :: (p, d),
                           kdRight :: Tree p d,
                           kdAxis :: Int }
                | TreeEmpty

data KdTree p d = KdTree (EuclideanSpace p) (Tree p d)

fromList :: EuclideanSpace p -> [(p, d)] -> KdTree p d
fromList s points = KdTree s $ fromListWithDepth s points 0

-- |fromListWithDepth selects an axis based on depth so that the axis cycles
-- through all valid values.
fromListWithDepth :: EuclideanSpace p -> [(p, d)] -> Int -> Tree p d
fromListWithDepth _ [] _ = TreeEmpty
fromListWithDepth s points depth = node
    where axis = depth `mod` _dimension s

          -- Sort point list and choose median as pivot element
          sortedPoints =
              L.sortBy (\(a,_) (b,_) -> (_coord s) axis a `compare` (_coord s) axis b) points
          medianIndex = length sortedPoints `div` 2

          -- Create node and construct subtrees
          -- TODO: This can be done faster
          node = TreeNode { kdLeft = fromListWithDepth s (take medianIndex sortedPoints) (depth+1),
                            kdPoint = sortedPoints !! medianIndex,
                            kdRight = fromListWithDepth s (drop (medianIndex+1) sortedPoints) (depth+1),
                            kdAxis = axis }

-- TODO: just make KdTree foldable I guess
toListInternal :: Tree p d -> [(p, d)]
toListInternal t = go t []
 where go TreeEmpty = id
       go (TreeNode l p r _) = go l . (p :) . go r

toList :: KdTree p d -> [(p, d)]
toList (KdTree _ t) = toListInternal t

-- |subtrees t returns a list containing t and all its subtrees, including the
-- empty leaf nodes.
subtrees :: KdTree p d -> [KdTree p d]
subtrees t = go t []
 where go (KdTree _ TreeEmpty) = id
       go root@(KdTree s (TreeNode l _ r _)) = go (KdTree s l) . (root :) . go (KdTree s r)

-- |nearestNeighbor tree p returns the nearest neighbor of p in tree.
nearestNeighbor :: KdTree p d -> p -> Maybe (p, d)
nearestNeighbor (KdTree s t) = nearestNeighbor' t
 where nearestNeighbor' TreeEmpty _ = Nothing
       nearestNeighbor' (TreeNode TreeEmpty x TreeEmpty _) _ = Just x
       nearestNeighbor' (TreeNode l (p, d) r axis) probe =
         if xProbe <= xp then findNearest l r else findNearest r l
           where xProbe = (_coord s) axis probe
                 xp = (_coord s) axis p
                 findNearest tree1 tree2 =
                   let candidates1 = case nearestNeighbor' tree1 probe of
                                       Nothing -> [(p, d)]
                                       Just best1 -> [best1, (p, d)]
                       sphereIntersectsPlane = (xProbe - xp)^(2 :: Int) <= (_dist2 s) probe p
                       candidates2 = if sphereIntersectsPlane
                                     -- TODO don't concat
                                     then candidates1 ++ maybeToList (nearestNeighbor' tree2 probe)
                                     else candidates1
                   in  Just . L.minimumBy (\(a,_) (b,_) -> compareDistance s probe a b) $ candidates2

-- |nearNeighbors tree p returns all neighbors within distance r from p in tree.
nearNeighbors :: KdTree p d -> Double -> p -> [(p, d)]
nearNeighbors (KdTree s t) radius probe = nearNeighbors' t
 where nearNeighbors' TreeEmpty = []
       nearNeighbors' (TreeNode TreeEmpty (p, d) TreeEmpty _) =
         if (_dist2 s) p probe <= radius*radius
         then [(p, d)]
         else []
       nearNeighbors' (TreeNode l (p, d) r axis) =
         if xProbe <= xp
         then let nearest = maybePivot ++ nearNeighbors' l
              in  if xProbe + abs radius > xp
                  then nearNeighbors' r ++ nearest
                  else nearest
         else let nearest = maybePivot ++ nearNeighbors' r
              in  if xProbe - abs radius < xp
                  then nearNeighbors' l ++ nearest
                  else nearest
        where xProbe     = (_coord s) axis probe
              xp         = (_coord s) axis p
              maybePivot = if (_dist2 s) probe p <= radius * radius
                           then [(p, d)]
                           else []

-- |isValid tells whether the K-D tree property holds for a given tree.
-- Specifically, it tests that all points in the left subtree lie to the left
-- of the plane, p is on the plane, and all points in the right subtree lie to
-- the right.
isValid :: KdTree p d -> Bool
isValid (KdTree _ TreeEmpty) = True
isValid (KdTree s (TreeNode l (p, _) r axis)) = leftIsGood && rightIsGood
    where x = (_coord s) axis p
          leftIsGood = all ((<= x) . (_coord s) axis) (map fst $ toListInternal l)
          rightIsGood = all ((>= x) . (_coord s) axis) (map fst $ toListInternal r)

-- |allSubtreesAreValid tells whether the K-D tree property holds for the given
-- tree and all subtrees.
allSubtreesAreValid :: KdTree p d -> Bool
allSubtreesAreValid = all isValid . subtrees

-- |kNearestNeighbors tree k p returns the k closest points to p within tree.
-- TODO fucking horrible
kNearestNeighbors :: Eq p => KdTree p d -> Int -> p -> [(p, d)]
kNearestNeighbors (KdTree _ TreeEmpty) _ _ = []
kNearestNeighbors _ k _ | k <= 0 = []
kNearestNeighbors tree k probe = nearest : kNearestNeighbors tree' (k-1) probe
    where nearest = fromJust $ nearestNeighbor tree probe
          tree' = tree `remove` fst nearest

-- |remove t p removes the point p from t.
remove :: (Eq p) => KdTree p d -> p -> KdTree p d
remove (KdTree s t) pKill = KdTree s $ remove' t
 where remove' TreeEmpty = TreeEmpty
       remove' (TreeNode l (p, d) r axis) =
         if p == pKill
           then fromListWithDepth s (toListInternal l ++ toListInternal r) axis
           else if (_coord s) axis pKill <= (_coord s) axis p
                then TreeNode (remove' l) (p, d) r axis
                else TreeNode l (p, d) (remove' r) axis

data Point2d = Point2d Double Double deriving (Show, Eq)

mk2DEuclideanSpace :: EuclideanSpace Point2d
mk2DEuclideanSpace = EuclideanSpace
                     { _dimension = 2
                     , _coord = coord
                     , _dist2 = dist
                     }
 where coord 0 (Point2d x _) = x
       coord 1 (Point2d _ y) = y
       coord _ _ = error "Tried to access invalid coordinate of Point2d!"

       dist (Point2d x1 y1) (Point2d x2 y2) = let dx = x2 - x1
                                                  dy = y2 - y1
                                              in dx*dx + dy*dy

instance Arbitrary Point2d where
    arbitrary = do
        x <- arbitrary
        y <- arbitrary
        return (Point2d x y)

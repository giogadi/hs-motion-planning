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

{-# LANGUAGE DeriveGeneric #-}

module Data.Trees.KdTree
       ( EuclideanSpace (..)
       , compareDistance
       , KdTree
       , buildKdTree
       , toList
       , nearestNeighbor
       , nearNeighbors
       , kNearestNeighbors
       , allSubtreesAreValid
       , mk2DEuclideanSpace
       , Point2d (..)
       ) where

import Control.DeepSeq
import Control.DeepSeq.Generics (genericRnf)
import GHC.Generics

import Data.Function
import qualified Data.List as L
import Data.Maybe
import qualified Data.Set as S
import qualified Data.Vector as V
import Test.QuickCheck

data EuclideanSpace p = EuclideanSpace
                        { _dimension :: Int
                        , _coord     :: Int -> p -> Double
                        , _dist2     :: p -> p -> Double
                        } deriving Generic
instance NFData p => NFData (EuclideanSpace p) where rnf = genericRnf

compareDistance :: EuclideanSpace p -> p -> p -> p -> Ordering
compareDistance s p a b = (_dist2 s) p a `compare` (_dist2 s) p b

-- TODO - might not even need to store kdAxis
data Tree p d = TreeNode { kdLeft :: Tree p d,
                           kdPoint :: (p, d),
                           kdRight :: Tree p d,
                           kdAxis :: Int }
                | TreeEmpty
                deriving Generic
instance (NFData p, NFData d) => NFData (Tree p d) where rnf = genericRnf

data KdTree p d = KdTree (EuclideanSpace p) (Tree p d) deriving Generic
instance (NFData p, NFData d) => NFData (KdTree p d) where rnf = genericRnf

buildTree :: EuclideanSpace p -> [V.Vector (p, d)] -> Int -> Tree p d
buildTree s sortedByAxis axis
 | n == 0 = TreeEmpty
 | otherwise = let medianIx = n `div` (2 :: Int)
                   median = (sortedByAxis !! axis) V.! medianIx
                   split = _coord s axis $ fst median
                   partitionVec vec vAxis =
                     if vAxis == axis
                     then let (left, medAndRight) = V.splitAt medianIx vec
                          in  (left, V.tail medAndRight)
                     else let (left, medAndRight) = V.partition ((< split) . _coord s axis . fst) vec
                          in  case V.findIndex ((== split) . _coord s axis . fst) medAndRight of
                                Just ix -> let (l, r) = V.splitAt ix medAndRight
                                           in  (left, l V.++ V.tail r)
                                Nothing -> error "we done fucked up yo"
                   (leftPoints, rightPoints) = unzip $ zipWith partitionVec sortedByAxis [0..]
                   nextAxis = (axis + 1) `mod` _dimension s
               in  TreeNode
                   { kdLeft = (buildTree s leftPoints nextAxis)
                   , kdPoint = median
                   , kdRight = (buildTree s rightPoints nextAxis)
                   , kdAxis = nextAxis
                   }
  where n = V.length $ head sortedByAxis

buildKdTree :: EuclideanSpace p -> [(p, d)] -> KdTree p d
buildKdTree _ [] = error "Who wants an empty KdTree anyway?"
buildKdTree s ps = let sortByAxis points a = L.sortBy (compare `on` (_coord s a . fst)) points
                       sortedByAxis = map (V.fromList . sortByAxis ps) [0 .. (_dimension s - 1)]
                       pointToList p = zipWith (_coord s) [0 .. (_dimension s - 1)] (repeat p)
                       coordLists = L.transpose $ map (pointToList . fst) ps
                       uniqueCoords = map S.toList $ map S.fromList coordLists
                   in  if all ((== length ps) . length) uniqueCoords
                       then KdTree s (buildTree s sortedByAxis 0)
                       else error $ show $ map length uniqueCoords ++ [length ps]

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
remove t@(KdTree s _) pKill = let ps = toList t
                              in  buildKdTree s $ filter ((/= pKill) . fst) ps

data Point2d = Point2d Double Double deriving (Show, Eq, Generic)
instance NFData Point2d where rnf = genericRnf

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

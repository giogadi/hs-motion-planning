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
       , KdTree
       , buildKdTree
       , toList
       , nearestNeighbor
       -- , nearNeighbors
       -- , kNearestNeighbors
       , mk2DEuclideanSpace
       , Point2d (..)
       -- Tests
       , checkValidTree
       , checkEqualToLinear
       ) where

import Control.DeepSeq
import Control.DeepSeq.Generics (genericRnf)
import GHC.Generics

import Data.Function
import qualified Data.List as L
import qualified Data.Vector as V
import Test.QuickCheck

data EuclideanSpace p = EuclideanSpace
                        { _dimension :: Int
                        , _coord     :: Int -> p -> Double
                        , _dist2     :: p -> p -> Double
                        } deriving Generic
instance NFData p => NFData (EuclideanSpace p) where rnf = genericRnf

incrementAxis :: EuclideanSpace s -> Int -> Int
incrementAxis s axis = (axis + 1) `mod` _dimension s

-- TODO - might not even need to store kdAxis
data TreeNode p d = TreeNode { kdLeft :: Maybe (TreeNode p d)
                             , kdPoint :: (p, d)
                             , kdRight :: Maybe (TreeNode p d)
                             }
                deriving Generic
instance (NFData p, NFData d) => NFData (TreeNode p d) where rnf = genericRnf

data KdTree p d = KdTree (EuclideanSpace p) (TreeNode p d) deriving Generic
instance (NFData p, NFData d) => NFData (KdTree p d) where rnf = genericRnf

buildTreeInternal :: EuclideanSpace p -> [V.Vector (p, d)] -> Int -> TreeNode p d
buildTreeInternal s sortedByAxis axis
  | n == 1 = TreeNode Nothing (V.head $ head sortedByAxis) Nothing
  | otherwise =
    let medianIx = n `div` 2 :: Int
        median   = (sortedByAxis !! axis) V.! medianIx
        splitVal = _coord s axis $ fst median
        partitionVec vec vAxis =
          if vAxis == axis
          then let (left, medAndRight) = V.splitAt medianIx vec
               in  (left, V.tail medAndRight)
          else let (left, medAndRight) = V.partition ((< splitVal) . _coord s axis . fst) vec
               in  case V.findIndex ((== splitVal) . _coord s axis . fst) medAndRight of
                     Just ix -> let (l, r) = V.splitAt ix medAndRight
                                in  (left, l V.++ V.tail r)
                     Nothing -> error "we done fucked up yo"
        (leftPoints, rightPoints) = unzip $ zipWith partitionVec sortedByAxis [0..]
    in  TreeNode
        { kdLeft = maybeBuildTree leftPoints
        , kdPoint = median
        , kdRight = maybeBuildTree rightPoints
        }
  where n = V.length $ head sortedByAxis
        maybeBuildTree ps
          | V.null $ head ps = Nothing
          | otherwise = Just $ buildTreeInternal s ps $ incrementAxis s axis

-- buildTree :: EuclideanSpace p -> [V.Vector (p, d)] -> Int -> Tree p d
-- buildTree s sortedByAxis axis
--  | n == 0 = TreeEmpty
--  | otherwise = let medianIx = n `div` (2 :: Int)
--                    median = (sortedByAxis !! axis) V.! medianIx
--                    split = _coord s axis $ fst median
--                    partitionVec vec vAxis =
--                      if vAxis == axis
--                      then let (left, medAndRight) = V.splitAt medianIx vec
--                           in  (left, V.tail medAndRight)
--                      else let (left, medAndRight) = V.partition ((< split) . _coord s axis . fst) vec
--                           in  case V.findIndex ((== split) . _coord s axis . fst) medAndRight of
--                                 Just ix -> let (l, r) = V.splitAt ix medAndRight
--                                            in  (left, l V.++ V.tail r)
--                                 Nothing -> error "we done fucked up yo"
--                    (leftPoints, rightPoints) = unzip $ zipWith partitionVec sortedByAxis [0..]
--                    nextAxis = (axis + 1) `mod` _dimension s
--                in  TreeNode
--                    { kdLeft = (buildTree s leftPoints nextAxis)
--                    , kdPoint = median
--                    , kdRight = (buildTree s rightPoints nextAxis)
--                    , kdAxis = nextAxis
--                    }
--   where n = V.length $ head sortedByAxis

buildKdTree :: EuclideanSpace p -> [(p, d)] -> KdTree p d
buildKdTree _ [] = error "Who wants an empty KdTree anyway?"
buildKdTree s ps = let sortByAxis points a = L.sortBy (compare `on` (_coord s a . fst)) points
                       sortedByAxis = map (V.fromList . sortByAxis ps) [0 .. (_dimension s - 1)]
                   in  KdTree s $ buildTreeInternal s sortedByAxis 0

-- TODO: just make KdTree foldable I guess
toListInternal :: TreeNode p d -> [(p, d)]
toListInternal t = go (Just t) []
  where go Nothing = id
        go (Just (TreeNode l p r)) = go l . (p :) . go r

toList :: KdTree p d -> [(p, d)]
toList (KdTree _ t) = toListInternal t

isTreeValid :: EuclideanSpace p -> Int -> TreeNode p d -> Bool
isTreeValid s axis (TreeNode l (p, _) r) =
  let xAxisVal = _coord s axis p
      nodeKey (TreeNode _ (p', _) _) = p'
      nextAxis = incrementAxis s axis
      leftChildValid = maybe True ((<= xAxisVal) . _coord s axis . nodeKey) l
      rightChildValid = maybe True ((> xAxisVal) . _coord s axis . nodeKey) r
      leftSubtreeValid = maybe True (isTreeValid s nextAxis) l
      rightSubtreeValid = maybe True (isTreeValid s nextAxis) r
  in  leftChildValid && rightChildValid && leftSubtreeValid && rightSubtreeValid

nearestNeighbor :: KdTree p d -> p -> (p, d)
nearestNeighbor (KdTree s t) = fst . go 0 t
  where
    go _ (TreeNode Nothing (x, d) Nothing) query = ((x, d), _dist2 s query x)
    go axis (TreeNode maybeLeft (x, d) maybeRight) query =
      let queryAxisValue = (_coord s) axis query
          xAxisValue     = (_coord s) axis x
          xDist          = _dist2 s query x
          nextAxis       = incrementAxis s axis
          -- "onside" refers to the child on the same side of this
          -- node's axis as the query.
          nearestInSubtrees maybeOnsideTree maybeOffsideTree =
            let maybeNearestOnside  = fmap ((flip $ go nextAxis) query) maybeOnsideTree
                nearest :: ((p, d), Double) -> Maybe ((p, d), Double) -> ((p, d), Double)
                nearest p Nothing = p
                nearest p@(_, dist1) (Just (q@(_, dist2))) =
                  if dist1 < dist2 then p else q
                nearestSoFar = nearest ((x, d), xDist) maybeNearestOnside
                checkOffsideTree = (queryAxisValue - xAxisValue)^(2 :: Int) < snd nearestSoFar
                maybeNearestOffside = if checkOffsideTree
                                      then fmap ((flip $ go nextAxis) query) maybeOffsideTree
                                      else Nothing
            in  nearest nearestSoFar maybeNearestOffside
      in  if queryAxisValue <= xAxisValue
          then nearestInSubtrees maybeLeft maybeRight
          else nearestInSubtrees maybeRight maybeLeft

-- -- |nearestNeighbor tree p returns the nearest neighbor of p in tree.
-- nearestNeighbor :: KdTree p d -> p -> Maybe (p, d)
-- nearestNeighbor (KdTree s t) = Just . fst . fromJust . nearestNeighbor' t
--  where nearestNeighbor' TreeEmpty _ = Nothing
--        nearestNeighbor' (TreeNode TreeEmpty x TreeEmpty _) probe = Just (x, _dist2 s probe $ fst x)
--        nearestNeighbor' (TreeNode l (p, d) r axis) probe =
--          if xProbe <= xp then findNearest l r else findNearest r l
--            where xProbe = (_coord s) axis probe
--                  xp = (_coord s) axis p
--                  pDist = _dist2 s probe p
--                  findNearest tree1 tree2 =
--                    let candidate1 = case nearestNeighbor' tree1 probe of
--                                       Nothing -> ((p, d), pDist)
--                                       Just (bestP, bestD) -> if bestD < pDist
--                                                              then (bestP, bestD)
--                                                              else ((p, d), pDist)
--                        sphereIntersectsPlane = (xProbe - xp)^(2 :: Int) <= pDist
--                    in  if sphereIntersectsPlane
--                        then let candidate2 = nearestNeighbor' tree2 probe
--                             in  case candidate2 of
--                                   Nothing -> Just candidate1
--                                   Just c2 -> if snd c2 < snd candidate1
--                                              then candidate2
--                                              else Just candidate1
--                        else Just candidate1

-- |nearNeighbors tree p returns all neighbors within distance r from p in tree.
-- nearNeighbors :: KdTree p d -> Double -> p -> [(p, d)]
-- nearNeighbors (KdTree s t) radius probe = nearNeighbors' t
--  where nearNeighbors' TreeEmpty = []
--        nearNeighbors' (TreeNode TreeEmpty (p, d) TreeEmpty _) =
--          if (_dist2 s) p probe <= radius*radius
--          then [(p, d)]
--          else []
--        nearNeighbors' (TreeNode l (p, d) r axis) =
--          if xProbe <= xp
--          then let nearest = maybePivot ++ nearNeighbors' l
--               in  if xProbe + abs radius > xp
--                   then nearNeighbors' r ++ nearest
--                   else nearest
--          else let nearest = maybePivot ++ nearNeighbors' r
--               in  if xProbe - abs radius < xp
--                   then nearNeighbors' l ++ nearest
--                   else nearest
--         where xProbe     = (_coord s) axis probe
--               xp         = (_coord s) axis p
--               maybePivot = if (_dist2 s) probe p <= radius * radius
--                            then [(p, d)]
--                            else []

-- |kNearestNeighbors tree k p returns the k closest points to p within tree.
-- TODO fucking horrible
-- kNearestNeighbors :: Eq p => KdTree p d -> Int -> p -> [(p, d)]
-- kNearestNeighbors (KdTree _ TreeEmpty) _ _ = []
-- kNearestNeighbors _ k _ | k <= 0 = []
-- kNearestNeighbors tree k probe = nearest : kNearestNeighbors tree' (k-1) probe
--     where nearest = fromJust $ nearestNeighbor tree probe
--           tree' = tree `remove` fst nearest

-- |remove t p removes the point p from t.
-- remove :: (Eq p) => KdTree p d -> p -> KdTree p d
-- remove t@(KdTree s _) pKill = let ps = toList t
--                               in  buildKdTree s $ filter ((/= pKill) . fst) ps

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

--------------------------------------------------------------------------------
-- Tests
--------------------------------------------------------------------------------

testElements :: [p] -> [(p, ())]
testElements ps = zip ps $ repeat ()

checkValidTree :: EuclideanSpace p -> [p] -> Bool
checkValidTree _ [] = True
checkValidTree s ps =
  let (KdTree _ t) = buildKdTree s $ testElements ps
  in  isTreeValid s 0 t

nearestNeighborLinear :: EuclideanSpace p -> [(p, d)] -> p -> (p, d)
nearestNeighborLinear s xs query =
  L.minimumBy (\(p1, _) (p2, _) -> compareDistance p1 p2) xs
  where compareDistance p1 p2 = _dist2 s query p1 `compare` _dist2 s query p2

checkEqualToLinear :: Eq p => EuclideanSpace p -> ([p], p) -> Bool
checkEqualToLinear _ ([], _) = True
checkEqualToLinear s (ps, query) =
  let kdt = buildKdTree s $ testElements ps
  in  nearestNeighbor kdt query == nearestNeighborLinear s (testElements ps) query

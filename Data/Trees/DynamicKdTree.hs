{-# LANGUAGE DeriveGeneric #-}

module Data.Trees.DynamicKdTree
       ( DkdTree
       , KDT.EuclideanSpace (..)
       , KDT.mk2DEuclideanSpace
       , emptyDkdTree
       , singleton
       , nearestNeighbor
       , kNearestNeighbors
       , insert
       , size
       , toList
       , batchInsert
       -- Begin Tests
       , checkLogNTrees
       , checkTreeSizesPowerOf2
       , checkNumElements
       , checkNearestEqualToBatch
       , checkKNearestEqualToBatch
       , checkEqualToLinear
       , checkKNearestSorted
       , module Test.QuickCheck
       ) where

import Data.Bits
import Data.List hiding (insert)
import Data.Maybe

import Control.DeepSeq
import Control.DeepSeq.Generics (genericRnf)
import GHC.Generics
import Test.QuickCheck hiding ((.&.))

import qualified Data.Trees.KdTree as KDT

data DkdTree p d = DkdTree
                  { _trees    :: [KDT.KdTree p d]
                  , _space    :: KDT.EuclideanSpace p
                  , _numNodes :: Int
                  } deriving Generic
instance (NFData p, NFData d) => NFData (DkdTree p d) where rnf = genericRnf

-- TODO remove this
emptyDkdTree :: KDT.EuclideanSpace p -> DkdTree p d
emptyDkdTree s = DkdTree [] s 0

singleton :: KDT.EuclideanSpace p -> (p, d) -> DkdTree p d
singleton s (p, d) = DkdTree [KDT.buildKdTree s [(p, d)]] s 1

nearestNeighbor :: DkdTree p d -> p -> Maybe (p, d)
nearestNeighbor (DkdTree ts s _) probe =
  let nearests = catMaybes $ map (flip KDT.nearestNeighbor probe) ts
  in  if   null nearests
      then Nothing
      else Just $ minimumBy (\(p1, _) (p2, _) -> KDT.compareDistance s probe p1 p2) nearests

insert :: DkdTree p d -> (p, d) -> DkdTree p d
insert (DkdTree trees s n) p =
  let bitList = map (((.&.) 1) . (n `shiftR`)) [0..]
      (onesPairs, theRestPairs) = span ((== 1) . fst) $ zip bitList trees
      ((_, ones), (_, theRest)) = (unzip onesPairs, unzip theRestPairs)
      newTree = KDT.buildKdTree s $ p : concatMap KDT.toList ones
  in  DkdTree (newTree : theRest) s $ n + 1

kNearestNeighbors :: Eq p => DkdTree p d -> Int -> p -> [(p, d)]
kNearestNeighbors (DkdTree trees s _) k probe =
  let neighborSets = map (\t -> KDT.kNearestNeighbors t k probe) trees
  in  take k $ foldr merge [] neighborSets
 where merge [] ys = ys
       merge xs [] = xs
       merge xs@(x:xt) ys@(y:yt)
         | distX <= distY = x : merge xt ys
         | otherwise      = y : merge xs yt
        where distX = (KDT._dist2 s) probe $ fst x
              distY = (KDT._dist2 s) probe $ fst y

size :: DkdTree p d -> Int
size (DkdTree _ _ n) = n

toList :: DkdTree p d -> [(p, d)]
toList (DkdTree trees _ _) = concatMap KDT.toList trees

batchInsert :: DkdTree p d -> [(p, d)] -> DkdTree p d
batchInsert t =  foldl' insert t

--------------------------------------------------------------------------------
-- Tests
--------------------------------------------------------------------------------

testElements :: [p] -> [(p, Int)]
testElements ps = zip ps [1..]

checkLogNTrees :: KDT.EuclideanSpace p -> [p] -> Bool
checkLogNTrees s ps = let lengthIsLogN (DkdTree ts _ n) = length ts == popCount n
                      in  all lengthIsLogN $ scanl insert (emptyDkdTree s) $ testElements ps

checkTreeSizesPowerOf2 :: KDT.EuclideanSpace p -> [p] -> Bool
checkTreeSizesPowerOf2 s ps =
  let sizesPowerOf2 (DkdTree ts _ _) = all (== 1) $ map (popCount . length . KDT.toList) ts
  in  all sizesPowerOf2 $ scanl insert (emptyDkdTree s) $ testElements ps

checkNumElements :: KDT.EuclideanSpace p -> [p] -> Bool
checkNumElements s ps =
  let numsMatch (num, DkdTree ts _ n) = n == num && n == (sum $ map (length . KDT.toList) ts)
  in  all numsMatch $ zip [0..] $ scanl insert (emptyDkdTree s) $ testElements ps

checkNearestEqualToBatch :: Eq p => KDT.EuclideanSpace p -> ([p], p) -> Bool
checkNearestEqualToBatch s (ps, query) =
  let kdt = KDT.buildKdTree s $ testElements ps
      kdtAnswer = KDT.nearestNeighbor kdt query
      dkdt = foldl' insert (emptyDkdTree s) $ testElements ps
      dkdtAnswer = nearestNeighbor dkdt query
  in  dkdtAnswer == kdtAnswer

checkKNearestEqualToBatch :: Eq p => KDT.EuclideanSpace p -> ([p], p) -> Bool
checkKNearestEqualToBatch s (ps, query) =
  let k = 10 -- TODO make this quick-checked and bounded by something reasonable
      kdt = KDT.buildKdTree s $ testElements ps
      kdtAnswer = KDT.kNearestNeighbors kdt k query
      dkdt = foldl' insert (emptyDkdTree s) $ testElements ps
      dkdtAnswer = kNearestNeighbors dkdt k query
  in  dkdtAnswer == kdtAnswer

nearestNeighborLinear :: KDT.EuclideanSpace p -> [(p, d)] -> p -> Maybe (p, d)
nearestNeighborLinear _ [] _ = Nothing
nearestNeighborLinear s xs probe =
  Just $ minimumBy (\(p1, _) (p2, _) -> KDT.compareDistance s probe p1 p2) xs

checkEqualToLinear :: Eq p => KDT.EuclideanSpace p -> ([p], p) -> Bool
checkEqualToLinear s (ps, query) =
  let linears = scanl (\xs x -> xs ++ [x]) [] $ testElements ps
      dkdts   = scanl insert (emptyDkdTree s) $ testElements ps
  in  map (\xs -> nearestNeighborLinear s xs query) linears == map (`nearestNeighbor` query) dkdts

checkKNearestSorted :: Eq p => KDT.EuclideanSpace p -> ([p], p) -> Bool
checkKNearestSorted s (ps, query) =
  let kdt = KDT.buildKdTree s $ testElements ps
      kNearestDists = map (KDT._dist2 s query . fst) $ KDT.kNearestNeighbors kdt (length ps) query
  in  kNearestDists == sort kNearestDists

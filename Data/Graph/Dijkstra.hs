module Data.Graph.Dijkstra
       ( dijkstra
       ) where

-- Graph library import
import Data.Graph.Inductive hiding (dijkstra)

-- Priority queue import
import qualified Data.PQueue.Prio.Min as PQ

-- Standard imports
import Data.List (find)
import Data.Maybe (fromJust)

type CostAccum c = c -> c -> c

-- Internal routine implementing Dijkstra's shortest paths
-- algorithm. Deemed internal because it needs to be kickstarted with
-- a singleton node queue.
dijkstraInternal ::
  (Graph gr, Ord b) => gr a b -> PQ.MinPQueue b [Node] -> CostAccum b -> [[Node]]
dijkstraInternal g q accum
  | PQ.null q = []
  | otherwise =
    case match v g of
      (Just cxt,g') -> p:dijkstraInternal  g' (PQ.unions (q' : expand cxt minDist p)) accum
      (Nothing, g') -> dijkstraInternal g' q' accum
  where ((minDist,p@(v:_)), q') = PQ.deleteFindMin q
        expand (_,_,_,s) dist pathToC =
          map (\(edgeCost,v) -> PQ.singleton (dist `accum` edgeCost) ((v:pathToC))) s

-- Given a graph, a start node, and a goal node, returns a list of
-- labeled nodes corresponding to the shortest path from the start to
-- the goal nodes, where the user specifies a function for
-- accumulating the edge labels, the identity cost value, and a
-- function to compare costs.
dijkstra :: (Graph gr, Ord b) => gr a b -> Node -> Node -> CostAccum b -> b -> [LNode a]
dijkstra g start goal accum identity =
  let paths = dijkstraInternal g (PQ.singleton identity [start]) accum
      pathNodes  = find (((==) goal) . head) paths -- Can paths be empty?
  in
   case pathNodes of
     Nothing -> []
     Just ps -> reverse $ map (\n -> (n, fromJust $ lab g n)) ps

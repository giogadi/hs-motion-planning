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
import Data.Monoid

-- Internal routine implementing Dijkstra's shortest paths
-- algorithm. Deemed internal because it needs to be kickstarted with
-- a singleton node queue.
dijkstraInternal ::
  (Graph gr, Ord b, Monoid b) => gr a b -> PQ.MinPQueue b [Node] -> [[Node]]
dijkstraInternal g q
  | PQ.null q = []
  | otherwise =
    case match v g of
      (Just cxt,g') -> p:dijkstraInternal  g' (PQ.unions (q' : expand cxt minDist p))
      (Nothing, g') -> dijkstraInternal g' q'
  where ((minDist,p@(v:_)), q') = PQ.deleteFindMin q
        expand (_,_,_,s) dist pathToC =
          map (\(edgeCost,n) -> PQ.singleton (dist `mappend` edgeCost) (n:pathToC)) s

-- Given a graph, a start node, and a goal node, returns a list of
-- labeled nodes corresponding to the shortest path from the start to
-- the goal nodes, where the edge costs are accumulated according to
-- the Monoid instance of the edge label type and costs are compared
-- by the edge label's Ord instance
dijkstra :: (Graph gr, Ord b, Monoid b) => gr a b -> Node -> Node -> [LNode a]
dijkstra g start goal =
  let paths = dijkstraInternal g (PQ.singleton `mempty` [start])
      pathNodes  = find ((goal ==) . head) paths -- Can paths be empty?
  in
   case pathNodes of
     Nothing -> []
     Just ps -> reverse $ map (\n -> (n, fromJust $ lab g n)) ps

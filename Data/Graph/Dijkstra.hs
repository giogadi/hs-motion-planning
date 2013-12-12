module Data.Graph.Dijkstra
       ( dijkstra
       , dijkstraPath
       ) where

-- Graph library import
import Data.Graph.Inductive hiding (dijkstra)

-- Priority queue import
import qualified Data.PQueue.Prio.Min as PQ

-- Standard imports
import Data.List (find)
import Data.Maybe (fromJust)
import Data.Monoid
import Debug.Trace

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

-- Given a graph and a start node, returns a list of lists of nodes
-- corresponding to the shortest paths from the start to all other
-- nodes, where the edge costs are accumulated according to the Monoid
-- instance of the edge label type and costs are compared by the edge
-- label's Ord instance.
dijkstra :: (Graph gr, Ord b, Monoid b) => gr a b -> Node -> [[Node]]
dijkstra g start = dijkstraInternal g (PQ.singleton mempty [start])

dijkstraPath :: (Graph gr, Ord b, Monoid b) => gr a b -> Node -> Node -> [LNode a]
dijkstraPath g start goal =
  let paths = dijkstra g start
      pathNodes  = find ((goal ==) . head) paths -- Can paths be empty?
  in
   case pathNodes of
     Nothing -> []
     Just ps -> reverse $ map (\n -> (n, fromJust $ lab g n)) ps

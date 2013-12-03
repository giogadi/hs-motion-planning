module Data.Cost
       where

data CostSpace c s = CostSpace
  { _motionCost :: s -> s -> c
  , _accumCost  :: c -> c -> c
  , _idCost     :: c
  , _infCost    :: c
  , _cmp        :: c -> c -> Bool -- Returns true if first argument is
                                  -- strictly better than second
  }

pathCost :: [s] -> CostSpace c s -> c
pathCost [] _ =
  error "Data.Cost.pathCost must be called with a path consisting of at least two states."
pathCost [s] _ =
  error "Data.Cost.pathCost must be called with a path consisting of at least two states."
pathCost p@(_:t) cs = foldr1 (_accumCost cs) $ zipWith (_motionCost cs) p t

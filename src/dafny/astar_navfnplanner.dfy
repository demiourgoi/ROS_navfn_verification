/*
 * Implementation and verification of NavFn planner in Dafny
 */

/*
 * ----------
 * Data types
 * ----------
 */
datatype Point = Point(row: int, col: int)                                        // Tile location in a discrete map
datatype RealPoint = RealPoint(row: real, col: real)                              // Continuous location
datatype OffsetPoint = OffsetPoint(base: Point, offset: RealPoint)                // Base tile + offset
datatype Pose = Pose(pos: Point)
datatype CostMap = CostMap(value: Point -> real, numRows: nat, numCols: nat)      // CostMap assigns a cost to each tile
datatype RealInf = Real(r: real) | Infinity                                       // Set R ∪ {+\infty}


type Path = seq<RealPoint>

// Matrix with potentials. A finite potential implies no obstacles
// The lower the potential, the closer to the goal
type PotentialMap = seq<seq<RealInf>>

const obstacleCost: real := 254.0
const mapCost: real := 50.0
const stepSize: real := 0.5

/* 
 * ------------------------------------------------------------------------------------
 *                           Operations on R ∪ {+\infty}
 * ------------------------------------------------------------------------------------
 */


/*
 * Returns the smallest of two elements
 */
function method MinInfinity(x1: RealInf, x2: RealInf): RealInf
  ensures MinInfinity(x1, x2) == x1 || MinInfinity(x1, x2) == x2
{
  if (x1.Infinity?) then x2
  else if (x2.Infinity?) then x1
  else if (x1.r <= x2.r) then x1 else x2
}

/*
 * Returns the greatest of two elements
 */
function method MaxInfinity(x1: RealInf, x2: RealInf): RealInf {
  if (x1.Infinity?) then x1
  else if (x2.Infinity?) then x2
  else if (x1.r >= x2.r) then x1 else x2
}

/*
 * Returns true iff x1 < x2
 */
predicate method GreaterThan(x1: RealInf, x2: RealInf)
  requires x1.Real? || x2.Real?
{
  x1.Infinity? || (x2.Real? && x1.r > x2.r)
}

/*
 * Returns true iff x1 <= x2
 */
predicate method GreaterThanOrEqual(x1: RealInf, x2: RealInf)
{
  x1.Infinity? || (x1.Real? && x2.Real? && x1.r >= x2.r)
}

/*
 * Substraction operator: x1 - x2
 * It requires x2 to be finite to avoid indetermination or -\infty
 */
function method Minus(x1: RealInf, x2: RealInf): RealInf
  requires x2.Real?
{
  if (x1.Infinity?) then Infinity else Real(x1.r - x2.r)
}


/* 
 * ------------------------------------------------------------------------------------
 *                       Pose-related functions and properties
 * ------------------------------------------------------------------------------------
 */


/*
 * Tells us whether two poses are adjacent and lie in the same row
 */
predicate AdjacentHorizontal(p1: Pose, p2: Pose)
{
  (p1.pos.row == p2.pos.row && p1.pos.col == p2.pos.col + 1)
  || (p1.pos.row == p2.pos.row && p1.pos.col == p2.pos.col - 1)
}

/*
 * Tells us whether two poses are adjacent and lie in the same column
 */
predicate AdjacentVertical(p1: Pose, p2: Pose)
{
  (p1.pos.col == p2.pos.col && p1.pos.row == p2.pos.row - 1)
  || (p1.pos.col == p2.pos.col && p1.pos.row == p2.pos.row + 1)
}

/*
 * Tells us whether two poses are adjacent considering rows and columns, but not diagonals
 */
predicate Adjacent(p1: Pose, p2: Pose, costMap: CostMap)
{
  AdjacentHorizontal(p1, p2) || AdjacentVertical(p1, p2)
}

/*
 * Returns a list with positions adjacent to p (including p itself)
 */
function method AdjacentOf(p: Point): seq<Point>
{
  [Point(p.row, p.col),
   Point(p.row - 1, p.col - 1),
   Point(p.row - 1, p.col),
   Point(p.row - 1, p.col + 1),
   Point(p.row, p.col - 1),
   Point(p.row, p.col + 1),
   Point(p.row + 1, p.col - 1),
   Point(p.row + 1, p.col),
   Point(p.row + 1, p.col + 1)
  ]
}

method EuclidDistance(p1: Point, p2: Point) returns (d: real) {
  var diffRows := AbsInt(p1.row - p2.row);
  var diffCols := AbsInt(p1.col - p2.col);
  d := Sqrt((diffRows * diffRows + diffCols * diffCols) as real) * mapCost;
}

function method AbsInt(x: int): int {
  if (x >= 0) then x else -x
}

function method AbsReal(x: real): real {
  if (x >= 0.0) then x else -x
}

function method {:extern} Sqrt(x: real): real
  requires x >= 0.0
  ensures Sqrt(x) >= 0.0
  ensures Sqrt(x) * Sqrt(x) == x

lemma SqrtZero(x: real)
  requires x > 0.0
  ensures Sqrt(x) > 0.0
{
  if (Sqrt(x) == 0.0) {
    assert x == 0.0;
  }
}


/* 
 * ------------------------------------------------------------------------------------
 *                     Cost map-related functions and predicates
 * ------------------------------------------------------------------------------------
 */

/*
 * It tells whether a given position in the cost map is free of obstacle
 */
predicate method Open(costMap: CostMap, row: int, col: int)
{
  costMap.value(Point(row, col)) < obstacleCost
}

/*
 * A cost map is valid iff every position has a positive cost
 */
predicate ValidCostMap(costMap: CostMap)
{
  forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols :: costMap.value(Point(i, j)) > 0.0
}


/* 
 * ------------------------------------------------------------------------------------
 *                  Potential map-related functions and predicates
 * ------------------------------------------------------------------------------------
 */

/*
 * It is satisfied iff the given potential map is a matrix with the given dimensions
 */
predicate PotentialMapHasDimensions(p: PotentialMap, numRows: nat, numCols: nat) {
  |p| == numRows
  && forall r: seq<RealInf> | r in p :: |r| == numCols
}

function method MinHorizontal(p: Pose, pot: PotentialMap, numRows: nat, numCols: nat): RealInf
  requires PotentialMapHasDimensions(pot, numRows, numCols)
  requires 0 <= p.pos.row < numRows && 0 <= p.pos.col < numCols
  ensures MinHorizontal(p, pot, numRows, numCols).Real? ==> 
    exists p': Pose :: AdjacentHorizontal(p', p) && 0 <= p'.pos.row < numRows && 0 <= p'.pos.col < numCols 
                    && pot[p'.pos.row][p'.pos.col] == MinHorizontal(p, pot, numRows, numCols)
{
  var left := if p.pos.col > 0 then pot[p.pos.row][p.pos.col - 1] else Infinity;
  var right := if p.pos.col + 1 < numCols then pot[p.pos.row][p.pos.col + 1] else Infinity;
  assert AdjacentHorizontal(Pose(Point(p.pos.row, p.pos.col - 1)), p);
  assert AdjacentHorizontal(Pose(Point(p.pos.row, p.pos.col + 1)), p);
  MinInfinity(left, right)
}

function method MinVertical(p: Pose, pot: PotentialMap, numRows: nat, numCols: nat): RealInf
  requires PotentialMapHasDimensions(pot, numRows, numCols)
  requires 0 <= p.pos.row < numRows && 0 <= p.pos.col < numCols
  ensures MinVertical(p, pot, numRows, numCols).Real? ==> 
    exists p': Pose :: AdjacentVertical(p', p) && 0 <= p'.pos.row < numRows && 0 <= p'.pos.col < numCols 
                    && pot[p'.pos.row][p'.pos.col] == MinVertical(p, pot, numRows, numCols)
{
  var up := if p.pos.row > 0 then pot[p.pos.row - 1][p.pos.col] else Infinity;
  var down := if p.pos.row + 1 < numRows then pot[p.pos.row + 1][p.pos.col] else Infinity;
  assert AdjacentVertical(Pose(Point(p.pos.row - 1, p.pos.col)), p);
  assert AdjacentVertical(Pose(Point(p.pos.row + 1, p.pos.col)), p);
  MinInfinity(up, down)
}


/*
 * A potential map satisfies position safety w.r.t. a cost map iff every tile with finite potential
 * is free of obstacles.
 */
predicate PositionSafe(pot: PotentialMap, costMap: CostMap)
{
  PotentialMapHasDimensions(pot, costMap.numRows, costMap.numCols)
  && forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols ::
    pot[i][j].Real? ==> Open(costMap, i, j)
}


/*
 * Given a position p in a potential map of (numRows x numCols) tiles, it tells
 * whether p has a (non-diagonal) adjacent position with finite potential
 */
predicate HasSomeAdjacentReal(p: Pose, pot: PotentialMap, numRows: nat, numCols: nat)
  requires PotentialMapHasDimensions(pot, numRows, numCols)
  requires 0 <= p.pos.row < numRows && 0 <= p.pos.col < numCols
{
  (p.pos.row > 0 && pot[p.pos.row - 1][p.pos.col].Real?)
      || (p.pos.row + 1 < numRows && pot[p.pos.row + 1][p.pos.col].Real?)
      || (p.pos.col > 0 && pot[p.pos.row][p.pos.col - 1].Real?)
      || (p.pos.col + 1 < numCols && pot[p.pos.row][p.pos.col + 1].Real?)
}

/*
 * A potential map satisfies progress iff every tile with finite potential (excluding the goal)
 * has another adjacent tile with lower potential.
 */
predicate Progress(pot: PotentialMap, goal: Pose, numRows: nat, numCols: nat)
{
  PotentialMapHasDimensions(pot, numRows, numCols)
  && forall i, j | 0 <= i < numRows && 0 <= j < numCols ::
        i != goal.pos.row && j != goal.pos.col && pot[i][j].Real? ==> 
          exists i', j' :: Adjacent(Pose(Point(i, j)), Pose(Point(i', j')), CostMap((p => 0.0), 0, 0))    // TODO: remove costmap
                && 0 <= i' < numRows && 0 <= j' < numCols
                && pot[i'][j'].Real?
                && GreaterThan(pot[i][j], pot[i'][j'])
}


/* 
 * ------------------------------------------------------------------------------------
 *                                    Pose queue
 * ------------------------------------------------------------------------------------
 */

/*
 * A queue is valid w.r.t. a potential map and a cost map iff all its positions
 * lie within the costmap and every position has an adjacent one with finite potential
 */
predicate ValidQueue(queue: seq<Pose>, pot: PotentialMap, costMap: CostMap)
  requires PotentialMapHasDimensions(pot, costMap.numRows, costMap.numCols)
{
  forall p: Pose | p in queue ::
    0 <= p.pos.row < costMap.numRows
    && 0 <= p.pos.col < costMap.numCols
    && HasSomeAdjacentReal(p, pot, costMap.numRows, costMap.numCols)
}

/* 
 * ------------------------------------------------------------------------------------
 *                                   NavFn algorithm
 * ------------------------------------------------------------------------------------
 */
method AStar(start: Pose, goal: Pose, costMap: CostMap, numIterations: nat, numPathIterations: nat) returns (error: bool, path: Path, navfn: PotentialMap)
  requires 0 <= start.pos.row < costMap.numRows && 0 <= start.pos.col < costMap.numCols
  requires 0 <= goal.pos.row < costMap.numRows && 0 <= goal.pos.col < costMap.numCols
  requires Open(costMap, goal.pos.row, goal.pos.col)
  requires ValidCostMap(costMap)
  ensures !error ==>
    |path| >= 1
    && path[0] == RealPoint(start.pos.row as real, start.pos.col as real)
    && path[|path| - 1] == RealPoint(goal.pos.row as real, goal.pos.col as real)

{
  navfn := ComputeNavFn(start, goal, costMap, numIterations);
  ghost var closestPath: seq<Point>;
  if (navfn[start.pos.row][start.pos.col].Real?) {
    error, path, closestPath := ComputePath(start.pos, goal.pos, navfn, numPathIterations, costMap.numRows, costMap.numCols);
  } else {
    error := true;
  }
}

/* 
 * ------------------------------------------------------------------------------------
 *                        Computation of NavFn function
 * ------------------------------------------------------------------------------------
 */


/*
 * It computes the navigation function given the start and goal positions and a costMap.
 * The numIterations parameters specifies an upper bound on how many times the contents of
 * the excess queue are transferred to the next queue.
 */
method ComputeNavFn(start: Pose, goal: Pose, costMap: CostMap, numIterations: nat) returns (navfn: PotentialMap)
  requires ValidCostMap(costMap)
  requires 0 <= start.pos.row < costMap.numRows && 0 <= start.pos.col < costMap.numCols
  requires 0 <= goal.pos.row < costMap.numRows && 0 <= goal.pos.col < costMap.numCols
  requires Open(costMap, goal.pos.row, goal.pos.col)
  ensures PositionSafe(navfn, costMap)
{
  var pot := BuildInitialPotentialMap(costMap.numRows, costMap.numCols, goal.pos.row, goal.pos.col);
  var initialThreshold := EuclidDistance(start.pos, goal.pos);
  initialThreshold := initialThreshold + obstacleCost;
  var current := InitCurrentQueue(goal, pot, costMap);
  navfn := ComputeNavFnRec(start, goal, costMap, pot, current, [], [], initialThreshold, numIterations);
}


/*
 * Compute the navigation function (i.e. potential map) given a start and goal positions, a cost map, an
 *
 */
method ComputeNavFnRec(start: Pose, goal: Pose,
                      costMap: CostMap, pot: PotentialMap,
                      current: seq<Pose>, next: seq<Pose>, excess: seq<Pose>,
                      threshold: real, numIterations: nat) returns (pot': PotentialMap)
  requires PositionSafe(pot, costMap)
  requires Progress(pot, goal, costMap.numRows, costMap.numCols)
  requires ValidQueue(current, pot, costMap) && ValidQueue(next, pot, costMap) && ValidQueue(excess, pot, costMap)
  requires ValidCostMap(costMap)
  requires 0 <= start.pos.row < costMap.numRows && 0 <= start.pos.col < costMap.numCols
  requires 0 <= goal.pos.row < costMap.numRows && 0 <= goal.pos.col < costMap.numCols
  ensures PositionSafe(pot', costMap)
  ensures Progress(pot', goal, costMap.numRows, costMap.numCols)
  decreases numIterations, |current|
{
  if (numIterations == 0) {
    return pot;
  }

  if (|current| > 0) {
    if (!Open(costMap, current[0].pos.row, current[0].pos.col)) {
      pot' := ComputeNavFnRec(start, goal, costMap, pot, current[1..], next, excess, threshold, numIterations);
      return;
    } else {
      var front := current[0];
      var minV := MinVertical(front, pot, costMap.numRows, costMap.numCols);
      var minH := MinHorizontal(front, pot, costMap.numRows, costMap.numCols);

      assert minV != Infinity || minH != Infinity by {
        assert HasSomeAdjacentReal(front, pot, costMap.numRows, costMap.numCols);
        if ((front.pos.row > 0 && pot[front.pos.row - 1][front.pos.col].Real?) 
            || (front.pos.row + 1 < costMap.numRows && pot[front.pos.row + 1][front.pos.col].Real?)) {
          assert minV.Real?;
        } else {
          assert  (front.pos.col > 0 && pot[front.pos.row][front.pos.col - 1].Real?) 
                    || (front.pos.col + 1 < costMap.numCols && pot[front.pos.row][front.pos.col + 1].Real?);
          assert minH.Real?;
        }
      } // Because current queue is valid
      var min := MinInfinity(minV, minH);
      var snd := MaxInfinity(minV, minH);
      assert min == minV || min == minH;
      ghost var pMin: Pose;
      if (minV == min) {
        ghost var minVPose :| AdjacentVertical(minVPose, front) 
            && 0 <= minVPose.pos.row < costMap.numRows
            && 0 <= minVPose.pos.col < costMap.numCols 
            && pot[minVPose.pos.row][minVPose.pos.col] == minV;
        pMin := minVPose;
        assert Adjacent(pMin, front, costMap) 
              && 0 <= pMin.pos.row < costMap.numRows
              && 0 <= pMin.pos.col < costMap.numCols
              && pot[pMin.pos.row][pMin.pos.col] == min;
      } else {
        ghost var minHPose :| AdjacentHorizontal(minHPose, front) 
            && 0 <= minHPose.pos.row < costMap.numRows
            && 0 <= minHPose.pos.col < costMap.numCols 
            && pot[minHPose.pos.row][minHPose.pos.col] == minH;
        pMin := minHPose;
        assert Adjacent(pMin, front, costMap) 
              && 0 <= pMin.pos.row < costMap.numRows
              && 0 <= pMin.pos.col < costMap.numCols
              && pot[pMin.pos.row][pMin.pos.col] == min;
      }
      assert Adjacent(pMin, front, costMap);
      assert 0 <= pMin.pos.row < costMap.numRows && 0 <= pMin.pos.col < costMap.numCols;
      assert pot[pMin.pos.row][pMin.pos.col] == min;
      var potAux, updated := UpdatePotential(pot, front, costMap, min, snd, pMin, goal);
      var next', excess' := next, excess;
      if (updated) {
        assert ValidQueue(current, potAux, costMap);
        assert ValidQueue(next, potAux, costMap);
        assert ValidQueue(excess, potAux, costMap);
        next', excess' := TraverseNeighbors(front, start, costMap, potAux, threshold, next, excess);
        assert ValidQueue(next', potAux, costMap);
        assert ValidQueue(excess', potAux, costMap);
      }
      pot' := ComputeNavFnRec(start, goal, costMap, potAux, current[1..], next', excess', threshold, numIterations);
    }
  } else {
    if (pot[start.pos.row][start.pos.col] != Infinity) {
      return pot;
    }
    if (next == []) {
      pot' := ComputeNavFnRec(start, goal, costMap, pot, excess, [], next, threshold + 2.0 * mapCost, numIterations - 1);
    } else {
      pot' := ComputeNavFnRec(start, goal, costMap, pot, next, [], excess, threshold, numIterations - 1);
    }
  }
}

/*
 * It returns a potential map of (numRows x numCols) size with all positions set to +\infty, except the position
 * (initRow, initCol), which has zero potential.
 */
method BuildInitialPotentialMap(numRows: nat, numCols: nat, initRow: nat,  initCol: nat) returns (p: PotentialMap)
  requires 0 <= initRow < numRows && 0 <= initCol < numCols
  ensures PotentialMapHasDimensions(p, numRows, numCols)
  ensures forall i, j | 0 <= i < numRows && 0 <= j < numCols && (i != initRow || j != initCol) :: p[i][j] == Infinity
  ensures p[initRow][initCol] == Real(0.0)
  ensures Progress(p, Pose(Point(initRow, initCol)), numRows, numCols); 
{
  var i := 0;
  var j := 0;

  p := seq(numRows, i => seq(numCols, j => Infinity));
  p := p[initRow := p[initRow][initCol := Real(0.0)]];
}


/*
 * It returns a queue with the open positions that are (non-diagonally) adjacent to p, provided that these
 * positions are within the bounds of the cost map.
 */
method InitCurrentQueue(p: Pose, ghost pot: PotentialMap, costMap: CostMap) returns (current: seq<Pose>)
  requires PotentialMapHasDimensions(pot, costMap.numRows, costMap.numCols)
  requires 0 <= p.pos.row < costMap.numRows && 0 <= p.pos.col < costMap.numCols
  requires pot[p.pos.row][p.pos.col].Real?
  ensures ValidQueue(current, pot, costMap)
{
  var p1l :=
    if (p.pos.col + 1 < costMap.numCols && Open(costMap, p.pos.row, p.pos.col + 1)) then [Pose(Point(p.pos.row, p.pos.col + 1))] else [];
  var p2l :=
    if (p.pos.col > 0 && Open(costMap, p.pos.row, p.pos.col - 1)) then [Pose(Point(p.pos.row, p.pos.col - 1))] else [];
  var p3l :=
    if (p.pos.row > 0 && Open(costMap, p.pos.row - 1, p.pos.col)) then [Pose(Point(p.pos.row - 1, p.pos.col))] else [];
  var p4l :=
    if (p.pos.row + 1 < costMap.numRows && Open(costMap, p.pos.row + 1, p.pos.col)) then [Pose(Point(p.pos.row + 1, p.pos.col))] else [];

  current := p1l + p2l + p3l + p4l;
}

/*
 * Assuming that the position p in the costMap is open, it updates the potential map at position p with
 * the information gathered from the adjacent positions.
 */
method UpdatePotential(pot: PotentialMap, p: Pose, costMap: CostMap, min: RealInf, snd: RealInf, ghost posMin: Pose, ghost goal: Pose) returns (pot': PotentialMap, updated: bool)
  requires PositionSafe(pot, costMap)
  requires Progress(pot, goal, costMap.numRows, costMap.numCols)
  requires ValidCostMap(costMap)
  requires 0 <= p.pos.row < costMap.numRows && 0 <= p.pos.col < costMap.numCols
  requires 0 <= posMin.pos.row < costMap.numRows && 0 <= posMin.pos.col < costMap.numCols
  requires pot[posMin.pos.row][posMin.pos.col] == min
  requires Adjacent(p, posMin, costMap)
  requires min.Real?
  requires Open(costMap, p.pos.row, p.pos.col)
  ensures PositionSafe(pot', costMap)
  ensures forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols ::
    pot[i][j].Real? ==> pot'[i][j].Real?
  ensures pot'[p.pos.row][p.pos.col].Real?
  ensures Progress(pot', goal, costMap.numRows, costMap.numCols)
{
  var hf := costMap.value(Point(p.pos.row, p.pos.col));
  var diff := Minus(snd, min);
  var v';
  if (GreaterThanOrEqual(diff, Real(hf))) {
    v' := hf + min.r;
  } else {
    assert diff.Real?;
    var d := diff.r / hf;
    var interpol := -0.2301 * d * d + 0.5307 * d + 0.7040;
    assume interpol > 0.0;
    // if (interpol < 0.00001) { interpol := 0.00001; }      // WARNING: Same behaviour?
    v' := min.r + hf * interpol;
  }
  assert v' > min.r;
  if (GreaterThan(pot[p.pos.row][p.pos.col], Real(v'))) {
    pot' := pot[p.pos.row := pot[p.pos.row][p.pos.col := Real(v')]];
    updated := true;
    assert Progress(pot', goal, costMap.numRows, costMap.numCols) by {
      forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols
      ensures i != goal.pos.row && j != goal.pos.col && pot'[i][j].Real? ==> 
            exists i', j' :: Adjacent(Pose(Point(i, j)), Pose(Point(i', j')), CostMap((p => 0.0), 0, 0))    // TODO: remove costmap
                  && 0 <= i' < costMap.numRows && 0 <= j' < costMap.numCols
                  && pot'[i'][j'].Real?
                  && GreaterThan(pot'[i][j], pot'[i'][j']);
      { 
        if (i != goal.pos.row && j != goal.pos.col && pot'[i][j].Real?) {
          if (i == p.pos.row && j == p.pos.col) {
            var i' := posMin.pos.row;
            var j' := posMin.pos.col;
            assert Adjacent(Pose(Point(i, j)), Pose(Point(i', j')), CostMap((p => 0.0), 0, 0));
            assert 0 <= i' < costMap.numRows && 0 <= j' < costMap.numCols;
            assert pot'[i'][j'] == pot[i'][j'] == min;
            assert pot'[i'][j'].Real?;
            assert pot'[i][j].Real?;
            assert pot'[i][j].r == v';
            assert GreaterThan(pot'[i][j], pot'[i'][j']);
          } else {
            assert pot'[i][j] == pot[i][j];
          }
        }
      }  
    }
  } else {
    pot' := pot;
    updated := false;
  }  
}

/*
 * It traverses all the positions adjacent to p and add them to the next and excess queues
 */
method TraverseNeighbors(p: Pose, start: Pose, costMap: CostMap, pot: PotentialMap, threshold: real, next: seq<Pose>, excess: seq<Pose>)
    returns (next': seq<Pose>, excess': seq<Pose>)
  requires PositionSafe(pot, costMap)
  requires ValidQueue(next, pot, costMap) && ValidQueue(excess, pot, costMap)
  requires 0 <= p.pos.row < costMap.numRows && 0 <= p.pos.col < costMap.numCols
  requires pot[p.pos.row][p.pos.col].Real?
  ensures ValidQueue(next', pot, costMap) && ValidQueue(excess', pot, costMap)
{
  var neighbor: Pose;

  var p1l, p1r: seq<Pose>;
  neighbor := Pose(Point(p.pos.row, p.pos.col - 1));
  if (p.pos.col > 0 && Open(costMap, p.pos.row, p.pos.col - 1) && neighbor !in next && neighbor !in excess) {
    p1l, p1r := TraverseNeighbor(p, neighbor, start, costMap, pot, threshold);
  } else {
    p1l := []; p1r := [];
  }

  var p2l, p2r: seq<Pose>;
  neighbor := Pose(Point(p.pos.row, p.pos.col + 1));
  if (p.pos.col < costMap.numCols - 1 && Open(costMap, p.pos.row, p.pos.col + 1) && neighbor !in next && neighbor !in excess) {
    p2l, p2r := TraverseNeighbor(p, neighbor, start, costMap, pot, threshold);
  } else {
    p2l := []; p2r := [];
  }

  var p3l, p3r: seq<Pose>;
  neighbor := Pose(Point(p.pos.row - 1, p.pos.col));
  if (p.pos.row > 0 && Open(costMap, p.pos.row - 1, p.pos.col) && neighbor !in next && neighbor !in excess) {
    p3l, p3r := TraverseNeighbor(p, neighbor, start, costMap, pot, threshold);
  } else {
    p3l := []; p3r := [];
  }

  var p4l, p4r: seq<Pose>;
  neighbor := Pose(Point(p.pos.row + 1, p.pos.col));
  if (p.pos.row < costMap.numRows - 1 && Open(costMap, p.pos.row + 1, p.pos.col) && neighbor !in next && neighbor !in excess) {
    p4l, p4r := TraverseNeighbor(p, neighbor, start, costMap, pot, threshold);
  } else {
    p4l := []; p4r := [];
  }

  next' := next + p1l + p2l + p3l + p4l;
  excess' := excess + p1r + p2r + p3r + p4r;
}

/*
 * It determines whether newP (which is non-diagonally adjacent to p) should be stored in the next queue or
 * in the excess queue.
 */
 method TraverseNeighbor(p: Pose, newP: Pose, start: Pose, costMap: CostMap, pot: PotentialMap, threshold: real) returns (next': seq<Pose>, excess': seq<Pose>)
  requires PositionSafe(pot, costMap)
  requires 0 <= p.pos.row < costMap.numRows && 0 <= p.pos.col < costMap.numCols
  requires 0 <= newP.pos.row < costMap.numRows && 0 <= newP.pos.col < costMap.numCols
  requires Adjacent(p, newP, costMap)
  requires pot[p.pos.row][p.pos.col].Real?
  ensures ValidQueue(next', pot, costMap) && ValidQueue(excess', pot, costMap)
{
  var h := EuclidDistance(p.pos, start.pos);
  if (!GreaterThan(pot[newP.pos.row][newP.pos.col], Real(pot[p.pos.row][p.pos.col].r + h + (1.0 / 1.41421) * costMap.value(Point(newP.pos.row, newP.pos.col))))) {
    next' := [];
    excess' := [];
  } else {
    if (pot[p.pos.row][p.pos.col].r + h < threshold) {
      next' := [newP];
      excess' := [];
    } else {
      next' := [];
      excess' := [newP];
    }
  }
}

/* 
 * ------------------------------------------------------------------------------------
 *                   Computation of path from potential map
 * ------------------------------------------------------------------------------------
 */


function method SignReal(x: real): real {
  if (x > 0.0) then 1.0
  else if (x < 0.0) then -1.0
  else 0.0
}

function method Round(x: real): int {
	if x - x.Floor as real > 0.5 || (x - x.Floor as real == 0.5 && x > 0.0) then x.Floor + 1 else x.Floor
}

function method ClosestPoint(p: OffsetPoint): Point {
  Point(p.base.row + Round(p.offset.row), p.base.col + Round(p.offset.col))
}

function method ToRealPoint(p: OffsetPoint): RealPoint {
  RealPoint(p.base.row as real + p.offset.row, p.base.col as real + p.offset.col)
}



lemma MultSameSign(x: real, y: real)
  requires (x < 0.0 && y < 0.0) || (x > 0.0 && y > 0.0)
  ensures x * y > 0.0
{ }

lemma NormPositive(x: real, y: real)
  requires x != 0.0 || y != 0.0
  ensures Sqrt(x * x + y * y) > 0.0
{
  var sumsq := x * x + y * y;

  if (x != 0.0) {
    calc {
      sumsq;
      ==
      x * x + y * y;
      >=
      x * x;
      > { MultSameSign(x, x); }
      0.0;
    }
  }
  else {
    assert y != 0.0;
    calc {
      sumsq;
      ==
      x * x + y * y;
      >=
      y * y;
      > { MultSameSign(y, y); }
      0.0;
    }
  }
  SqrtZero(sumsq);
}

lemma {:axiom} NormBound(x: real, y: real)
  requires x != 0.0 || y != 0.0
  ensures Sqrt(x * x + y * y) > 0.0
  ensures -1.0 < x * stepSize / Sqrt(x * x + y * y) < 1.0
// Assumed for the moment, but can be proved

lemma ClosestToInteger(p: OffsetPoint)
  requires p.offset == RealPoint(0.0, 0.0)
  ensures ClosestPoint(p) == p.base
{
  // Proved automatically
}

predicate method AllFree(p: Point, potentialMap: PotentialMap, ghost numRows: nat, ghost numCols: nat)
  requires |potentialMap| == numRows
  requires forall x | x in potentialMap :: |x| == numCols
{
  if (|potentialMap| == 0) then
    false
  else
    0 < p.row < |potentialMap| - 1 && 0 < p.col < |potentialMap[0]| - 1
    && forall adj | adj in AdjacentOf(p) :: potentialMap[adj.row][adj.col].Real?
}

method ComputePath(start: Point, goal: Point, potentialMap: PotentialMap, numSteps: nat, numRows: nat, numCols: nat)
  returns (error: bool, path: seq<RealPoint>, ghost closest: seq<Point>)
  requires PotentialMapHasDimensions(potentialMap, numRows, numCols)
  requires 0 <= start.row < numRows && 0 <= start.col < numCols
  requires potentialMap[start.row][start.col].Real?
  ensures !error ==>
            |path| == |closest|
            && |path| >= 1
            && path[0] == RealPoint(start.row as real, start.col as real)
            && path[|path| - 1] == RealPoint(goal.row as real, goal.col as real)
{
  var i := numSteps;
  var p: OffsetPoint := OffsetPoint(start, RealPoint(0.0, 0.0));
  ghost var cl := p.base;
  assert cl == start;
  path := [];
  closest := [];
  error := false;
  while (i > 0 && !error)
    decreases i
    invariant !error ==> 0 <= cl.row < numRows && 0 <= cl.col < numCols
    invariant !error ==> -1.0 <= p.offset.row <= 1.0 && -1.0 <= p.offset.col <= 1.0
    invariant !error ==> cl == p.base
    invariant !error ==> potentialMap[p.base.row][p.base.col].Real?
    invariant |path| == |closest|
    invariant path == [] ==> p.base == start && p.offset == RealPoint(0.0, 0.0)
    invariant (if path == [] then ToRealPoint(p) else path[0]) == RealPoint(start.row as real, start.col as real)
    invariant forall i | 0 <= i < |closest| :: 0 <= closest[i].row < numRows && 0 <= closest[i].col < numCols &&
        potentialMap[closest[i].row][closest[i].col].Real?
  {
    var clpoint := ClosestPoint(p);
    assert path == [] ==> (ClosestToInteger(p); clpoint == start);
    if (clpoint == goal) {
      path := path + [RealPoint(goal.row as real, goal.col as real)];
      closest := closest + [goal];
      error := false;
      return;
    }

    path := path + [ToRealPoint(p)];
    closest := closest + [cl];
    i := i - 1;

    var oscillation := |path| > 2 && path[|path| - 1] == path[|path| - 3];

    error, p, cl := NextMove(p, potentialMap, numRows, numCols, oscillation);
  }

  if (i == 0) {
    error := true;
  }
}

method GetGradient(pos: Point, potentialMap: PotentialMap, numRows: nat, numCols: nat)
  returns (grad : RealPoint)
  requires PotentialMapHasDimensions(potentialMap, numRows, numCols)
  requires 0 <= pos.row < numRows && 0 <= pos.col < numCols
{
  var gx: real := 0.0;
  var gy: real := 0.0;
  grad := RealPoint(0.0, 0.0);

  // Outside bounds
  if (!(1 <= pos.row < numRows - 1 && 1 <= pos.col < numCols - 1)) {
    return;
  }

  // Potential at neighbor positions
  var center := potentialMap[pos.row][pos.col];
  var north := potentialMap[pos.row - 1][pos.col];
  var south := potentialMap[pos.row + 1][pos.col];
  var west := potentialMap[pos.row][pos.col - 1];
  var east := potentialMap[pos.row][pos.col + 1];

  if (!center.Real?) {
    if (west.Real?) {
      gx := - obstacleCost;
    }
    else if (east.Real?) {
      gx := obstacleCost;
    }
    else {
      gx := 0.0;
    }

    if (north.Real?) {
      gy := - obstacleCost;
    }
    else if (south.Real?) {
      gy := obstacleCost;
    }
    else {
      gy := 0.0;
    }
  }
  else {
    if (west.Real?) {
      gx := gx + west.r - center.r;
    }
    if (east.Real?) {
      gx := gx + center.r - east.r;
    }
    if (north.Real?) {
      gy := gy + north.r - center.r;
    }
    if (south.Real?) {
      gy := gy + center.r - south.r;
    }
  }

  var norm := Sqrt(gx * gx + gy * gy);

  if (norm > 0.0) {
    gx := gx / norm;
    gy := gy / norm;
  }

  grad := RealPoint(gy, gx);
}

method NextMove(p: OffsetPoint, potentialMap: PotentialMap, numRows: nat, numCols: nat, oscillation: bool)
  returns (error: bool, p': OffsetPoint, ghost closest': Point)
  requires PotentialMapHasDimensions(potentialMap, numRows, numCols)
  requires 0 <= p.base.row < numRows && 0 <= p.base.col < numCols
  requires -1.0 <= p.offset.row <= 1.0 && -1.0 <= p.offset.col <= 1.0
//  requires closest == ClosestPoint(p)
  requires potentialMap[p.base.row][p.base.col].Real?
  ensures !error ==> closest' == p'.base
  ensures !error ==> 0 <= closest'.row < numRows && 0 <= closest'.col < numCols
  ensures !error ==> -1.0 <= p'.offset.row <= 1.0 && -1.0 <= p'.offset.col <= 1.0
  ensures !error ==> potentialMap[closest'.row][closest'.col].Real?
{
  if (AllFree(p.base, potentialMap, numRows, numCols) && !oscillation) {
    // Gradient of the neighbor positions
    var gcenter := GetGradient(p.base, potentialMap, numRows, numCols);
    var geast := GetGradient(Point(p.base.row, p.base.col + 1), potentialMap, numRows, numCols);
    var gsouth := GetGradient(Point(p.base.row + 1, p.base.col), potentialMap, numRows, numCols);
    var gse := GetGradient(Point(p.base.row + 1, p.base.col + 1), potentialMap, numRows, numCols);

    // Extrapolated gradient
    var x := (1.0 - p.offset.row) * ((1.0 - p.offset.col) * gcenter.col + p.offset.col * geast.col)
                 + p.offset.row * ((1.0 - p.offset.col) * gsouth.col + p.offset.col * gse.col);
    var y := (1.0 - p.offset.row) * ((1.0 - p.offset.col) * gcenter.row + p.offset.col * geast.row)
                 + p.offset.row * ((1.0 - p.offset.col) * gsouth.row + p.offset.col * gse.row);

    // There is no path
    if (x == 0.0 && y == 0.0) {
      error := true;
      return;
    }

    var nx := p.base.col;
    var ny := p.base.row;
    var dx := p.offset.col;
    var dy := p.offset.row;

    // (x, y) is not zero
    NormPositive(x, y);

    dx := dx + x * stepSize / Sqrt(x * x + y * y);
    dy := dy + y * stepSize / Sqrt(x * x + y * y);

    NormBound(x, y);
    NormBound(y, x);
    assert -2.0 < dx < 2.0;
    assert -2.0 < dy < 2.0;

    if (AbsReal(dx) > 1.0) {
      nx := nx + SignReal(dx) as int;
      dx := dx - SignReal(dx);
    }
    if (AbsReal(dy) > 1.0) {
      ny := ny + SignReal(dy) as int;
      dy := dy - SignReal(dy);
    }
    assert -1.0 <= dx <= 1.0;
    assert -1.0 <= dy <= 1.0;

    p' := OffsetPoint(Point(ny, nx), RealPoint(dy, dx));
    closest' := p'.base;
    error := false;

    assert potentialMap[p'.base.row][p'.base.col].Real?;
  } else {
    p' := p;
    closest' := p'.base;

    var adjs := AdjacentOf(p.base);
    var i := 0;
    var min_idx := 0;

    while (i < |adjs|)
      invariant 0 <= i <= |adjs|
      invariant 0 <= min_idx < |adjs|
    {
      if (0 <= adjs[i].row < numRows && 0 <= adjs[i].col < numCols &&
          0 <= adjs[min_idx].row < numRows && 0 <= adjs[min_idx].col < numCols &&
          !potentialMap[adjs[i].row][adjs[i].col].Infinity? && (
             potentialMap[adjs[min_idx].row][adjs[min_idx].col].Infinity? ||
             potentialMap[adjs[i].row][adjs[i].col].r.Floor < potentialMap[adjs[min_idx].row][adjs[min_idx].col].r.Floor)) {
        min_idx := i;
      }
      i := i + 1;
    }

    if (adjs[min_idx].row < 0 || adjs[min_idx].row >= numRows || adjs[min_idx].col < 0 || adjs[min_idx].col >= numCols ||
      potentialMap[adjs[min_idx].row][adjs[min_idx].col].Infinity?) {
      error :=  true;
    } else {
      error := false;
      p' := OffsetPoint(Point(adjs[min_idx].row, adjs[min_idx].col), RealPoint(0.0, 0.0));
      closest' := p'.base;
      assert potentialMap[p'.base.row][p'.base.col].Real?;
    }
  }
}

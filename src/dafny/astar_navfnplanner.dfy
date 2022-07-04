datatype Point = Point(row: int, col: int)
datatype RealPoint = RealPoint(row: real, col: real)
datatype OffsetPoint = OffsetPoint(base: Point, offset: RealPoint)
datatype Pose = Pose(pos: Point)
datatype CostMap = CostMap(value: Point -> real, numRows: nat, numCols: nat)
datatype RealInf = Real(r: real) | Infinity


type Path = seq<RealPoint>
type PotentialMap = seq<seq<RealInf>>

const obstacleCost: real := 254.0
const mapCost: real := 50.0
const stepSize: real := 0.5

predicate method Open(costMap: CostMap, row: int, col: int)
{
  costMap.value(Point(row, col))  < obstacleCost
}

predicate PotentialMapHasDimensions(p: PotentialMap, numRows: nat, numCols: nat) {
  |p| == numRows
  && forall r: seq<RealInf> | r in p :: |r| == numCols
}

predicate ValidPotentialMap(pot: PotentialMap, costMap: CostMap)
{
  PotentialMapHasDimensions(pot, costMap.numRows, costMap.numCols)
  && forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols ::
    pot[i][j].Real? ==> Open(costMap, i, j)
}

predicate ValidCostMap(costMap: CostMap)
{
  forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols :: costMap.value(Point(i, j)) > 0.0
}

predicate AdjacentHorizontal(p1: Pose, p2: Pose)
{
  (p1.pos.row == p2.pos.row && p1.pos.col == p2.pos.col + 1)
  || (p1.pos.row == p2.pos.row && p1.pos.col == p2.pos.col - 1)
}

predicate AdjacentVertical(p1: Pose, p2: Pose)
{
  (p1.pos.col == p2.pos.col && p1.pos.row == p2.pos.row - 1)
  || (p1.pos.col == p2.pos.col && p1.pos.row == p2.pos.row + 1)
}

predicate Adjacent(p1: Pose, p2: Pose, costMap: CostMap)
{
  AdjacentHorizontal(p1, p2) || AdjacentVertical(p1, p2)
}

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



predicate HasSomeAdjacentReal(p: Pose, pot: PotentialMap, numRows: nat, numCols: nat)
  requires PotentialMapHasDimensions(pot, numRows, numCols)
  requires 0 <= p.pos.row < numRows && 0 <= p.pos.col < numCols
{
  (p.pos.row > 0 && pot[p.pos.row - 1][p.pos.col].Real?)
      || (p.pos.row + 1 < numRows && pot[p.pos.row + 1][p.pos.col].Real?)
      || (p.pos.col > 0 && pot[p.pos.row][p.pos.col - 1].Real?)
      || (p.pos.col + 1 < numCols && pot[p.pos.row][p.pos.col + 1].Real?)
}


predicate ValidQueue(queue: seq<Pose>, pot: PotentialMap, costMap: CostMap)
  requires PotentialMapHasDimensions(pot, costMap.numRows, costMap.numCols)
{
  forall p: Pose | p in queue ::
    0 <= p.pos.row < costMap.numRows
    && 0 <= p.pos.col < costMap.numCols
    && HasSomeAdjacentReal(p, pot, costMap.numRows, costMap.numCols)
}

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
  var pot := BuildInitialPotentialMap(costMap.numRows, costMap.numCols, goal.pos.row, goal.pos.col);
  var initialThreshold := EuclidDistance(start.pos, goal.pos);
  initialThreshold := initialThreshold + obstacleCost;
  var current := InitCurrentQueue(goal, pot, costMap);

  var pot': PotentialMap := AStarIteration(start, goal, costMap, pot, current, [], [], initialThreshold, numIterations);
  ghost var closestPath: seq<Point>;
  if (pot'[start.pos.row][start.pos.col].Real?) {
    error, path, closestPath := ComputePath(start.pos, goal.pos, pot', numPathIterations, costMap.numRows, costMap.numCols);
    navfn := pot';
  } else {
    error := true;
  }
}

method AStarIteration(start: Pose, goal: Pose,
                      costMap: CostMap, pot: PotentialMap,
                      current: seq<Pose>, next: seq<Pose>, excess: seq<Pose>,
                      threshold: real, numIterations: nat) returns (pot': PotentialMap)
  requires ValidPotentialMap(pot, costMap)
  requires ValidQueue(current, pot, costMap) && ValidQueue(next, pot, costMap) && ValidQueue(excess, pot, costMap)
  requires ValidCostMap(costMap)
  requires 0 <= start.pos.row < costMap.numRows && 0 <= start.pos.col < costMap.numCols
  requires 0 <= goal.pos.row < costMap.numRows && 0 <= goal.pos.col < costMap.numCols
  ensures ValidPotentialMap(pot', costMap)
  decreases numIterations, |current|
{
  if (numIterations == 0) {
    return pot;
  }

  if (|current| > 0) {
    if (!Open(costMap, current[0].pos.row, current[0].pos.col)) {
      pot' := AStarIteration(start, goal, costMap, pot, current[1..], next, excess, threshold, numIterations);
      return;
    } else {
      var minV := MinVertical(current[0], pot, costMap.numRows);
      var minH := MinHorizontal(current[0], pot, costMap.numCols);
      assert minV != Infinity || minH != Infinity;
      var min := MinInfinity(minV, minH);
      var snd := MaxInfinity(minV, minH);
      var potAux, updated := UpdatePotential(pot, current[0], goal, costMap, min, snd);
      var next', excess' := next, excess;
      if (updated) {
        assert ValidQueue(current, potAux, costMap);
        assert ValidQueue(next, potAux, costMap);
        assert ValidQueue(excess, potAux, costMap);
        next', excess' := TraverseNeighbors(current[0], start, costMap, potAux, threshold, next, excess);
        assert ValidQueue(next', potAux, costMap);
        assert ValidQueue(excess', potAux, costMap);
      }
      pot' := AStarIteration(start, goal, costMap, potAux, current[1..], next', excess', threshold, numIterations);
    }
  } else {
    if (pot[start.pos.row][start.pos.col] != Infinity) {
      return pot;
    }
    if (next == []) {
      pot' := AStarIteration(start, goal, costMap, pot, excess, [], next, threshold + 2.0 * mapCost, numIterations - 1);
    } else {
      pot' := AStarIteration(start, goal, costMap, pot, next, [], excess, threshold, numIterations - 1);
    }
  }
}

method BuildInitialPotentialMap(numRows: nat, numCols: nat, initRow: nat,  initCol: nat) returns (p: PotentialMap)
  requires 0 <= initRow < numRows && 0 <= initCol < numCols
  ensures PotentialMapHasDimensions(p, numRows, numCols)
  ensures forall i, j | 0 <= i < numRows && 0 <= j < numCols && (i != initRow || j != initCol) :: p[i][j] == Infinity
  ensures p[initRow][initCol] == Real(0.0)
{
  var i := 0;
  var j := 0;

  p := seq(numRows, i => seq(numCols, j => Infinity));
  p := p[initRow := p[initRow][initCol := Real(0.0)]];
}


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

method UpdatePotential(pot: PotentialMap, p: Pose, goal: Pose, costMap: CostMap, min: RealInf, snd: RealInf) returns (pot': PotentialMap, updated: bool)
  requires ValidPotentialMap(pot, costMap)
  requires ValidCostMap(costMap)
  requires 0 <= p.pos.row < costMap.numRows && 0 <= p.pos.col < costMap.numCols
  requires min.Real?
  requires Open(costMap, p.pos.row, p.pos.col)
  ensures ValidPotentialMap(pot', costMap)
  ensures forall i, j | 0 <= i < costMap.numRows && 0 <= j < costMap.numCols ::
    pot[i][j].Real? ==> pot'[i][j].Real?
  ensures pot'[p.pos.row][p.pos.col].Real?
{
  var hf := costMap.value(Point(p.pos.row, p.pos.col));
  var diff := Minus(snd, min);
  var v';
  if (GreaterThanOrEqual(diff, Real(hf))) {
    v' := hf + min.r;
    pot' := pot[p.pos.row := pot[p.pos.row][p.pos.col := Real(v')]];
  } else {
    assert diff.Real?;
    var d := diff.r / hf;
    var interpol := -0.2301 * d * d + 0.5307 * d + 0.7040;
    v' := min.r + hf * interpol;
    pot' := pot[p.pos.row := pot[p.pos.row][p.pos.col := Real(v')]];
  }
  updated := false;
  if (GreaterThan(pot[p.pos.row][p.pos.col], Real(v'))) {
    pot' := pot[p.pos.row := pot[p.pos.row][p.pos.col := Real(v')]];
    updated := true;
  }
}

method TraverseNeighbors(p: Pose, start: Pose, costMap: CostMap, pot: PotentialMap, threshold: real, next: seq<Pose>, excess: seq<Pose>)
    returns (next': seq<Pose>, excess': seq<Pose>)
  requires ValidPotentialMap(pot, costMap)
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

method TraverseNeighbor(p: Pose, newP: Pose, start: Pose, costMap: CostMap, pot: PotentialMap, threshold: real) returns (next': seq<Pose>, excess': seq<Pose>)
  requires ValidPotentialMap(pot, costMap)
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


function method AbsInt(x: int): int {
  if (x >= 0) then x else -x
}

function method AbsReal(x: real): real {
  if (x >= 0.0) then x else -x
}

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

method EuclidDistance(p1: Point, p2: Point) returns (d: real) {
  var diffRows := AbsInt(p1.row - p2.row);
  var diffCols := AbsInt(p1.col - p2.col);
  d := Sqrt((diffRows * diffRows + diffCols * diffCols) as real) * mapCost;
}

function method MinHorizontal(p: Pose, pot: PotentialMap, numCols: nat): RealInf
  requires exists numRows :: PotentialMapHasDimensions(pot, numRows, numCols) && 0 <= p.pos.row < numRows
  requires 0 <= p.pos.col < numCols
{
  var left := if p.pos.col > 0 then pot[p.pos.row][p.pos.col - 1] else Infinity;
  var right := if p.pos.col + 1 < numCols then pot[p.pos.row][p.pos.col + 1] else Infinity;
  MinInfinity(left, right)
}

function method MinVertical(p: Pose, pot: PotentialMap, numRows: nat): RealInf
  requires exists numCols :: PotentialMapHasDimensions(pot, numRows, numCols) && 0 <= p.pos.col < numCols
  requires 0 <= p.pos.row < numRows
{
  var up := if p.pos.row > 0 then pot[p.pos.row - 1][p.pos.col] else Infinity;
  var down := if p.pos.row + 1 < numRows then pot[p.pos.row + 1][p.pos.col] else Infinity;
  MinInfinity(up, down)
}

function method MinInfinity(x1: RealInf, x2: RealInf): RealInf {
  if (x1.Infinity?) then x2
  else if (x2.Infinity?) then x1
  else if (x1.r <= x2.r) then x1 else x2
}

function method MaxInfinity(x1: RealInf, x2: RealInf): RealInf {
  if (x1.Infinity?) then x1
  else if (x2.Infinity?) then x2
  else if (x1.r >= x2.r) then x1 else x2
}

predicate method GreaterThan(x1: RealInf, x2: RealInf)
  requires x1.Real? || x2.Real?
{
  x1.Infinity? || (x2.Real? && x1.r > x2.r)
}

predicate method GreaterThanOrEqual(x1: RealInf, x2: RealInf)
{
  x1.Infinity? || (x1.Real? && x2.Real? && x1.r >= x2.r)
}


function method Minus(x1: RealInf, x2: RealInf): RealInf
  requires x2.Real?
{
  if (x1.Infinity?) then Infinity else Real(x1.r - x2.r)
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
      >
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
      >
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

datatype Point = Point(row: int, col: int)
datatype RealPoint = RealPoint(row: real, col: real)
datatype Pose = Pose(pos: Point)
datatype CostMap = CostMap(value: Point -> real, numRows: nat, numCols: nat)
datatype RealInf = Real(r: real) | Infinity


type Path = seq<RealPoint>
type PotentialMap = seq<seq<RealInf>>

const obstacleCost: real := 254.0
const mapCost: real := 50.0


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
  [Point(p.row - 1, p.col - 1),
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

method AStar(start: Pose, goal: Pose, costMap: CostMap, numIterations: nat) returns (error: bool, path: Path)
  requires 0 <= start.pos.row < costMap.numRows && 0 <= start.pos.col < costMap.numCols
  requires 0 <= goal.pos.row < costMap.numRows && 0 <= goal.pos.col < costMap.numCols
  requires Open(costMap, goal.pos.row, goal.pos.col)
  requires ValidCostMap(costMap)
  ensures !error ==>
    |path| >= 1
    && path[0] == RealPoint(start.pos.row as real, start.pos.col as real)
    && path[|path| - 1] == RealPoint(goal.pos.row as real, goal.pos.col as real)
    && forall p | p in path :: Open(costMap, ClosestPoint(p).row, ClosestPoint(p).col)

{
  var pot := BuildInitialPotentialMap(costMap.numRows, costMap.numCols, goal.pos.row, goal.pos.col);
  var initialThreshold := EuclidDistance(start.pos, goal.pos);
  initialThreshold := initialThreshold + obstacleCost;
  var current := InitCurrentQueue(goal, pot, costMap);

  var pot': PotentialMap := AStarIteration(start, goal, costMap, pot, current, [], [], initialThreshold, numIterations);
  ghost var closestPath: seq<Point>;
  if (pot'[start.pos.row][start.pos.col].Real?) {
    error, path, closestPath := ComputePath(start.pos, goal.pos, pot', numIterations, costMap.numRows, costMap.numCols);
    assert !error ==> forall p | p in path :: pot'[ClosestPoint(p).row][ClosestPoint(p).col].Real?;
    assert !error ==> forall p | p in path :: Open(costMap, ClosestPoint(p).row, ClosestPoint(p).col);
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

  if (pot[start.pos.row][start.pos.col] != Infinity) {
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
      var potAux := UpdatePotential(pot, current[0], goal, costMap, min, snd);
      assert ValidQueue(current, potAux, costMap);
      assert ValidQueue(next, potAux, costMap);
      assert ValidQueue(excess, potAux, costMap);
      var next', excess' := TraverseNeighbors(current[0], start, costMap, potAux, threshold, next, excess);
      assert ValidQueue(next', potAux, costMap);
      assert ValidQueue(excess', potAux, costMap);
      pot' := AStarIteration(start, goal, costMap, potAux, current[1..], next', excess', threshold, numIterations);
    }
  } else {
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
    if (p.pos.row > 0 && Open(costMap, p.pos.row - 1, p.pos.col)) then [Pose(Point(p.pos.row - 1, p.pos.col))] else [];
  var p2l :=
    if (p.pos.row + 1 < costMap.numRows && Open(costMap, p.pos.row + 1, p.pos.col)) then [Pose(Point(p.pos.row + 1, p.pos.col))] else [];
  var p3l :=
    if (p.pos.col > 0 && Open(costMap, p.pos.row, p.pos.col - 1)) then [Pose(Point(p.pos.row, p.pos.col - 1))] else [];
  var p4l :=
    if (p.pos.col + 1 < costMap.numCols && Open(costMap, p.pos.row, p.pos.col + 1)) then [Pose(Point(p.pos.row, p.pos.col + 1))] else [];

  current := p1l + p2l + p3l + p4l;
}

method UpdatePotential(pot: PotentialMap, p: Pose, goal: Pose, costMap: CostMap, min: RealInf, snd: RealInf) returns (pot': PotentialMap)
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
  if (GreaterThan(diff, Real(hf))) {
    var v' := hf + min.r;
    pot' := pot[p.pos.row := pot[p.pos.row][p.pos.col := Real(v')]];
  } else {
    assert diff.Real?;
    var d := diff.r / hf;
    var interpol := -0.2301 * d * d + 0.5307 * d + 0.7040;
    var v' := min.r + hf * interpol;
    pot' := pot[p.pos.row := pot[p.pos.row][p.pos.col := Real(v')]];
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
  var p1l, p1r: seq<Pose>;
  if (p.pos.row > 0 && Open(costMap, p.pos.row - 1, p.pos.col)) {
    p1l, p1r := TraverseNeighbor(p, Pose(Point(p.pos.row - 1, p.pos.col)), start, costMap, pot, threshold);
  } else {
    p1l := []; p1r := [];
  }

  var p2l, p2r: seq<Pose>;
  if (p.pos.row < costMap.numRows - 1 && Open(costMap, p.pos.row + 1, p.pos.col)) {
    p2l, p2r := TraverseNeighbor(p, Pose(Point(p.pos.row + 1, p.pos.col)), start, costMap, pot, threshold);
  } else {
    p2l := []; p2r := [];
  }

  var p3l, p3r: seq<Pose>;
  if (p.pos.col > 0 && Open(costMap, p.pos.row, p.pos.col - 1)) {
    p3l, p3r := TraverseNeighbor(p, Pose(Point(p.pos.row, p.pos.col - 1)), start, costMap, pot, threshold);
  } else {
    p3l := []; p3r := [];
  }

  var p4l, p4r: seq<Pose>;
  if (p.pos.col < costMap.numCols - 1 && Open(costMap, p.pos.row, p.pos.col + 1)) {
    p4l, p4r := TraverseNeighbor(p, Pose(Point(p.pos.row, p.pos.col + 1)), start, costMap, pot, threshold);
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

function method ClosestPoint(p: RealPoint): Point {
  Point((p.row + 0.5).Floor, (p.col + 0.5).Floor)
}



method EuclidDistance(p1: Point, p2: Point) returns (d: real) {
  var diffRows := AbsInt(p1.row - p2.row);
  var diffCols := AbsInt(p1.col - p2.col);
  d := (diffRows * diffRows + diffCols * diffCols) as real; // TODO: sqrt?
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
            && forall i | 0 <= i < |path| :: ClosestPoint(path[i]) == closest[i]
            && forall i | 0 <= i < |closest| :: 0 <= closest[i].row < numRows && 0 <= closest[i].col < numCols && potentialMap[closest[i].row][closest[i].col].Real?
{
  var i := numSteps;
  var p: RealPoint := RealPoint(start.row as real, start.col as real);
  ghost var cl := ClosestPoint(p);
  assert cl == start;
  path := [p];
  closest := [cl];
  error := false;
  while (i > 0 && !error && ClosestPoint(p) != goal)
    decreases !error, i
    invariant !error ==> 0 <= cl.row < numRows && 0 <= cl.col < numCols
    invariant !error ==> cl == ClosestPoint(p)
    invariant !error ==> potentialMap[cl.row][cl.col].Real?
    invariant |path| == |closest|
    invariant |path| > 0
    invariant path[0] == RealPoint(start.row as real, start.col as real)
    invariant forall i | 0 <= i < |path| :: ClosestPoint(path[i]) == closest[i]
    invariant forall i | 0 <= i < |closest| :: 0 <= closest[i].row < numRows && 0 <= closest[i].col < numCols &&
        potentialMap[closest[i].row][closest[i].col].Real?
  {
    error, p, cl := NextMove(p, cl, potentialMap, numRows, numCols);
    if (!error) {
      path := path + [p];
      closest := closest + [cl];
      i := i - 1;
    }
  }

  if (error || i == 0) {
    error := true;
  } else {
    error := false;
    path := path + [RealPoint(goal.row as real, goal.col as real)];
    closest := closest + [goal];
  }
}

method NextMove(p: RealPoint, ghost closest: Point, potentialMap: PotentialMap, numRows: nat, numCols: nat)
  returns (error: bool, p': RealPoint, ghost closest': Point)
  requires PotentialMapHasDimensions(potentialMap, numRows, numCols)
  requires 0 <= closest.row < numRows && 0 <= closest.col < numCols
  requires closest == ClosestPoint(p)
  requires potentialMap[closest.row][closest.col].Real?
  ensures !error ==> closest' == ClosestPoint(p')
  ensures !error ==> 0 <= closest'.row < numRows && 0 <= closest'.col < numCols
  ensures !error ==> potentialMap[closest'.row][closest'.col].Real?
{
  var cp := ClosestPoint(p);
  assert cp == closest;
  if (AllFree(cp, potentialMap, numRows, numCols)) {
    var drow: real := *;
    var dcol: real := *;

    if (AbsReal(drow) > 1.0) {
      drow := SignReal(drow);
    }
    if (AbsReal(dcol) > 1.0) {
      dcol := SignReal(dcol);
    }
    assert -1.0 <= drow <= 1.0;
    assert -1.0 <= dcol <= 1.0;

    p' := RealPoint(p.row + drow, p.col + dcol);
    closest' := ClosestPoint(p');
    error := false;

    assert 0 <= closest'.row < numRows;
    assert 0 <= closest'.col < numCols;

    assert potentialMap[closest'.row][closest'.col].Real?;
  } else {
    p' := p;
    closest' := ClosestPoint(p');

    var adjs := AdjacentOf(cp);
    var i := 0;
    var min_idx := 0;

    while (i < |adjs|)
      invariant 0 <= i <= |adjs|
      invariant 0 <= min_idx < |adjs|
      invariant forall j | 0 <= j < i ::
        (adjs[j].row < 0 || adjs[j].row >= numRows || adjs[j].col < 0 || adjs[j].col >= numCols ||
         adjs[min_idx].row < 0 || adjs[min_idx].row >= numRows || adjs[min_idx].col < 0 || adjs[min_idx].col >= numCols ||
        GreaterThanOrEqual(potentialMap[adjs[j].row][adjs[j].col], potentialMap[adjs[min_idx].row][adjs[min_idx].col]))
    {
      if (0 <= adjs[i].row < numRows && 0 <= adjs[i].col < numCols &&
          0 <= adjs[min_idx].row < numRows && 0 <= adjs[min_idx].col < numCols &&
          !GreaterThanOrEqual(potentialMap[adjs[i].row][adjs[i].col], potentialMap[adjs[min_idx].row][adjs[min_idx].col])) {
        min_idx := i;
      }
      i := i + 1;
    }

    if (adjs[min_idx].row < 0 || adjs[min_idx].row >= numRows || adjs[min_idx].col < 0 || adjs[min_idx].col >= numCols ||
      potentialMap[adjs[min_idx].row][adjs[min_idx].col].Infinity?) {
      error :=  true;
    } else {
      error := false;
      p' := RealPoint(adjs[min_idx].row as real, adjs[min_idx].col as real);
      closest' := ClosestPoint(p');
      assert potentialMap[closest'.row][closest'.col].Real?;
    }
  }
}


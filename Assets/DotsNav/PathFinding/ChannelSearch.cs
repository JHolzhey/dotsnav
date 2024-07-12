using System;
using DotsNav.Collections;
using DotsNav.Drawing;
using DotsNav.Navmesh;
using DotsNav.PathFinding.Data;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace DotsNav.PathFinding
{
    readonly unsafe struct ChannelSearch
    {
        readonly PriorityQueue<Step> _open;
        readonly NativeHashSet<int> _closed;
        readonly NativeHashMap<int, Step> _closedStepsDebug;
        readonly NativeHashSet<int> _closedEdges;
        readonly NativeHashSet<IntPtr> _closedVerts;
        readonly List<Step> _steps;
        readonly List<PossibleDisturbance> _verts;
        readonly List<int> _validGoalEdges;

        public ChannelSearch(int maxNodes, int intialSteps, Allocator allocator) : this()
        {
            _open = new PriorityQueue<Step>(maxNodes);
            _closed = new NativeHashSet<int>(maxNodes, allocator);
            _closedStepsDebug = new NativeHashMap<int, Step>(10, allocator);
            _closedEdges = new NativeHashSet<int>(36, allocator);
            _closedVerts = new NativeHashSet<IntPtr>(36, allocator);
            _steps = new List<Step>(intialSteps, allocator);
            _verts = new List<PossibleDisturbance>(allocator);
            _validGoalEdges = new List<int>(allocator);
        }

        public interface INextClearance { public float Get(Edge* next); }
        public struct ClearanceLeft : INextClearance { public readonly float Get(Edge* next) => next->ClearanceLeft; }
        public struct ClearanceRight : INextClearance { public readonly float Get(Edge* next) => next->Sym->ClearanceRight; }

        public PathQueryState Search(float2 start, float2 goal, Navmesh.Navmesh* navmesh, List<Gate> path, float radius, DynamicBuffer<TriangleElement> triangleIds, out int cost)
        {
            var open = _open;
            var closed = _closed;
            var closedStepsDebug = _closedStepsDebug;
            var steps = _steps;
            var diameter = 2 * radius;
            var verts = _verts;
            var validGoalEdges = _validGoalEdges;
            var materialTypes = navmesh->MaterialTypes;
            cost = 80;

            if (!navmesh->Contains(start))
                return PathQueryState.StartInvalid;

            if (!navmesh->Contains(goal))
                return PathQueryState.GoalInvalid;

            // navmesh->FindTrianglesContainingPoint(start, out Edge* startEdge, out Edge* startEdgeMajor);
            Edge* startEdge = navmesh->FindTriangleContainingPoint(start, false);
            if (!EndpointValid(start, radius, startEdge))
                return PathQueryState.StartInvalid;
            var startId = startEdge->TriangleId;
            PlanePoint start3D = new (start, startEdge->FaceTriangle().Plane.SampleElevation(start));

            // navmesh->FindTrianglesContainingPoint(goal, out Edge* goalEdge, out Edge* goalEdgeMajor);
            Edge* goalEdge = navmesh->FindTriangleContainingPoint(goal, false);
            if (!EndpointValid(goal, radius, goalEdge))
                return PathQueryState.GoalInvalid;
            var goalId = goalEdge->TriangleId;
            PlanePoint goal3D = new (goal, goalEdge->FaceTriangle().Plane.SampleElevation(goal));

            var foundDisturbance = false;
            if (startId == goalId)
            {
                var sg = goal - start;
                var r = TransformRect((start + goal) / 2, new float2(diameter, math.length(sg)), Math.Angle(sg));
                verts.Clear();
                VerticesInQuad(r, startEdge, verts);
                for (int i = 0; i < verts.Length; i++)
                {
                    var v = verts[i];
                    var p = v.Vertex->Point;
                    var psg = Math.ProjectLine(start, goal, p) - start;
                    v.DistFromStart = math.lengthsq(psg);

                    if (CheckForDisturbance(p, start, goal, out var opp, startEdge, diameter))
                    {
                        foundDisturbance = true;
                        break;
                    }

                    v.Opposite = opp;
                    verts[i] = v;
                }

                if (!foundDisturbance)
                {
                    verts.Sort<PossibleDisturbance>();

                    for (int i = 0; i < verts.Length; i++)
                    {
                        var v = verts[i];
                        var p = v.Vertex->Point;
                        var e = GeometricPredicates.Orient2DFast(start, goal, p) > 0 ? new Gate {Left = p, Right = v.Opposite} : new Gate {Left = v.Opposite, Right = p};

                        if (path.Length > 0)
                        {
                            var gate = new Gate {Left = path[^1].Left, Right = e.Right};
                            path.Add(gate);
                            DebugDraw(gate.Left, gate.Right, Color.magenta);
                        }

                        path.Add(e);
                        DebugDraw(e.Left, e.Right, Color.magenta);
                    }

                    triangleIds.Add(startId);

                    return PathQueryState.PathFound;
                }
            }

            var ob = this;
            GetValidGoalEdges(_validGoalEdges);

            if (_validGoalEdges.Length == 0)
                return PathQueryState.NoPath;

            ExpandInitial(startEdge->Sym);
            ExpandInitial(startEdge->LNext->Sym);
            ExpandInitial(startEdge->LPrev->Sym);
            
            closed.Add(startId);
            closed.Add(goalId);

            while (open.Count > 0)
            {
                var step = open.Extract();
                var id = step.Id;

                if (id == goalId) // If found path:
                {                    
                    var e = step.Edge->TriangleId == goalId ? step.Edge : step.Edge->Sym;

                    float distanceCost = 0;
                    float totalCost = 0;
                    float maxSlopeCost = 0;

                    DebugDrawArrow(step.ReferencePoint, goal3D, Color.blue);
                    while (step.Previous != -1)
                    {
                        DebugDrawArrow(steps[step.Previous].ReferencePoint, step.ReferencePoint, Color.blue);
                        AddStepToGatePath(step);
                        step = steps[step.Previous];
                    }
                    DebugDrawArrow(start3D, step.ReferencePoint, Color.blue);
                    AddStepToGatePath(step);

                    void AddStepToGatePath(Step step)
                    {
                        Debug.Assert(step.MainEdgeType != Edge.Type.Obstacle, "Steps should not be obstacles");
                        // if (step.MainEdgeType == Edge.Type.Clearance) {
                        //     path.Add(new Gate { Left = step.Edge->GetMajorEdge()->Org->Point, Right = step.Edge->GetMajorEdge()->Dest->Point });
                        //     triangleIds.Add(step.Edge->TriangleId);
                        // } else { //if (step.MainEdgeType == Edge.Type.Terrain) {
                        //     path.Add(new Gate { Left = step.Edge->Org->Point, Right = step.Edge->Dest->Point });
                        //     triangleIds.Add(step.Edge->TriangleId);
                        // }
                        
                        path.Add(new Gate { Left = step.Edge->Org->Point, Right = step.Edge->Dest->Point });
                        triangleIds.Add(step.Edge->TriangleId);

                        // Debug:
                        maxSlopeCost = math.max(maxSlopeCost, step.Edge->CalcSlopeCost());
                        totalCost += step.G;
                        distanceCost += step.debugDistance;
                    }

                    triangleIds.Add(startId);

                    Debug.Log($"totalCost: {totalCost}, distanceCost: {distanceCost}, maxSlopeCost: {maxSlopeCost}");

                    // AddEndpointEdges(goal, e, true);

                    e = step.Edge->TriangleId == startId ? step.Edge : step.Edge->Sym;
                    // AddEndpointEdges(start, e, false);

                    path.Reverse();
                    for (int i = 0; i < path.Length; i++) {
                        // DebugDrawArrow(path[i].Left.XOY(), path[i].Right.XOY(), Color.white, 0.01f);
                    } 
                    return PathQueryState.PathFound;
                }

                closed.Add(id);
                closedStepsDebug.TryAdd(id, step);

                Edge* next = step.Edge->LNext->Sym;
                float clearanceRight = GetEdgeClearance<ClearanceRight>(next, step.PrevMajorEdge, out Edge* currMajorEdgeR);
                Expand(next, clearanceRight, currMajorEdgeR);
                // DebugDrawArrow(next->Sym->Org->Point3D, next->Sym->Dest->Point3D, Color.white, 0.01f);


                next = step.Edge->LPrev->Sym;
                float clearanceLeft = GetEdgeClearance<ClearanceLeft>(next, step.PrevMajorEdge, out Edge* currMajorEdgeL);
                Expand(next, clearanceLeft, currMajorEdgeL);
                // DebugDrawArrow(next->Org->Point3D, next->Dest->Point3D, Color.blue, 0.01f);

                ++cost;

                float GetEdgeClearance<TNextClearance>(Edge* next, Edge* prevMajorEdge, out Edge* currMajorEdge) where TNextClearance : unmanaged, INextClearance
                {
                    currMajorEdge = prevMajorEdge; // If this Minor edge doesn't have a MajorEdge, then prevMajorEdge carries over
                    switch(next->MainEdgeType) 
                    {
                    case Edge.Type.Clearance:
                        currMajorEdge = next->GetMajorEdge();

                        float clearance = float.MaxValue;
                        if (currMajorEdge->DNext == prevMajorEdge) {
                            clearance = new ClearanceRight().Get(currMajorEdge);
                        } else if (currMajorEdge->OPrev == prevMajorEdge) {
                            clearance = new ClearanceLeft().Get(currMajorEdge);
                        } else if (prevMajorEdge != null && currMajorEdge->Sym != prevMajorEdge) { // Then we are dealing with an overwritten MajorEdge edge
                            clearance = -1;
                            Debug.LogWarning("Overwritten MajorEdge");
// #if UNITY_ASSERTIONS
//                             Debug.Assert(step.Edge->IsMajorEdgeOverwritten || next->IsMajorEdgeOverwritten, $"Wrong, MajorEdge was not Overwritten: {currMajorEdge == prevMajorEdge}, {currMajorEdge->ONext == prevMajorEdge}, {currMajorEdge->LNext == prevMajorEdge}, {currMajorEdge->DPrev == prevMajorEdge}, {currMajorEdge->OPrev == prevMajorEdge}, {currMajorEdge->DNext == prevMajorEdge}, {currMajorEdge->RPrev == prevMajorEdge}, {currMajorEdge->RNext == prevMajorEdge}");
// #endif
                        }
                        // if (prevMajorEdge != null) { DebugDrawArrow(prevMajorEdge->Org->Point3D, prevMajorEdge->Dest->Point3D, Color.white, 0.01f); }
                        return clearance;
                    case Edge.Type.Terrain:
                        // if is more than clearance, special centering code
                        return new TNextClearance().Get(next);
                    case Edge.Type.Obstacle:
                        return -1;
                    case Edge.Type.Ignore:
                        return float.MaxValue;
                    default:
                        Debug.Assert(false, $"Unknown MainEdgeType: {next->MainEdgeType}");
                        return -1;
                    }
                }

                void Expand(Edge* next, float clearance, Edge* currMajorEdge)
                {
                    if (clearance < diameter)
                        return;

                    if (next->TriangleId == goalId)
                    {
                        if (!validGoalEdges.Contains(next->QuadEdgeId))
                            return;
                    }
                    else if (closed.Contains(next->TriangleId)) { // If next triangle has been visited, don't continue
                        var newStepDebugTest = new Step(null, next, steps.Length, step.G + C(step.ReferencePoint, next, out var refPDebug), H(refPDebug, goal3D), step.StepId, refPDebug, step.ReferencePoint);
                        
                        if (closedStepsDebug.ContainsKey(next->TriangleId)) {
                            Step edgePrevStep = closedStepsDebug[next->TriangleId];
                            Debug.Assert(edgePrevStep._gPlusH <= newStepDebugTest._gPlusH, "Testing");
                            Debug.Assert(edgePrevStep.G <= newStepDebugTest.G, "Testing 2");
                            if (edgePrevStep.G > newStepDebugTest.G) {
                                // DebugDrawArrow(Color.white);
                            }
                        }
                        return;
                    }
                    

                    var newStep = new Step
                    (
                        currMajorEdge,
                        next,
                        steps.Length,
                        step.G + C(step.ReferencePoint, next, out var referencePoint),
                        H(referencePoint, goal3D),
                        step.StepId,
                        referencePoint,
                        step.ReferencePoint
                    );

                    Step debugStep = newStep;
                    while (debugStep.Previous != -1)
                    {
                        DebugDrawArrow(steps[debugStep.Previous].ReferencePoint, debugStep.ReferencePoint, Color.black);
                        debugStep = steps[debugStep.Previous];
                    }

                    steps.Add(newStep);
                    open.Insert(newStep);
                }
            }

            return PathQueryState.NoPath;





            void AddEndpointEdges(float2 endpoint, Edge* edge, bool reverse)
            {
                var q = new Quad
                {
                    A = edge->Org->Point,
                    B = edge->Dest->Point,
                    C = Math.GetTangentRight(edge->Dest->Point, endpoint, radius),
                    D = Math.GetTangentLeft(edge->Org->Point, endpoint, radius)
                };

                DebugDraw(q, Color.yellow);

                var s = endpoint;
                var g = Math.ClosestPointOnLineSegment(s, edge->Org->Point, edge->Dest->Point);

                verts.Clear();
                ob.VerticesInQuad(q, edge, verts);

                for (int i = verts.Length - 1; i >= 0; i--)
                {
                    var v = verts[i];

                    if (v.Vertex == edge->Org || v.Vertex == edge->Dest)
                    {
                        verts.RemoveAt(i);
                        continue;
                    }

                    var p = v.Vertex->Point;
                    v.DistFromStart = math.distancesq(p, s);
                    CheckForDisturbance(p, s, g, out var opp, edge, diameter);
                    v.Opposite = opp;
                    verts[i] = v;
                }

                if (verts.Length > 1)
                    verts.Sort<PossibleDisturbance>();

                if (verts.Length > 0)
                {

                    for (int i = verts.Length - 1; i >= 0; i--)
                    {
                        var p = verts[i].Vertex->Point;

                        if (reverse) // goal
                        {
                            var gate = Math.CcwFast(s, g, p)
                                ? new Gate {Left = path[0].Left, Right = p, IsGoalGate = true}
                                : new Gate {Left = p, Right = path[0].Right, IsGoalGate = true};

                            DebugDraw(gate.Left, gate.Right, Color.white);
                            path.Insert(0, gate);
                        }
                        else
                        {
                            var gate = Math.CcwFast(s, g, p)
                                ? new Gate {Left = p, Right = path.Last().Right}
                                : new Gate {Left = path.Last().Left, Right = p};

                            DebugDraw(gate.Left, gate.Right, Color.magenta);
                            path.Add(gate);
                        }
                    }
                }
            }

            // TODO: Can ignore/comment out EndpointDisturbed for now anyways since we want agents that are clipping into obstacles to work
            void GetValidGoalEdges(List<int> valid)
            {
                valid.Clear();
                ValidEdge(goalEdge);
                ValidEdge(goalEdge->LNext);
                ValidEdge(goalEdge->LPrev);

                void ValidEdge(Edge* e)
                {
                    if (e->MainEdgeType != Edge.Type.Obstacle && !(e->HasMajorEdge && EndpointDisturbed(e->GetMajorEdge(), goal))) // TODO: EndpointDisturbed edge may be different from whats expected from Major edge
                        valid.Add(e->QuadEdgeId);
                }
            }

            void ExpandInitial(Edge* edge)
            {
                if (edge->MainEdgeType == Edge.Type.Obstacle || edge->TriangleId == goalId && !validGoalEdges.Contains(edge->QuadEdgeId))
                    return;

                if (edge->HasMajorEdge && EndpointDisturbed(edge->GetMajorEdge()->Sym, start)) // TODO: EndpointDisturbed edge may be different from whats expected from Major edge
                    return;

                var newStep = new Step
                (
                    null,
                    edge,
                    steps.Length,
                    C(start3D, edge, out float3 referencePoint),
                    H(referencePoint, goal3D),
                    -1,
                    referencePoint,
                    start3D
                );

                DebugDrawArrow(start3D, referencePoint, Color.black);

                steps.Add(newStep);
                open.Insert(newStep);
            }

            bool EndpointDisturbed(Edge* edgeMajor, float2 endpoint)
            {
                var q = new Quad
                {
                    A = edgeMajor->Org->Point,
                    B = edgeMajor->Dest->Point,
                    C = Math.GetTangentRight(edgeMajor->Dest->Point, endpoint, radius),
                    D = Math.GetTangentLeft(edgeMajor->Org->Point, endpoint, radius)
                };
                verts.Clear();
                ob.VerticesInQuad(q, edgeMajor, verts);
                for (int i = 0; i < verts.Length; i++)
                {
                    var v = verts[i];
                    var p = v.Vertex->Point;
                    if (CheckForDisturbance(p, endpoint, (edgeMajor->Org->Point + edgeMajor->Dest->Point) / 2, out _, edgeMajor, diameter))
                        return true;
                }

                return false;
            }

            // TODO: Use lengthsq instead? Also, add Heuristic weight?
            float C(PlanePoint from, Edge* next, out float3 referencePoint)
            {
                var o = next->Org->Point;
                var d = next->Dest->Point;
                var od = d - o; // Same as SegVector
                var offset = math.normalize(od) * radius;
                o += offset;
                d -= offset;
                if (!Math.IntersectSegSeg(from.point, goal, o, d, out float2 referencePoint2D))
                {
                    // metric 4 from paper
                    referencePoint2D = math.lengthsq(goal - o) < math.lengthsq(goal - d) ? o : d; // TODO: Not sure which one to use
                    // referencePoint2D = Math.ClosestPointOnLineSegment(goal/* from.point */, o, d);
                }
                referencePoint = new Seg(next->Org->Point3D, next->Dest->Point3D).PointGivenXZ(referencePoint2D);

                Debug.Assert(MathLib.IsEpsEqual(referencePoint2D, referencePoint.xz, 0.001f), $"referencePoint2D: {referencePoint2D}, referencePoint.xz: {referencePoint.xz}");

                Edge* traversedFace = next->Sym;
                float materialCost = materialTypes[traversedFace->TriangleMaterial].cost;
                float slopeCost = traversedFace->CalcSlopeCost();

                // Debug.Log($"slopeCost: {slopeCost}, materialCost: {materialCost}");
                Debug.Assert(materialCost * slopeCost >= 1f, $"materialCost * slopeCost: {materialCost * slopeCost}");
                return materialCost * slopeCost * math.length(referencePoint - from.Point3D);
            }

            float H(float3 p0, float3 p1)
            {
                return math.length(p1 - p0);
            }

            bool CheckForDisturbance(float2 v, float2 s, float2 g, out float2 opposite, Edge* tri, float d)
            {
                Edge* e;

                if (Math.Ccw(tri->Org->Point, tri->Dest->Point, v) && Math.ProjectSeg(tri->Org->Point, tri->Dest->Point, v, out _))
                    e = tri;
                else if (Math.Ccw(tri->LNext->Org->Point, tri->LNext->Dest->Point, v) && Math.ProjectSeg(tri->LNext->Org->Point, tri->LNext->Dest->Point, v, out _))
                    e = tri->LNext;
                else if (Math.Ccw(tri->LPrev->Org->Point, tri->LPrev->Dest->Point, v) && Math.ProjectSeg(tri->LPrev->Org->Point, tri->LPrev->Dest->Point, v, out _))
                    e = tri->LPrev;
                else
                {
                    if (math.any(tri->Org->Point != v) && math.any(tri->Dest->Point != v))
                        e = tri;
                    else if (math.any(tri->LNext->Org->Point != v) && math.any(tri->LNext->Dest->Point != v))
                        e = tri->LNext;
                    else if (math.any(tri->LPrev->Org->Point != v) && math.any(tri->LPrev->Dest->Point != v))
                        e = tri->LPrev;
                    else
                        throw new Exception();
                }

                var sgi = Math.ProjectLine(e->Org->Point, e->Dest->Point, v);
                opposite = (float2) (sgi + math.normalize(sgi - v) * d);

                var c = Navmesh.Navmesh.TryGetConstraint(math.length(opposite - v), v, e->Sym);

                if (c == null)
                    return false;

                opposite = (float2) Math.ProjectLine(c->Org->Point, c->Dest->Point, v);
                return math.length(opposite - v) < d && Math.IntersectSegSeg(v, opposite, s, g);
            }
        }
        static void DebugDrawArrow(float3 from, float3 to, Color color, float tangentOffset = 0f)
        {
            float3 offset = MathLib.CalcTangentToNormal(to - from)*tangentOffset;
            Arrow.Draw(from.XOZ(from.y + 0.01f) + offset, to.XOZ(to.y + 0.01f) + offset, 0.01f, color);
        }

        // [System.Diagnostics.Conditional("FUNNEL_DEBUG")]
        static void DebugDraw(float2 from, float2 to, Color color)
        {
            Line.Draw(from.ToXxY(.01f), to.ToXxY(.01f), color);
        }

        // [System.Diagnostics.Conditional("FUNNEL_DEBUG")]
        static void DebugDraw(Quad q, Color color)
        {
            DebugDraw(q.A, q.B, color);
            DebugDraw(q.B, q.C, color);
            DebugDraw(q.C, q.D, color);
            DebugDraw(q.D, q.A, color);
        }

        static bool EndpointValid(float2 p, float r, Edge* tri) // TODO: If vertices don't connect to obstacle edge this will stack overflow, hacky fix below
        {
            return EndpointValidRecursive(p, r, tri->Sym, tri->Sym, 0) &&
                   EndpointValidRecursive(p, r, tri->LNext->Sym, tri->LNext->Sym, 0) &&
                   EndpointValidRecursive(p, r, tri->LPrev->Sym, tri->LPrev->Sym, 0);
        }

        static bool EndpointValidRecursive(float2 p, float r, Edge* tri, Edge* startingTri, int depth)
        {
            return depth > 10 || (depth > 0 && (tri == startingTri || tri == startingTri->Sym)) || // TODO: First condition is a hack to fight infinite recursion stack overflow
                   (Math.IntersectSegCircle(tri->Org->Point, tri->Dest->Point, p, r) == 0 &&
                   math.length(p - tri->Org->Point) > r &&
                   math.length(p - tri->Dest->Point) > r) ||
                   (tri->MainEdgeType != Edge.Type.Obstacle &&
                   EndpointValidRecursive(p, r, tri->LNext->Sym, startingTri, depth + 1) &&
                   EndpointValidRecursive(p, r, tri->LPrev->Sym, startingTri, depth + 1));
        }

        static Quad TransformRect(float2 translation, float2 size, float angle)
        {
            var h = size / 2;
            return new Quad
            {
                A = Math.Rotate(-h, angle) + translation,
                B = Math.Rotate(new float2(h.x, -h.y), angle) + translation,
                C = Math.Rotate(h, angle) + translation,
                D = Math.Rotate(new float2(-h.x, h.y), angle) + translation
            };
        }

        // todo quad can be concave, use appropriate contains check below?
        void VerticesInQuad(Quad r, Edge* e, List<PossibleDisturbance> verts)
        {
            VerticesInQuadCheckOrg(r, e->Sym, verts);
            VerticesInQuadCheckOrg(r, e->LNext->Sym, verts);
            VerticesInQuadCheckOrg(r, e->LPrev->Sym, verts);
            _closedEdges.Clear();
            _closedVerts.Clear();
        }

        void VerticesInQuadCheckOrg(Quad r, Edge* e, List<PossibleDisturbance> verts)
        {
            var v = e->Org;
            if (Contains(r, v->Point) && !_closedVerts.Contains((IntPtr) v))
            {
                _closedVerts.Add((IntPtr) v);
                verts.Add(new PossibleDisturbance(v));
            }

            VerticesInQuadRecursive(r, e, verts);
        }

        void VerticesInQuadRecursive(Quad r, Edge* e, List<PossibleDisturbance> verts)
        {
            if (_closedEdges.Contains(e->QuadEdgeId))
                return;
            _closedEdges.Add(e->QuadEdgeId);

            if (Intersect(r, e->Org->Point, e->Dest->Point))
            {
                var v = e->LNext->Dest;
                if (Contains(r, v->Point) && !_closedVerts.Contains((IntPtr) v))
                {
                    _closedVerts.Add((IntPtr) v);
                    verts.Add(new PossibleDisturbance(v));
                }

                VerticesInQuadRecursive(r, e->LNext->Sym, verts);
                VerticesInQuadRecursive(r, e->LPrev->Sym, verts);
            }
        }

        static bool Intersect(Quad r, float2 p0, float2 p1)
        {
            return Contains(r, p0) ||
                   Contains(r, p1) ||
                   Math.IntersectSegSeg(r.A, r.B, p0, p1) ||
                   Math.IntersectSegSeg(r.B, r.C, p0, p1) ||
                   Math.IntersectSegSeg(r.C, r.D, p0, p1) ||
                   Math.IntersectSegSeg(r.D, r.A, p0, p1);
        }

        static bool Contains(Quad r, float2 p)
        {
            return GeometricPredicates.Orient2DFast(r.A, r.B, p) >= 0 &&
                   GeometricPredicates.Orient2DFast(r.B, r.C, p) >= 0 &&
                   GeometricPredicates.Orient2DFast(r.C, r.D, p) >= 0 &&
                   GeometricPredicates.Orient2DFast(r.D, r.A, p) >= 0;
        }

        public void Clear()
        {
            _open.Clear();
            _closed.Clear();
            _closedStepsDebug.Clear();
            _steps.Clear();
            _verts.Clear();
            _validGoalEdges.Clear();
        }

        public void Dispose()
        {
            _open.Dispose();
            _closed.Dispose();
            _closedStepsDebug.Dispose();
            _steps.Dispose();
            _verts.Dispose();
            _validGoalEdges.Dispose();
            _closedEdges.Dispose();
            _closedVerts.Dispose();
        }

        readonly struct Step : PriorityQueue<Step>.IElement
        {
            // todo confusing, probably better off making this StepId and add TriangleId, could forego index in priorityqueue
            public int Id => Edge->TriangleId;
            public readonly Edge* PrevMajorEdge;
            public readonly Edge* Edge;
            public readonly Edge.Type MainEdgeType;
            public readonly int StepId;
            public readonly float G;
            public readonly int Previous;
            public readonly PlanePoint ReferencePoint;
            public readonly float _gPlusH; // TODO: Made public for debug

            public readonly float debugDistance;


            public Step(Edge* prevMajorEdge, Edge* edge, int stepId, float g, float h, int previous, PlanePoint referencePoint, float3 prevPoint)
            {
                PrevMajorEdge = prevMajorEdge;
                Edge = edge;
                MainEdgeType = edge->MainEdgeType;
                StepId = stepId;
                G = g;
                _gPlusH = g + h;
                Previous = previous;
                ReferencePoint = referencePoint;

                debugDistance = math.length(referencePoint.Point3D - prevPoint);
            }

            public int CompareTo(Step other)
                => _gPlusH.CompareTo(other._gPlusH);
        }
    }
}
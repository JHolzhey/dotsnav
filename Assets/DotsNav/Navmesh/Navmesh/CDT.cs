using System.Diagnostics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using DotsNav.Core;
using System.Drawing;
using Unity.Entities.UniversalDelegates;

namespace DotsNav.Navmesh
{
    public unsafe partial struct Navmesh
    {
        static Edge* GetLeftEdge(Vertex* a, float2 p, bool isMajor) // Essentially just clamps between two edges around vertex such that p is between both, returns one that is to the left
        {
            var result = a->GetEdge(isMajor);
            var o = result->Org->Point;
            InfiniteLoopDetection.Reset();
            while (!Math.Ccw(o, p, result->Dest->Point))
            {
                InfiniteLoopDetection.Register(1000, "GetLeftEdge 0");
                result = result->ONext;
            }

            InfiniteLoopDetection.Reset();
            while (Math.Ccw(o, p, result->OPrev->Dest->Point))
            {
                InfiniteLoopDetection.Register(1000, "GetLeftEdge 1");
                result = result->OPrev;
            }

            return result;
        }

        static Edge* GetConnection(Vertex* a, Vertex* b, bool isMajor)
        {
            var e = a->GetEdgeEnumerator(isMajor);
            while (e.MoveNext())
                if (e.Current->Dest == b)
                    return e.Current;
            return null;
        }

        Edge* Connect(Vertex* a, Vertex* b, Edge.Type newEdgeType, Edge* majorEdge) // Connect two vertices with an edge
        {
            bool isMajor = Edge.IsEdgeTypeMajor(newEdgeType);
            Assert.IsTrue(a->GetEdge(isMajor) != null);
            Assert.IsTrue(b->GetEdge(isMajor) != null);
            return Connect(GetLeftEdge(a, b->Point, isMajor)->Sym, GetLeftEdge(b, a->Point, isMajor)->OPrev, newEdgeType, majorEdge);
        }

        // Flips edge e counterclockwise inside its enclosing quadrilateral.
        // http://karlchenofhell.org/cppswp/lischinski.pdf
        void Swap(Edge* e)
        {
            UnityEngine.Debug.Log("Swap");
            UnityEngine.Debug.Assert(!e->Constrained && e->EdgeType.HasNoFlagsB(Edge.Type.Obstacle | Edge.Type.Terrain)
                && !e->EdgeType.HasAllFlagsB(Edge.Type.Clearance | Edge.Type.Minor)
                , $"Cannot flip a constrained edge. EdgeType: {e->EdgeType}, e->Constrained: {e->Constrained}");
            
            bool isMajor = Edge.IsEdgeTypeMajor(e->EdgeType);
            if (isMajor) {
                _modifiedMajorEdges.TryAdd((IntPtr)e);
            }

            e->Org->RemoveEdge(e, isMajor);
            e->Dest->RemoveEdge(e->Sym, isMajor);

            V.TryAdd((IntPtr) e->Org);
            V.TryAdd((IntPtr) e->Dest);

            var a = e->OPrev;
            var b = e->Sym->OPrev;
            Splice(e, a); // These two cuts ties with original Org and Dest vertices and corresponding edges
            Splice(e->Sym, b);

            Splice(e, a->LNext); // These two merge with the new Org and Dest which are the opposite counterclockwise vertices in the quad
            Splice(e->Sym, b->LNext);
            
            SetEndPoints(e, a->Dest, b->Dest, isMajor);

            V.TryAdd((IntPtr) a->Dest);
            V.TryAdd((IntPtr) b->Dest);

            DestroyedTriangle(e->TriangleId);
            DestroyedTriangle(e->Sym->TriangleId);

            NewTriangle(e);
            NewTriangle(e->Sym);
        }

        static void SetEndPoints(Edge* edge, Vertex* org, Vertex* dest, bool isMajor)
        {
            Debug.Assert(Edge.IsEdgeTypeMajor(edge->EdgeType) == isMajor);
            SetOrg(edge, org, isMajor);
            SetOrg(edge->Sym, dest, isMajor);
        }

        static void SetOrg(Edge* edge, Vertex* v, bool isMajor)
        {
            Assert.IsTrue(v != null);
            edge->Org = v;
            v->AddEdge(edge, isMajor);
        }

        // https://www.researchgate.net/publication/2478154_Fully_Dynamic_Constrained_Delaunay_Triangulations
        void FlipEdges(float2 p, Edge.Type newConstraintEdgeType) // Calls Swap
        {
            UnityEngine.Debug.Log($"FlipEdges: {newConstraintEdgeType}");
            while (_flipStack.Count > 0)
            {
                var e = _flipStack.Pop();
                Assert.IsTrue(Math.Ccw(e->Org->Point, e->Dest->Point, p));

                /* bool isSwap = false; // TODO: Could potentially get rid of all of this: (But also don't allow flipping a Minor Clearance Edge)
                if (constraintEdgeType == ConstraintType.Obstacle && !e->Constrained) {
                    UnityEngine.Debug.Assert(e->EdgeType == EdgeType.Obstacle || e->EdgeType == EdgeType.ConnectsToObstacle, $"EdgeType: {e->EdgeType}");
                    isSwap = true;
                } else if (constraintEdgeType == ConstraintType.Terrain && e->EdgeType == EdgeType.ConnectsToTerrain) {
                    UnityEngine.Debug.Assert(!e->Constrained, "Obvious, can delete this check");
                    isSwap = true;
                } */

                bool isSwap = false;
                if (Edge.IsEdgeTypeMajor(newConstraintEdgeType)) {
                    UnityEngine.Debug.Assert(Edge.IsEdgeTypeMajor(e->EdgeType));
                    UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(e->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle), e->Constrained));
                    isSwap = e->EdgeType.HasNoFlagsB(Edge.Type.Obstacle); // If is a major Obstacle
                } else { // constraint is Minor
                    UnityEngine.Debug.Assert(!Edge.IsEdgeTypeMajor(newConstraintEdgeType) && !Edge.IsEdgeTypeMajor(e->EdgeType));
                    UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(e->EdgeType.HasAnyFlagsB(Edge.Type.Terrain), e->Constrained));
                    isSwap = e->EdgeType.HasNoFlagsB(Edge.Type.Terrain | Edge.Type.Obstacle | Edge.Type.Clearance);
                }

                if (/* !e->Constrained */ isSwap && Math.CircumcircleContains(e->Org->Point, e->Dest->Point, p, e->DNext->Org->Point))
                {
                    _flipStack.Push(e->OPrev);
                    _flipStack.Push(e->DNext);
                    Assert.IsTrue(Math.Ccw(e->OPrev->Org->Point, e->OPrev->Dest->Point, p));
                    Assert.IsTrue(Math.Ccw(e->DNext->Org->Point, e->DNext->Dest->Point, p));
                    Swap(e);
                }
            }
        }

        // http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.61.3862&rep=rep1&type=pdf
        // TriangulatePseudopolygonDelaunay()
        void RetriangulateFace(Edge* edge, Edge.Type minorOrMajor)
        {
            UnityEngine.Debug.Log("RetriangulateFace");
            Assert.IsTrue(edge != null);
            Assert.IsTrue(edge != edge->LNext->LNext);
            if (edge->LNext->LNext->LNext == edge)
            {
                NewTriangle(edge); // Just inits TriangleIds
                return;
            }

            InfiniteLoopDetection.Reset();
            while (!Math.Ccw(edge->Org->Point, edge->Dest->Point, edge->LNext->Dest->Point))
            {
                InfiniteLoopDetection.Register(1000, "RetriangulateFace 0");
                edge = edge->LNext;
            }

            var c = edge->LNext;
            var e = c;
            InfiniteLoopDetection.Reset();
            while (true)
            {
                InfiniteLoopDetection.Register(1000, "RetriangulateFace 1");
                e = e->LNext;
                if (e->LNext == edge)
                    break;
                if (Math.CircumcircleContains(edge->Org->Point, edge->Dest->Point, c->Dest->Point, e->Dest->Point))
                    c = e;
            }

            Edge.Type newConnectionEdgesType = Edge.Type.None;
            if (Edge.IsEdgeTypeMajor(minorOrMajor)) { // All Edges connecting to Major (i.e. an Obstacle) constraint are Clearance edges
                newConnectionEdgesType = Edge.Type.Clearance | Edge.Type.Major;
            } else { // All Edges connecting to Minor constraints are TerrainSub edges
                newConnectionEdgesType = Edge.Type.TerrainSub | Edge.Type.Minor;
            }

            Assert.IsTrue(c != edge);
            var connected = false;
            if (c->LNext->LNext != edge)
            {
                V.TryAdd((IntPtr) edge->LPrev->Dest);
                V.TryAdd((IntPtr) c->LNext->Org);

                var b = Connect(edge->LPrev, c->LNext, newConnectionEdgesType, null);
                RetriangulateFace(b, minorOrMajor);
                connected = true;
            }

            if (c != edge->LNext)
            {
                V.TryAdd((IntPtr) c->Dest);
                V.TryAdd((IntPtr) edge->LNext->Org);

                var a = Connect(c, edge->LNext, newConnectionEdgesType, null);
                RetriangulateFace(a, minorOrMajor);
                connected = true;
            }

            if (connected)
                NewTriangle(edge);
        }

        Edge* RemoveVertex(Vertex* vert)
        {
            UnityEngine.Debug.Log("RemoveVertex");
            Assert.IsTrue(vert->GetEdge(true) != null); // TODO: True
            Assert.IsTrue(vert->GetEdge(true)->Org == vert); // TODO: True

            var remaining = vert->GetEdge(true)->LNext; // TODO: True
            InfiniteLoopDetection.Reset();
            while (true)
            {
                InfiniteLoopDetection.Register(1000, "RemoveVertex");
                var majorE = vert->GetEdge(true); // TODO: True
                if (majorE != null) {
                    RemoveEdge(majorE, true, true);
                }
                var minorE = vert->GetEdge(false);
                if (minorE != null) {
                    RemoveEdge(majorE, true, false);
                }
                if (majorE == null && minorE == null) {
                    break;
                }
            }

            V.Remove((IntPtr) vert);

            Assert.IsTrue(vert->GetEdge(false) == null && vert->GetEdge(true) == null); // Ensure edges have all been removed
            _qt.Remove(vert);
            var delPos = vert->SeqPos;
            ((Vertex*) _verticesSeq[^1])->SeqPos = delPos;
            _verticesSeq.RemoveAtSwapBack(delPos);
            _vertices.Recycle(vert);
            return remaining;
        }

        /// <summary>
        /// Returns an edge for which the specified point is contained within it's left face. If the point lies
        /// on an edge this edge is returned. If the point lies on a vertex an arbitrary edge with identical origin is returned.
        /// </summary>
        public Edge* FindTriangleContainingPoint(float2 p) => FindTriangleContainingPoint(p, out _);

        /// <summary>
        /// Returns an edge for which the specified point is contained within it's left face. If the point lies
        /// on an edge this edge is returned. If the point lies on a vertex an arbitrary edge with identical origin is returned.
        /// </summary>
        /// <param name="collinear">True when the specified point lies on the returned edge</param>
        public Edge* FindTriangleContainingPoint(float2 p, out bool collinear)
        {
            var e = FindClosestVertex(p)->GetEdge(true); // TODO: True
            Assert.IsTrue(e != null);
            InfiniteLoopDetection.Reset();
            while (true)
            {
                InfiniteLoopDetection.Register(1000, "FindTriangleContainingPoint");

                Edge* collinearEdge = null;
                var orient = Math.TriArea(e->Org->Point, e->Dest->Point, p);

                if (orient == 0)
                {
                    collinearEdge = e;
                }
                else if (orient < 0)
                {
                    e = e->Sym;
                    continue;
                }

                orient = Math.TriArea(e->ONext->Org->Point, e->ONext->Dest->Point, p);

                if (orient == 0)
                {
                    collinearEdge = e->ONext;
                }
                else if (orient > 0)
                {
                    e = e->ONext;
                    continue;
                }

                orient = Math.TriArea(e->DPrev->Org->Point, e->DPrev->Dest->Point, p);

                if (orient == 0)
                {
                    collinear = true;
                    return e->DPrev;
                }

                if (orient > 0)
                {
                    e = e->DPrev;
                    continue;
                }

                if (collinearEdge != null)
                {
                    collinear = true;
                    return collinearEdge;
                }

                collinear = false;
                return e;
            }
        }

        /// <summary>
        /// Returns a pointer to the vertex closest to the specified point
        /// </summary>
        public Vertex* FindClosestVertex(float2 p)
        {
            Assert.IsTrue(Contains(p), "Trying to find the closest vertex to a point outside the navmesh");
            return _qt.FindClosest(p);
        }

        Vertex* InsertPoint(float2 p, Edge.Type newConstraintEdgeType, Vertex* existingVertex = null)
        {
            UnityEngine.Debug.Log($"InsertPoint; {newConstraintEdgeType}, Vertex.Type: {(Vertex.Type)(newConstraintEdgeType & (Edge.Type.Major | Edge.Type.Minor))}");
            var closest = _qt.FindClosest(p, (Vertex.Type)(newConstraintEdgeType & (Edge.Type.Major | Edge.Type.Minor))); // Extract the Major or Minor bit

            //if (newConstraintEdgeType == ConstraintType.Obstacle && closest->GetEdge(true) == null) {
            //    Edge* minorEdge = closest->GetEdge(false);
            //    Assert.IsTrue(minorEdge != null, "If major edge doesn't exist, there must be a minor edge");
            //    //Edge* minorEdgesMajorEdge = minorEdge->MajorEdge;
            //    // TODO: Unfinished - TODO: Wtf was I doing here? I think this is old, delete
            //}

            if (math.lengthsq(closest->Point - p) <= _e * _e) {
                UnityEngine.Debug.Log($"InsertPoint - Found vertex, returning it instead of inserting new. Vertex.Type: {closest->VertexType}");
                UnityEngine.Debug.Assert((newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor) && closest->GetEdge(false) != null)
                    || (newConstraintEdgeType.HasAllFlagsB(Edge.Type.Major) && closest->GetEdge(true) != null));

                return closest;
            }

            // ConstraintType betterConstraintType = newConstraintEdgeType == ConstraintType.MajorEdgeInsertedToMinorGraph ? ConstraintType.Terrain : constraintEdgeType;
            var e = closest->GetEdge(Edge.IsEdgeTypeMajor(newConstraintEdgeType));
            Assert.IsTrue(e != null);
            InfiniteLoopDetection.Reset();

            while (true)
            {
                InfiniteLoopDetection.Register(1000, "InsertPoint");

                Edge* inEdge = null;

                var orient = Math.TriArea(e->Org->Point, e->Dest->Point, p);

                if (orient == 0)
                    inEdge = e;

                if (orient < 0)
                {
                    e = e->Sym;
                    continue;
                }

                orient = Math.TriArea(e->ONext->Org->Point, e->ONext->Dest->Point, p);

                if (orient == 0)
                    inEdge = e->ONext;

                if (orient > 0)
                {
                    e = e->ONext;
                    continue;
                }

                orient = Math.TriArea(e->DPrev->Org->Point, e->DPrev->Dest->Point, p);

                if (orient == 0)
                    inEdge = e->DPrev;

                if (orient > 0)
                {
                    e = e->DPrev;
                    continue;
                }

                if (inEdge != null)
                {
                    Assert.IsTrue(SplitIsRobust(p, inEdge));
                    return InsertPointInEdge(p, inEdge, /* betterConstraintType */ newConstraintEdgeType, existingVertex);
                }

                return InsertPointInFace(p, e, /* betterConstraintType */ newConstraintEdgeType, existingVertex);
            }
        }

        Vertex* InsertPointInEdge(float2 point, Edge* edge, Edge.Type newConstraintEdgeType, Vertex* existingVertex = null)
        {
            UnityEngine.Debug.Log("InsertPointInEdge");
            _flipStack.Push(edge->ONext->Sym);
            _flipStack.Push(edge->DPrev->Sym);
            _flipStack.Push(edge->OPrev);
            _flipStack.Push(edge->DNext);

            for (var i = 0; i < _flipStack.Count; i++) {
                Assert.IsTrue(Math.Ccw(_flipStack[i]->Org->Point, _flipStack[i]->Dest->Point, point));
            }

            DestroyedTriangle(edge->TriangleId);
            DestroyedTriangle(edge->Sym->TriangleId);

            var crep = edge->QuadEdge->Crep;
            var e = edge->OPrev;
            C.Remove((IntPtr) edge);


            UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Obstacle), edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle)), $"edge->EdgeType: {edge->EdgeType}");
            UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Terrain), newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Minor)
                && edge->EdgeType.HasAnyFlagsB(Edge.Type.Minor) && edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Terrain | Edge.Type.Clearance)), $"edge->EdgeType: {edge->EdgeType}");
            UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Clearance), newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Minor)
                && edge->EdgeType.HasAnyFlagsB(Edge.Type.Minor) && edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Terrain | Edge.Type.Clearance)), $"edge->EdgeType: {edge->EdgeType}");

            Edge.Type newConnectionEdgesType = Edge.Type.None;
            if (Edge.IsEdgeTypeMajor(newConstraintEdgeType)) { // Major Obstacles only intersect with Major Obstacles
                UnityEngine.Debug.Assert(edge->EdgeType.HasAllFlagsB(Edge.Type.Major | Edge.Type.Obstacle) && edge->EdgeType == newConstraintEdgeType);
                newConnectionEdgesType = Edge.Type.Major | Edge.Type.Clearance;
            } else { // is Minor edge
                UnityEngine.Debug.Assert((edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Obstacle) || edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance)
                    || edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Terrain))
                    && (newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Obstacle) || newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Terrain)));
                // Should not be intersecting Minor Obstacle with Minor Clearance
                UnityEngine.Debug.Assert(!(edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance) && newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Obstacle)));
                newConnectionEdgesType = Edge.Type.Minor | Edge.Type.TerrainSub;
            }
            QuadEdge* newMajorEdge = null;
            Edge* majorEdge = edge->MajorEdge;
            Edge.Type newSplitConstraintEdgesType = edge->EdgeType;
            RemoveEdge(edge, false, true);

            /* if (constraintEdgeType == ConstraintType.Terrain && edge->EdgeType == EdgeType.ConnectsToObstacle) {
                newMajorEdge = edge->QuadEdge; // Don't remove edge, make it the majorEdge of new subEdges
                edge->SetEdgeType(EdgeType.ConnectsToObstacleWithMinorTerrainConnects);
                newEdgesType = EdgeType.ConnectsToTerrainWithMajorConnectsToObstacle;
                
                // edge->Org->RemoveEdge(edge, ); // If vertex points to edge as head, then remove it
                // edge->Dest->RemoveEdge(edge->Sym, );

                //RemoveEdge(edge, false, false); // Debug
            }
            else if (constraintEdgeType == ConstraintType.Terrain && edge->EdgeType == EdgeType.ConnectsToTerrainWithMajorConnectsToObstacle) {
                newMajorEdge = edge->MajorEdge;
                newEdgesType = EdgeType.ConnectsToTerrainWithMajorConnectsToObstacle;
                RemoveEdge(edge, false, false);
            }
            else { // if (constraintEdgeType == ConstraintType.Obstacle && edge->EdgeType == EdgeType.Obstacle)
                RemoveEdge(edge, false, true);
                newEdgesType = edge->EdgeType;
            } */


            var result = existingVertex != null ? existingVertex : CreateVertex(point);
            V.TryAdd((IntPtr) result);
            V.TryAdd((IntPtr) e->Org);
            var newEdge = CreateEdge(e->Org, result, newSplitConstraintEdgesType, majorEdge);
            newEdge->QuadEdge->Crep = GetCrep(crep);
            Splice(newEdge, e);

            V.TryAdd((IntPtr) e->Dest);
            V.TryAdd((IntPtr) newEdge->Sym->Org);

            newEdge = Connect(e, newEdge->Sym, newConnectionEdgesType, null);
            e = newEdge->OPrev;

            V.TryAdd((IntPtr) e->Dest);
            V.TryAdd((IntPtr) newEdge->Sym->Org);

            newEdge = Connect(e, newEdge->Sym, newSplitConstraintEdgesType, majorEdge);
            newEdge->QuadEdge->Crep = crep;
            e = newEdge->OPrev;

            V.TryAdd((IntPtr) e->Dest);
            V.TryAdd((IntPtr) newEdge->Sym->Org);

            Connect(e, newEdge->Sym, newConnectionEdgesType, null);

            var te = result->GetEdge(Edge.IsEdgeTypeMajor(newConstraintEdgeType));
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);

            FlipEdges(point, newConstraintEdgeType);
            return result;
        }

        UnsafeList<Entity> GetCrep(UnsafeList<Entity> source)
        {
            var l = GetCrep();
            l.AddRange(source);
            return l;
        }

        UnsafeList<Entity> GetCrep() => _creps.Count > 0 ? _creps.Pop() : new UnsafeList<Entity>(CrepMinCapacity, Allocator.Persistent);

        Vertex* InsertPointInFace(float2 p, Edge* edge, Edge.Type newConstraintEdgeType, Vertex* existingVertex = null) // TODO: Perhaps make separate function to avoid existingVertex branch eventually
        {
            UnityEngine.Debug.Log("InsertPointInFace");
            _flipStack.Push(edge->ONext->Sym);
            _flipStack.Push(edge);
            _flipStack.Push(edge->DPrev->Sym);

            for (var i = 0; i < _flipStack.Count; i++)
                Assert.IsTrue(Math.Ccw(_flipStack[i]->Org->Point, _flipStack[i]->Dest->Point, p));

            DestroyedTriangle(edge->TriangleId);

            var result = existingVertex != null ? existingVertex : CreateVertex(p);

            V.TryAdd((IntPtr) result);
            V.TryAdd((IntPtr) edge->Org);
            V.TryAdd((IntPtr) edge->Dest);
            V.TryAdd((IntPtr) edge->LNext->Dest);

            Edge.Type newConnectionEdgesType = Edge.Type.None;
            if (Edge.IsEdgeTypeMajor(newConstraintEdgeType)) { // All Edges connecting to Major (i.e. an Obstacle) constraint are Clearance edges
                UnityEngine.Debug.Assert(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Obstacle), $"newConstraintEdgeType: {newConstraintEdgeType}");
                newConnectionEdgesType = Edge.Type.Clearance | Edge.Type.Major;
                UnityEngine.Debug.Log("WTFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
            } else { // All Edges connecting to Minor constraints are TerrainSub edges
                UnityEngine.Debug.Assert(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Terrain | Edge.Type.Obstacle | Edge.Type.Clearance), $"newConstraintEdgeType: {newConstraintEdgeType}");
                newConnectionEdgesType = Edge.Type.TerrainSub | Edge.Type.Minor;
            }

            var newEdge = CreateEdge(edge->Org, result, newConnectionEdgesType, null);
            Splice(newEdge, edge);
            newEdge = Connect(edge, newEdge->Sym, newConnectionEdgesType, null);
            Connect(newEdge->OPrev, newEdge->Sym, newConnectionEdgesType, null);

            var te = result->GetEdge(Edge.IsEdgeTypeMajor(newConstraintEdgeType));
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);

            FlipEdges(p, newConstraintEdgeType);
            return result;
        }

        struct Point
        {
            public Vertex* Vertex;
            public bool Modified;
            public bool FoundExisting;
            public float2 Before;
            public float2 After;
            public float2 P;

            public override string ToString()
                => $"Vert: {Vertex->ToString()}, P: {P}";
        }

        void InsertSegment(Vertex* a, Vertex* b, Entity id, Edge.Type newConstraintEdgeType, Edge* majorEdge)
        {
            UnityEngine.Debug.Log($"InsertSegment; {newConstraintEdgeType}");
            var dir = math.normalize(b->Point - a->Point);
            InsertSegmentRecursive(a, b, id, dir, a->Point, b->Point, newConstraintEdgeType, majorEdge);
        }

        void InsertSegmentRecursive(Vertex* a, Vertex* b, Entity id, float2 dir, float2 start, float2 end, Edge.Type newConstraintEdgeType, Edge* majorEdge)
        {
            UnityEngine.Debug.Log($"InsertSegmentRecursive; {newConstraintEdgeType}");
            _insertedPoints.Clear();
            _insertedPoints.Add(new Point {Vertex = a, P = a->Point});

            while (a != b)
            {
                var p0 = _insertedPoints[^1];
                var p1 = GetNextPoint(a, b, start, end, newConstraintEdgeType);

                if (!p0.Modified && !p1.Modified)
                {
                    if (p0.FoundExisting || p1.FoundExisting) {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - neither modified, both found existing");
                        InsertSegmentRecursive(p0.Vertex, p1.Vertex, id, dir, start, end, newConstraintEdgeType, majorEdge);
                    } else {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - neither modified, neither found existing");
                        InsertSegmentNoCrossConstraints(p0.Vertex, p1.Vertex, id, newConstraintEdgeType, majorEdge);
                    }
                }
                else if (p0.Modified && !p1.Modified)
                {
                    if (GetSupport(p0.After + _e / 2 * dir, p1.P - _e * dir, dir, out var p))
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - p0 modified, insert point");
                        var after = InsertPoint(p, newConstraintEdgeType);
                        InsertSegmentRecursive(after, p1.Vertex, id, dir, start, end, newConstraintEdgeType, majorEdge);
                        _edgeSearch.Search(p0.Vertex, after, id);
                    }
                    else
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - p0 modified");
                        _edgeSearch.Search(p0.Vertex, p1.Vertex, id);
                    }
                }
                else if (!p0.Modified) // p1 modified
                {
                    if (GetSupport(p1.Before - _e / 2 * dir, p0.P + _e * dir, -dir, out var p))
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - p1 modified, insert point");
                        var before = InsertPoint(p, newConstraintEdgeType);
                        InsertSegmentRecursive(p0.Vertex, before, id, dir, start, end, newConstraintEdgeType, majorEdge);
                        _edgeSearch.Search(before, p1.Vertex, id);
                    }
                    else
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - p1 modified");
                        _edgeSearch.Search(p0.Vertex, p1.Vertex, id);
                    }
                }
                else // both modified
                {
                    if (GetSupport(p0.After + _e / 2 * dir, p1.P - _e * dir, dir, out var s1) &&
                        GetSupport(p1.Before - _e / 2 * dir, s1 + _e * dir, -dir, out var s2))
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - both modified, double insert point");
                        var v0 = InsertPoint(s1, newConstraintEdgeType);
                        var v1 = InsertPoint(s2, newConstraintEdgeType);
                        InsertSegmentRecursive(v0, v1, id, dir, start, end, newConstraintEdgeType, majorEdge);
                        _edgeSearch.Search(p0.Vertex, v0, id);
                        _edgeSearch.Search(v1, p1.Vertex, id);
                    }
                    else
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - both modified");
                        _edgeSearch.Search(p0.Vertex, p1.Vertex, id);
                    }
                }

                a = p1.Vertex;
                _insertedPoints.Add(p1);
            }
        }

        Point GetNextPoint(Vertex* a, Vertex* b, float2 start, float2 end, Edge.Type newConstraintEdgeType)
        {
            UnityEngine.Debug.Log($"GetNextPoint; {newConstraintEdgeType}");
            InfiniteLoopDetection.Reset();

            // bool isMajor = false;
            // if (constraintEdgeType == ConstraintType.MajorEdgeInsertedToMinorGraph) {
            //     isMajor = true;
            // } else {
            //     isMajor = Edge.IsEdgeTypeMajor((EdgeType)constraintEdgeType);
            // }
            var e = GetLeftEdge(a, b->Point, Edge.IsEdgeTypeMajor(newConstraintEdgeType));
            while (e->Dest != b)
            {
                InfiniteLoopDetection.Register(1000, "GetNextPoint");

                var d = Math.TriArea(a->Point, b->Point, e->Dest->Point);

                if (d < 0 && e->Constrained) // TODO
                // if (d < 0 && MathLib.IfFirstCheckSecond(constraintEdgeType == ConstraintType.Obstacle || constraintEdgeType == ConstraintType.MajorEdgeInsertedToMinorGraph, e->Constrained))
                {
                    var p = (float2) Math.IntersectLineSegClamped(start, end, e->Org->Point, e->Dest->Point);
                    var pointExists = TryGetPoint(p, e, out var v);

                    if (v != null)
                    {
                        if (_insertedPoints.Length > 1)
                        {
                            var prev = _insertedPoints[^1].Vertex;
                            if (prev == v || e->Org == prev || e->Dest == prev)
                                continue;
                        }

                        if (_insertedPoints.Length > 2)
                        {
                            var prev = _insertedPoints[^2].Vertex;
                            if (prev == v || e->Org == prev || e->Dest == prev)
                                continue;
                        }

                        return new Point
                        {
                            Vertex = v,
                            FoundExisting = true,
                            P = p
                        };
                    }

                    if (pointExists || !SplitIsRobust(p, e))
                    {
                        UnityEngine.Debug.Log("GetNextPoint - CreatePRef");
                        var pRef = CreatePRef(p, e, newConstraintEdgeType);

                        if (_insertedPoints.Length > 1 && _insertedPoints[^1].Vertex == pRef)
                            continue;

                        var point = new Point
                        {
                            Vertex = pRef,
                            Modified = true,
                            P = p
                        };

                        var proj = (float2) Math.ProjectLine(a->Point, b->Point, point.Vertex->Point);
                        var pproj = proj - p;

                        if (math.dot(b - a, pproj) < 0)
                        {
                            point.Before = proj;
                            point.After = p;
                        }
                        else
                        {
                            point.Before = p;
                            point.After = proj;
                        }

                        return point;
                    }

                    var vert = InsertPointInEdge(p, e, newConstraintEdgeType);
                    return new Point
                    {
                        Vertex = vert,
                        P = p
                    };
                }

                e = d > 0 ? e->RPrev : e->ONext;
            }

            return new Point
            {
                Vertex = b,
                P = b->Point
            };
        }

        Vertex* CreatePRef(float2 p, Edge* e, Edge.Type newConstraintEdgeType)
        {
            UnityEngine.Debug.Log($"CreatePRef; {newConstraintEdgeType}");
            var stepSize = _e / 2;
            var po = e->Org->Point - p;
            var pd = e->Dest->Point - p;
            var dir = math.normalize(e->Dest->Point - e->Org->Point);
            var lpo = math.length(po) - _e;
            var lpd = math.length(pd) - _e;
            var offset = 0f;

            InfiniteLoopDetection.Reset();
            while (true)
            {
                InfiniteLoopDetection.Register(1000, "CreatePRef");

                offset += stepSize;

                if (offset >= lpo)
                    return e->Org;

                if (offset >= lpd)
                    return e->Dest;

                var pplus = p + offset * dir;
                var pointPresent = TryGetPoint(pplus, e, out var vertex);

                if (vertex != null)
                    return vertex;

                if (!pointPresent && SplitIsRobust(pplus, e))
                    return InsertPointInEdge(pplus, e, newConstraintEdgeType);

                var pmin = p - offset * dir;
                pointPresent = TryGetPoint(pmin, e, out vertex);

                if (vertex != null)
                    return vertex;

                if (!pointPresent && SplitIsRobust(pmin, e))
                    return InsertPointInEdge(pmin, e, newConstraintEdgeType);
            }
        }

        // todo qt is queried here and at callsite through InsertPoint
        bool GetSupport(float2 a, float2 b, float2 dir, out float2 p)
        {
            if (math.dot(b - a, dir) < 0)
            {
                p = default;
                return false;
            }

            var stepSize = _e / 2;
            var l = math.length(b - a);
            var offset = 0f;

            while (true)
            {
                p = a + offset * dir;
                // todo cache leaf node, these points are probably in the same bucket
                var closest = _qt.FindClosest(p);
                if (math.lengthsq(closest->Point - p) > _e * _e)
                    return true;

                offset += stepSize;
                if (offset >= l)
                {
                    p = default;
                    return false;
                }
            }
        }

        // Means: 'Insert Segment when there are no constraints between a and b'
        void InsertSegmentNoCrossConstraints(Vertex* a, Vertex* b, Entity id, Edge.Type newConstraintEdgeType, Edge* majorEdge)
        {
            UnityEngine.Debug.Log($"InsertSegmentNoCrossConstraints; {newConstraintEdgeType}");
            var c = GetConnection(a, b, Edge.IsEdgeTypeMajor(newConstraintEdgeType));

            if (c != null)
            {
                C.TryAdd((IntPtr) c);
                if (!c->IsConstrainedBy(id))
                    c->AddConstraint(id);
                ResetClearance(c);
                c->SetEdgeType(newConstraintEdgeType);
                UnityEngine.Debug.Log($"InsertSegmentNoCrossConstraints - Found existing edge, setting it to EdgeType: {newConstraintEdgeType}");
                return;
            }

            var e = GetLeftEdge(a, b->Point, Edge.IsEdgeTypeMajor(newConstraintEdgeType));

            InfiniteLoopDetection.Reset();
            while (e->Dest != b)
            {
                InfiniteLoopDetection.Register(1000, "InsertSegmentNoCrossConstraints");

                var d = Math.TriArea(a->Point, b->Point, e->Dest->Point);
                var next = d > 0 ? e->RPrev : e->ONext;

                if (d < 0)
                {
                    UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints - Weird thing -> Removing edge");
                    Assert.IsTrue(!e->Constrained);
                    RemoveEdge(e, true, Edge.IsEdgeTypeMajor(newConstraintEdgeType));
                }
                else if (d == 0 && e->Dest != a)
                {
                    var t = e->Dest;
                    UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints - Connect");
                    Connect(a, t, id, newConstraintEdgeType, majorEdge);
                    a = t;
                }

                e = next;
            }

            UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints - Connect #2");
            Connect(a, b, id, newConstraintEdgeType, majorEdge);
        }

        void Connect(Vertex* a, Vertex* b, Entity id, Edge.Type newEdgeType, Edge* majorEdge)
        {
            var connection = GetConnection(a, b, Edge.IsEdgeTypeMajor(newEdgeType));
            if (connection == null)
            {
                UnityEngine.Debug.Log($"Connect - null connection, so creating one with EdgeType: {newEdgeType}");
                V.TryAdd((IntPtr) a);
                V.TryAdd((IntPtr) b);

                connection = Connect(a, b, newEdgeType, majorEdge);
                RetriangulateFace(connection, newEdgeType);
                RetriangulateFace(connection->Sym, newEdgeType);
            }

            // todo inline wasUnconstrained (so if moves above addconstraint)
            var wasUnconstrained = !connection->Constrained;
            connection->AddConstraint(id);
            if (wasUnconstrained)
                ResetClearance(connection);
            connection->SetEdgeType(newEdgeType);
            C.TryAdd((IntPtr) connection);
        }

        void Connect(Vertex* a, Vertex* b, UnsafeList<Entity> crep, Edge.Type newEdgeType, Edge* majorEdge)
        {
            var connection = GetConnection(a, b, Edge.IsEdgeTypeMajor(newEdgeType));
            if (connection == null)
            {
                V.TryAdd((IntPtr) a);
                V.TryAdd((IntPtr) b);

                connection = Connect(a, b, newEdgeType, majorEdge);
                RetriangulateFace(connection, newEdgeType);
                RetriangulateFace(connection->Sym, newEdgeType);
            }

            connection->QuadEdge->Crep = crep;
            ResetClearance(connection);
            C.TryAdd((IntPtr) connection);
        }
    
        bool TryGetPoint(float2 p, Edge* e, out Vertex* v)
        {
            UnityEngine.Debug.Log("TryGetPoint");
            v = null;
            var closest = _qt.FindClosest(p);

            if (math.lengthsq(closest->Point - p) <= _e * _e)
            {
                var te = closest->GetEdge(Edge.IsEdgeTypeMajor(e->EdgeType));
                do
                {
                    if (te->QuadEdge == e->QuadEdge)
                    {
                        v = closest;
                        break;
                    }

                    te = te->ONext;
                } while (te != closest->GetEdge(Edge.IsEdgeTypeMajor(e->EdgeType)));

                return true;
            }

            return false;
        }

        static readonly FixedString128Bytes PointOutsideNavmeshMessage = "Trying to add a point outside the navmesh";


        internal void InsertMajorIntoMinor(Edge* edgeMajor, Entity cid)
        {
            Edge.Type newConstraintEdgeType = Edge.Type.Minor | (edgeMajor->EdgeType & ~Edge.Type.Major); // Force to be Minor Edge
            UnityEngine.Debug.Assert(newConstraintEdgeType.HasNoFlagsB(Edge.Type.Major) && newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor), $"poop; {newConstraintEdgeType}");
            UnityEngine.Debug.Log($"InsertMajorIntoMinor; {newConstraintEdgeType}");
            Edge.VerifyEdgeType(newConstraintEdgeType, false);
            Assert.IsTrue(edgeMajor != null);

            Vertex* vertMajor0 = edgeMajor->Org;
            Vertex* vertMajor1 = edgeMajor->Dest;
            Assert.IsTrue(vertMajor0->ContainsEdge(edgeMajor, true) && vertMajor1->ContainsEdge(edgeMajor->Sym, true));

            var vertMinor0 = InsertPoint(vertMajor0->Point, newConstraintEdgeType, vertMajor0);
            var vertMinor1 = InsertPoint(vertMajor1->Point, newConstraintEdgeType, vertMajor1);
            Assert.IsTrue(vertMinor0 == vertMajor0 && vertMinor1 == vertMajor1 && vertMinor0 != null && vertMinor1 != null);

            InsertSegment(vertMinor0, vertMinor1, cid, newConstraintEdgeType, edgeMajor);

            // Vertex.EdgeEnumerator newMinorEdges0 = vertMinor0->GetEdgeEnumerator(false);
            // while (newMinorEdges0.MoveNext()) {
            //     UnityEngine.Debug.Log("==========================================================================");
            //     newMinorEdges0.Current->Org = vertMajor0;
            //     vertMajor0->AddEdge(newMinorEdges0.Current, false);
            // }

            // Vertex.EdgeEnumerator newMinorEdges1 = vertMinor1->GetEdgeEnumerator(false);
            // while (newMinorEdges1.MoveNext()) {
            //     UnityEngine.Debug.Log("==========================================================================");
            //     newMinorEdges1.Current->Org = vertMajor1;
            //     vertMajor1->AddEdge(newMinorEdges1.Current, false);
            // }
            // vertMinor0->HackyDisconnectMinorEdge();
            // vertMinor1->HackyDisconnectMinorEdge();

            // RemoveVertex(vertMinor0, true);
            // RemoveVertex(vertMinor1, true);

            // TODO: Deal with point constraints
        }


        // InsertMajor and InsertMinor
        internal void Insert(float2* points, int start, int amount, Entity cid, float4x4 ltw, Edge.Type newConstraintEdgeType = Edge.Type.Major | Edge.Type.Obstacle)
        {
            UnityEngine.Debug.Log($"Insert; {newConstraintEdgeType}");
            Vertex* lastVert = null;
            var end = start + amount;
            Vertex* point = null;

            for (var i = start; i < end; i++)
            {
                var c = Math.Mul2D(ltw, points[i]);
                Assert.IsTrue(_verticesSeq.Length < 5 || Contains(c), PointOutsideNavmeshMessage);
                var vert = InsertPoint(c, newConstraintEdgeType);
                Assert.IsTrue(vert != null);

                    if (amount == 2 && math.all(points[0] == points[1])) { // TODO: Delete all special stuff eventually
                        // UnityEngine.Debug.Log("Setting IsSpecial");
                        vert->IsSpecial = true;
                    }

                if (i == start)
                {
                    ++vert->ConstraintHandles;
                    _constraints[cid] = (IntPtr) vert;
                    point = vert;
                }

                if (lastVert != null && vert != lastVert)
                {
                    UnityEngine.Debug.Log("Insert - InsertSegment");
                    InsertSegment(lastVert, vert, cid, newConstraintEdgeType, null);
                    point = null;
                }
                lastVert = vert;
            }

            if (point != null)
                ++point->PointConstraints;
        }

        internal void RemoveConstraintMinor(Entity id)
        {
            RemoveConstraint(id, false, Edge.Type.Minor | Edge.Type.Terrain, Edge.Type.Minor | Edge.Type.TerrainSub);
        }

        internal void RemoveConstraintMajor(Entity id)
        {
            RemoveConstraint(id, true, Edge.Type.Major | Edge.Type.Obstacle, Edge.Type.Major | Edge.Type.Clearance);
        }

        void RemoveConstraint(Entity id, bool isMajor, Edge.Type constraintEdgeType, Edge.Type replacementEdgeType)
        {
            UnityEngine.Debug.Log("RemoveConstraint");
            _vlist.Clear();
            _elist.Clear();

            Assert.IsTrue(_constraints.ContainsKey(id), "Attempting to remove an unknown or static obstacle");
            var v = (Vertex*) _constraints[id];
            Assert.IsTrue(v->GetEdge(isMajor) != null); // TODO: true
            Assert.IsTrue(v->ConstraintHandles > 0);
            --v->ConstraintHandles;
            _constraints.Remove(id);
            var mark = NextMark;

            _open.Push(v);
            while (_open.Count > 0) // Depth first search for all (Quad)Edges of this constraint
            {
                var vert = _open.Pop();
                var i = vert->GetEdgeEnumerator(isMajor);
                while (i.MoveNext())
                {
                    if (i.Current->IsConstrainedBy(id) && i.Current->Mark != mark)
                    {
                        UnityEngine.Debug.Assert(i.Current->EdgeType.HasAllFlagsB(constraintEdgeType));
                        _elist.Add((IntPtr) i.Current);
                        i.Current->Mark = mark;
                        _open.Push(i.Current->Dest);
                    }
                }
            }

            if (_elist.Length == 0) // If no edges to this constraint, then this must be a point constraint
            {
                Assert.IsTrue(v->PointConstraints > 0);
                if (--v->PointConstraints == 0)
                    _vlist.Add((IntPtr) v);
            }
            else
            {
                for (var i = 0; i < _elist.Length; i++) // Add vertices of constraint edges to list for potential removal
                {
                    var e = (Edge*) _elist[i];
                    if (e->Org->Mark != mark && e->Org->PointConstraints == 0)
                    {
                        _vlist.Add((IntPtr) e->Org);
                        e->Org->Mark = mark;
                    }

                    if (e->Dest->Mark != mark && e->Dest->PointConstraints == 0)
                    {
                        _vlist.Add((IntPtr) e->Dest);
                        e->Dest->Mark = mark;
                    }
                }

                for (var i = 0; i < _elist.Length; i++) // Remove constraint Edges
                {
                    var edge = (Edge*) _elist[i];
                    UnityEngine.Debug.Assert(edge->EdgeType.HasAllFlagsB(constraintEdgeType));
                    edge->RemoveConstraint(id); // Only remove Entity Id of removed obstacle from edge's list

                    if (!edge->Constrained) // If no other constraints attached to this edge then it can be flipped to satisfy delaunay
                    {
                        V.TryAdd((IntPtr) edge->Org);
                        V.TryAdd((IntPtr) edge->Dest);
                        edge->RefineFailed = false;
                        ResetClearance(edge);
                        edge->SetEdgeType(replacementEdgeType);
                        _flipStack.Push(edge);
                        FlipQuad();
                    }
                }
            }

            for (var i = 0; i < _vlist.Length; i++)
                RemoveIfEligible((Vertex*) _vlist[i], isMajor, constraintEdgeType, replacementEdgeType);
        }

        void RemoveIfEligible(Vertex* v, bool isMajor, Edge.Type constraintEdgeType, Edge.Type replacementEdgeType)
        {
            UnityEngine.Debug.Log("RemoveIfEligible");
            if (v->PointConstraints > 0 || v->ConstraintHandles > 0) // If v is a point constraint or if v holds reference to a constraint then don't remove
                return;

            var amount = 0;
            var constrained = stackalloc Edge*[2];

            var e = v->GetEdgeEnumerator(isMajor); // Check if v connects to constraint edges
            while (e.MoveNext())
            {
                if (e.Current->Constrained)
                {
                    if (amount == 2) // If v connects to more than 2 constraints then we know we can't remove it so just return
                        return;
                    constrained[amount++] = e.Current;
                }
            }

            if (amount == 0) // If v not connected to any constraint edges, remove it, and fix the face affected
            {
                e = v->GetEdgeEnumerator(isMajor);
                while (e.MoveNext()) {
                    V.TryAdd((IntPtr) e.Current->Dest);
                }
                var face = RemoveVertex(v);
                RetriangulateFace(face, replacementEdgeType);
                return;
            }

            if (amount != 2 || !constrained[0]->ConstraintsEqual(constrained[1])) // If both constrained edges are the exact same (reference same obstacles) then continue
                return;

            var e1 = constrained[0];
            var e2 = constrained[1];
            UnityEngine.Debug.Assert(e1->EdgeType == e2->EdgeType);
            UnityEngine.Debug.Assert(e1->EdgeType == constraintEdgeType);
            Assert.IsTrue(e1->Dest != v && e2->Dest != v);
            Assert.IsTrue(e1->Dest != e2->Dest);
            var d1 = e1->Dest->Point;
            var d2 = e2->Dest->Point;
            var collinear = Math.TriArea(d1, d2, v->Point);

            if (collinear == 0) // If both constrained edges are exactly collinear, then remove the collinear vertex which connects them, and fix faces
            {                   // TODO: We remove only one of the 2 edges because the other edge becomes the sole edge? Or maybe because RemoveVertex deals with the other one anyways? Probably the first one
                e = v->GetEdgeEnumerator(isMajor);
                while (e.MoveNext())
                    V.TryAdd((IntPtr) e.Current->Dest);

                var v1 = e1->Dest;
                var v2 = e2->Dest;
                var crep = e1->QuadEdge->Crep;
                RemoveEdge(e1, false, isMajor);
                RemoveVertex(v);
                var e3 = Connect(v1, v2, constraintEdgeType, null); // TODO: null
                RetriangulateFace(e3, replacementEdgeType);
                RetriangulateFace(e3->Sym, replacementEdgeType);
                e3->QuadEdge->Crep = crep;
            }
            else // Then edges are not exactly collinear, but if they are almost (i.e. Semi) collinear then Remove v
            {
                var t = collinear / math.length(d2 - d1);

                if (collinear > 0)
                {
                    if (t < _collinearMargin && Math.TriArea(d1, d2, e1->DPrev->Org->Point) < 0 && Math.TriArea(d1, d2, e2->DNext->Org->Point) < 0)
                    {
                        e = v->GetEdgeEnumerator(isMajor);
                        while (e.MoveNext())
                            V.TryAdd((IntPtr) e.Current->Dest);
                        RemoveSemiCollinear(v, e1, e2, constraintEdgeType, replacementEdgeType);
                    }
                }
                else if (t > -_collinearMargin && Math.TriArea(d1, d2, e1->DNext->Org->Point) > 0 && Math.TriArea(d1, d2, e2->DPrev->Org->Point) > 0)
                {
                    e = v->GetEdgeEnumerator(isMajor);
                    while (e.MoveNext())
                        V.TryAdd((IntPtr) e.Current->Dest);
                    RemoveSemiCollinear(v, e1, e2, constraintEdgeType, replacementEdgeType);
                }
            }
        }

        void RemoveSemiCollinear(Vertex* v, Edge* e1, Edge* e2, Edge.Type constraintEdgeType, Edge.Type replacementEdgeType) // TODO: Needs parameter
        {
            UnityEngine.Debug.Log("RemoveSemiCollinear");
            var crep = GetCrep(e1->QuadEdge->Crep);
            var a = e1->Dest;
            var b = e2->Dest;
            e1->QuadEdge->Crep.Clear();
            e2->QuadEdge->Crep.Clear();
            _flipStack.Push(e1);
            _flipStack.Push(e2);
            FlipQuad(); // First satisfy the delaunay of collinear edges then remove the vertex and retriangulate
            var face1 = RemoveVertex(v);
            RetriangulateFace(face1, replacementEdgeType);
            InsertSegmentNoCrossConstraints(a, b, crep, constraintEdgeType, null); // TODO: null
        }

        void InsertSegmentNoCrossConstraints(Vertex* a, Vertex* b, UnsafeList<Entity> crep, Edge.Type newConstraintEdgeType, Edge* majorEdge)
        {
            UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints crep +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            var c = GetConnection(a, b, Edge.IsEdgeTypeMajor(newConstraintEdgeType));

            if (c != null)
            {
                C.TryAdd((IntPtr) c);
                c->QuadEdge->Crep = crep;
                ResetClearance(c);
                return;
            }

            var e = GetLeftEdge(a, b->Point, Edge.IsEdgeTypeMajor(newConstraintEdgeType));

            InfiniteLoopDetection.Reset();
            while (e->Dest != b)
            {
                InfiniteLoopDetection.Register(1000, "InsertSegmentNoCrossConstraints");

                var d = Math.TriArea(a->Point, b->Point, e->Dest->Point);
                var next = d > 0 ? e->RPrev : e->ONext;

                if (d < 0)
                {
                    Assert.IsTrue(!e->Constrained);
                    RemoveEdge(e, true, Edge.IsEdgeTypeMajor(newConstraintEdgeType));
                }
                else if (d == 0 && e->Dest != a)
                {
                    var t = e->Dest;
                    UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints crep - EdgeType Verify");
                    Connect(a, t, GetCrep(crep), newConstraintEdgeType, majorEdge); // TODO: Verify
                    a = t;
                }

                e = next;
            }

            UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints crep - EdgeType Verify");
            Connect(a, b, crep, newConstraintEdgeType, majorEdge); // TODO: Verify
        }

        void FlipQuad()
        {
            UnityEngine.Debug.Log("FlipQuad");
            while (_flipStack.Count > 0)
            {
                var edge = _flipStack.Pop();

                bool isSwap = false;
                if (Edge.IsEdgeTypeMajor(edge->EdgeType)) {
                    UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle), edge->Constrained));
                    isSwap = edge->EdgeType.HasNoFlagsB(Edge.Type.Obstacle); // If is a major Obstacle
                } else { // edge is Minor
                    UnityEngine.Debug.Assert(edge->EdgeType.HasAnyFlagsB(Edge.Type.Minor));
                    UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(edge->EdgeType.HasAnyFlagsB(Edge.Type.Terrain), edge->Constrained));
                    isSwap = edge->EdgeType.HasNoFlagsB(Edge.Type.Terrain | Edge.Type.Obstacle | Edge.Type.Clearance);
                }

                // TODO: Change!
                if (/* !edge->Constrained */ isSwap && Math.CircumcircleContains(edge->Org->Point, edge->Dest->Point, edge->ONext->Dest->Point, edge->DNext->Org->Point))
                {
                    _flipStack.Push(edge->OPrev);
                    _flipStack.Push(edge->DNext);
                    _flipStack.Push(edge->Sym->OPrev);
                    _flipStack.Push(edge->Sym->DNext);
                    Swap(edge);
                }
            }
        }

        Edge* CreateEdge(Vertex* a, Vertex* b, Edge.Type newEdgeType, Edge* majorEdge)
        {
            UnityEngine.Debug.Log("CreateEdge");
            var q = _quadEdges.GetElementPointer(new QuadEdge {Crep = GetCrep(), Id = NextEdgeId, EdgeType = newEdgeType});

            q->MajorEdge = majorEdge;

            q->Edge0 = new Edge(q, 0);
            q->Edge1 = new Edge(q, 1);
            q->Edge2 = new Edge(q, 2);
            q->Edge3 = new Edge(q, 3);

            q->Edge0.Next = &q->Edge0;
            q->Edge1.Next = &q->Edge3;
            q->Edge2.Next = &q->Edge2;
            q->Edge3.Next = &q->Edge1;

            if (Edge.IsEdgeTypeMajor(newEdgeType)) {
                _modifiedMajorEdges.TryAdd((IntPtr)(&q->Edge0));
            }

            SetEndPoints(&q->Edge0, a, b, Edge.IsEdgeTypeMajor(newEdgeType));
            return &q->Edge0;
        }

        void RemoveEdge(Edge* e, bool recycleCrep, bool isMajor)
        {
            UnityEngine.Debug.Log("RemoveEdge");
            DestroyedTriangle(e->TriangleId); // Add both triangles to Destroyed list so that we can later check to see if a previously created path needs to be invalidated
            DestroyedTriangle(e->Sym->TriangleId);

            Edge.VerifyEdgeType(e->EdgeType, isMajor);
            // bool debusMajor = e->EdgeType == EdgeType.Obstacle || e->EdgeType == EdgeType.ConnectsToObstacle || e->EdgeType == EdgeType.ConnectsToObstacleWithMinorTerrains;

            e->Org->RemoveEdge(e, isMajor); // If vertex points to edge as head, then remove it
            e->Dest->RemoveEdge(e->Sym, isMajor);
            Splice(e, e->OPrev); // TODO: I think this is merging edges
            Splice(e->Sym, e->Sym->OPrev);



            /* UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(isMajor, e->Org->GetEdge(false) != null && e->Dest->GetEdge(false) != null), "Weird");
            if (isMajor && e->Org->GetEdge(false) != null && e->Dest->GetEdge(false) != null) {
                UnityEngine.Debug.Assert(e->Org->ContainsEdge(e, false) && e->Dest->ContainsEdge(e, false), "Also weird");

                Vertex.EdgeEnumerator orgEdgeEnumerator = e->Org->GetEdgeEnumerator(isMajor);
                while (orgEdgeEnumerator.MoveNext()) {
                    if (orgEdgeEnumerator.Current->MajorEdge == e || orgEdgeEnumerator.Current->MajorEdge == e->Sym) {
                        return true;
                    }
                }
                foreach (Edge* edge in e->Org->GetEdgeEnumerator(false)) {

                }
                // RemoveConstraint();
            } */



            var qe = e->QuadEdge; // Remove the associated QuadEdge and maybe recycle its crep
            if (recycleCrep)
            {
                qe->Crep.Clear();
                _creps.Push(qe->Crep);
            }

            _quadEdges.Recycle(qe);
        }

        void DestroyedTriangle(int tri)
        {
            DestroyedTriangles.TryAdd(tri);
        }

        Vertex* CreateVertex(float2 p)
        {
            // CommonLib.CreatePrimitive(UnityEngine.PrimitiveType.Sphere, p.XOY(), new float3(0.01f), UnityEngine.Color.red, default, 5f);
            UnityEngine.Debug.Log("CreateVertex");
            var v = _vertices.GetElementPointer(new Vertex {Point = p, SeqPos = _verticesSeq.Length});
            _verticesSeq.Add((IntPtr) v);
            _qt.Insert(v);
            return v;
        }

        /// <summary>
        /// Create a new Edge connecting the destination of a to the origin of b,
        /// such that all three edges have the same left face after the connection is complete.
        /// </summary>
        Edge* Connect(Edge* a, Edge* b, Edge.Type newEdgeType, Edge* majorEdge)
        {
            Assert.IsTrue(GetConnection(a->Dest, b->Org, Edge.IsEdgeTypeMajor(newEdgeType)) == null);
            var result = CreateEdge(a->Dest, b->Org, newEdgeType, majorEdge);
            Splice(result, a->LNext);
            Splice(result->Sym, b);
            return result;
        }

        // This operator affects the two edge rings around the origins of a and b, and, independently, the two edge
        // rings around the left faces of a and b. In each case, (i) if the two rings are distinct, Splice will combine
        // them into one; (ii) if the two are the same ring, Splice will break it into two separate pieces.
        // Thus, Splice can be used both to attach the two edges together and to break them apart.
        // Guibus and Stolfi (1985 p.96)
        static void Splice(Edge* a, Edge* b)
        {
            var alpha = a->ONext->Rot;
            var beta = b->ONext->Rot;
            var temp1 = b->ONext;
            var temp2 = a->ONext;
            var temp3 = beta->ONext;
            var temp4 = alpha->ONext;
            a->Next = temp1;
            b->Next = temp2;
            alpha->Next = temp3;
            beta->Next = temp4;
        }

        void NewTriangle(Edge* e) // Just inits TriangleIds
        {
            var tid = NextTriangleId;
            e->TriangleId = tid;
            e->LNext->TriangleId = tid;
            e->LPrev->TriangleId = tid;
        }
    }
}
using System.Diagnostics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using DotsNav.Core;
using System.Drawing;

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

        Edge* Connect(Vertex* a, Vertex* b, EdgeType newEdgeType) // Connect two vertices with an edge
        {
            bool isMajor = Vertex.IsEdgeTypeMajor(newEdgeType);
            Assert.IsTrue(a->GetEdge(isMajor) != null);
            Assert.IsTrue(b->GetEdge(isMajor) != null);
            return Connect(GetLeftEdge(a, b->Point, isMajor)->Sym, GetLeftEdge(b, a->Point, isMajor)->OPrev, newEdgeType);
        }

        // Flips edge e counterclockwise inside its enclosing quadrilateral.
        // http://karlchenofhell.org/cppswp/lischinski.pdf
        void Swap(Edge* e)
        {
            UnityEngine.Debug.Log("Swap");
            //Assert.IsTrue(!e->Constrained && e->EdgeType != EdgeType.Obstacle && e->EdgeType != EdgeType.Terrain, "Cannot flip a constrained edge");
            UnityEngine.Debug.Assert(!e->Constrained && e->EdgeType != EdgeType.Obstacle && e->EdgeType != EdgeType.Terrain, $"Cannot flip a constrained edge. EdgeType: {e->EdgeType}, e->Constrained: {e->Constrained}");
            
            bool isMajor = Vertex.IsEdgeTypeMajor(e->EdgeType);
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
        void FlipEdges(float2 p, ConstraintType constraintType) // Calls Swap
        {
            UnityEngine.Debug.Log($"FlipEdges: {constraintType}");
            while (_flipStack.Count > 0)
            {
                var e = _flipStack.Pop();
                Assert.IsTrue(Math.Ccw(e->Org->Point, e->Dest->Point, p));

                bool isSwap = false; // TODO: Could potentially get rid of all of this: (But also don't allow flipping a Terrain ConnectsToObstacle Minor Edge)
                if (constraintType == ConstraintType.Obstacle && !e->Constrained) {
                    UnityEngine.Debug.Assert(e->EdgeType == EdgeType.Obstacle || e->EdgeType == EdgeType.ConnectsToObstacle, $"EdgeType: {e->EdgeType}");
                    isSwap = true;
                } else if (constraintType == ConstraintType.Terrain && e->EdgeType == EdgeType.ConnectsToTerrain) {
                    UnityEngine.Debug.Assert(!e->Constrained, "Obvious, can delete this check");
                    isSwap = true;
                }

                if (isSwap && Math.CircumcircleContains(e->Org->Point, e->Dest->Point, p, e->DNext->Org->Point))
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
        void RetriangulateFace(Edge* edge)
        {
            UnityEngine.Debug.Log("RetriangulateFace");
            Assert.IsTrue(edge != null);
            Assert.IsTrue(edge != edge->LNext->LNext);
            if (edge->LNext->LNext->LNext == edge)
            {
                NewTriangle(edge); // Just inits TriangleIds
                return;
            }

            InfiniteLoopDetection.Reset(); // Debug detection of infinite loop
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

            Assert.IsTrue(c != edge);
            var connected = false;
            if (c->LNext->LNext != edge)
            {
                V.TryAdd((IntPtr) edge->LPrev->Dest);
                V.TryAdd((IntPtr) c->LNext->Org);

                UnityEngine.Debug.Log("RetriangulateFace - EdgeType.None");
                var b = Connect(edge->LPrev, c->LNext, EdgeType.None);
                RetriangulateFace(b);
                connected = true;
            }

            if (c != edge->LNext)
            {
                V.TryAdd((IntPtr) c->Dest);
                V.TryAdd((IntPtr) edge->LNext->Org);

                UnityEngine.Debug.Log("RetriangulateFace - EdgeType.None");
                var a = Connect(c, edge->LNext, EdgeType.None);
                RetriangulateFace(a);
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
                var e = vert->GetEdge(true); // TODO: True
                if (e != null) {
                    RemoveEdge(e, true, true);
                }
                var minorE = vert->GetEdge(false);
                if (minorE != null) {
                    RemoveEdge(e, true, false);
                }
                if (e == null && minorE == null) {
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

        Vertex* InsertPoint(float2 p, ConstraintType constraintType, Vertex* existingVertex = null)
        {
            UnityEngine.Debug.Log($"InsertPoint; {constraintType}");
            Vertex.Type vertexType = constraintType == ConstraintType.MajorEdgeInsertedToMinorGraph ? Vertex.Type.Minor : Vertex.Type.Major;
            var closest = _qt.FindClosest(p, vertexType);

            if (constraintType == ConstraintType.Obstacle && closest->GetEdge(true) == null) {
                Edge* minorEdge = closest->GetEdge(false);
                Assert.IsTrue(minorEdge != null, "If major edge doesn't exist, there must be a minor edge");
                //Edge* minorEdgesMajorEdge = minorEdge->MajorEdge;
                // TODO: Unfinished
            }

            if (math.lengthsq(closest->Point - p) <= _e * _e) {
                UnityEngine.Debug.Log("InsertPoint - Found vertex, returning it instead of inserting new");
                return closest;
            }

            ConstraintType betterConstraintType = constraintType == ConstraintType.MajorEdgeInsertedToMinorGraph ? ConstraintType.Terrain : constraintType;
            var e = closest->GetEdge(Vertex.IsEdgeTypeMajor((EdgeType)betterConstraintType));
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
                    return InsertPointInEdge(p, inEdge, betterConstraintType, existingVertex);
                }

                return InsertPointInFace(p, e, betterConstraintType, existingVertex);
            }
        }

        Vertex* InsertPointInEdge(float2 point, Edge* edge, ConstraintType constraintType, Vertex* existingVertex = null)
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


            UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(constraintType == ConstraintType.Obstacle, edge->EdgeType == EdgeType.Obstacle));
            UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(constraintType == ConstraintType.Terrain, edge->EdgeType == EdgeType.Terrain || edge->EdgeType == EdgeType.ConnectsToObstacle));

            EdgeType newEdgesType = EdgeType.None;
            QuadEdge* newMajorEdge = null;
            if (constraintType == ConstraintType.Terrain && edge->EdgeType == EdgeType.ConnectsToObstacle) {
                newMajorEdge = edge->QuadEdge; // Don't remove edge, make it the majorEdge of new subEdges
                edge->SetEdgeType(EdgeType.ConnectsToObstacleWithMinorTerrainConnects);
                newEdgesType = EdgeType.ConnectsToTerrainWithMajorConnectsToObstacle;
                
                // edge->Org->RemoveEdge(edge, ); // If vertex points to edge as head, then remove it
                // edge->Dest->RemoveEdge(edge->Sym, );

                //RemoveEdge(edge, false, false); // Debug
            }
            else if (constraintType == ConstraintType.Terrain && edge->EdgeType == EdgeType.ConnectsToTerrainWithMajorConnectsToObstacle) {
                newMajorEdge = edge->MajorEdge;
                newEdgesType = EdgeType.ConnectsToTerrainWithMajorConnectsToObstacle;
                RemoveEdge(edge, false, false);
            }
            else { // if (constraintType == ConstraintType.Obstacle && edge->EdgeType == EdgeType.Obstacle)
                RemoveEdge(edge, false, true);
                newEdgesType = edge->EdgeType;
            }


            var result = existingVertex != null ? existingVertex : CreateVertex(point);
            V.TryAdd((IntPtr) result);
            V.TryAdd((IntPtr) e->Org);
            var newEdge = CreateEdge(e->Org, result, newEdgesType);
            newEdge->QuadEdge->Crep = GetCrep(crep);
            Splice(newEdge, e);

            V.TryAdd((IntPtr) e->Dest);
            V.TryAdd((IntPtr) newEdge->Sym->Org);

            newEdge = Connect(e, newEdge->Sym, newEdgesType);
            e = newEdge->OPrev;

            V.TryAdd((IntPtr) e->Dest);
            V.TryAdd((IntPtr) newEdge->Sym->Org);

            newEdge = Connect(e, newEdge->Sym, newEdgesType);
            newEdge->QuadEdge->Crep = crep;
            e = newEdge->OPrev;

            V.TryAdd((IntPtr) e->Dest);
            V.TryAdd((IntPtr) newEdge->Sym->Org);

            Connect(e, newEdge->Sym, newEdgesType);

            var te = result->GetEdge(Vertex.IsEdgeTypeMajor((EdgeType)constraintType));
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);

            FlipEdges(point, constraintType);
            return result;
        }

        UnsafeList<Entity> GetCrep(UnsafeList<Entity> source)
        {
            var l = GetCrep();
            l.AddRange(source);
            return l;
        }

        UnsafeList<Entity> GetCrep() => _creps.Count > 0 ? _creps.Pop() : new UnsafeList<Entity>(CrepMinCapacity, Allocator.Persistent);

        Vertex* InsertPointInFace(float2 p, Edge* edge, ConstraintType constraintType, Vertex* existingVertex = null) // TODO: Perhaps make separate function to avoid branch eventually
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

            EdgeType newEdgesType = EdgeType.None;
            if (constraintType == ConstraintType.Terrain) {
                newEdgesType = EdgeType.ConnectsToTerrain;
            } else if (constraintType == ConstraintType.Obstacle) {
                newEdgesType = EdgeType.ConnectsToObstacle;
            }

            var newEdge = CreateEdge(edge->Org, result, newEdgesType);
            Splice(newEdge, edge);
            newEdge = Connect(edge, newEdge->Sym, newEdgesType);
            Connect(newEdge->OPrev, newEdge->Sym, newEdgesType);

            var te = result->GetEdge(Vertex.IsEdgeTypeMajor((EdgeType)constraintType));
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);
            te = te->ONext;
            NewTriangle(te);

            FlipEdges(p, constraintType);
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

        void InsertSegment(Vertex* a, Vertex* b, Entity id, ConstraintType constraintType)
        {
            UnityEngine.Debug.Log($"InsertSegment; {constraintType}");
            var dir = math.normalize(b->Point - a->Point);
            InsertSegmentRecursive(a, b, id, dir, a->Point, b->Point, constraintType);
        }

        void InsertSegmentRecursive(Vertex* a, Vertex* b, Entity id, float2 dir, float2 start, float2 end, ConstraintType constraintType)
        {
            UnityEngine.Debug.Log($"InsertSegmentRecursive; {constraintType}");
            _insertedPoints.Clear();
            _insertedPoints.Add(new Point {Vertex = a, P = a->Point});

            while (a != b)
            {
                var p0 = _insertedPoints[^1];
                var p1 = GetNextPoint(a, b, start, end, constraintType);

                if (!p0.Modified && !p1.Modified)
                {
                    if (p0.FoundExisting || p1.FoundExisting)
                        InsertSegmentRecursive(p0.Vertex, p1.Vertex, id, dir, start, end, constraintType);
                    else
                        InsertSegmentNoConstraints(p0.Vertex, p1.Vertex, id, constraintType);
                }
                else if (p0.Modified && !p1.Modified)
                {
                    if (GetSupport(p0.After + _e / 2 * dir, p1.P - _e * dir, dir, out var p))
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - p0 modified, insert point");
                        var after = InsertPoint(p, constraintType);
                        InsertSegmentRecursive(after, p1.Vertex, id, dir, start, end, constraintType);
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
                        var before = InsertPoint(p, constraintType);
                        InsertSegmentRecursive(p0.Vertex, before, id, dir, start, end, constraintType);
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
                        var v0 = InsertPoint(s1, constraintType);
                        var v1 = InsertPoint(s2, constraintType);
                        InsertSegmentRecursive(v0, v1, id, dir, start, end, constraintType);
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

        Point GetNextPoint(Vertex* a, Vertex* b, float2 start, float2 end, ConstraintType constraintType)
        {
            UnityEngine.Debug.Log($"GetNextPoint; {constraintType}");
            InfiniteLoopDetection.Reset();

            // bool isMajor = false;
            // if (constraintType == ConstraintType.MajorEdgeInsertedToMinorGraph) {
            //     isMajor = true;
            // } else {
            //     isMajor = Vertex.IsEdgeTypeMajor((EdgeType)constraintType);
            // }
            var e = GetLeftEdge(a, b->Point, Vertex.IsEdgeTypeMajor((EdgeType)constraintType));
            while (e->Dest != b)
            {
                InfiniteLoopDetection.Register(1000, "GetNextPoint");

                var d = Math.TriArea(a->Point, b->Point, e->Dest->Point);

                // if (d < 0 && e->Constrained)
                if (d < 0 && MathLib.IfFirstCheckSecond(constraintType == ConstraintType.Obstacle, e->Constrained))
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
                        var pRef = CreatePRef(p, e, constraintType);

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

                    var vert = InsertPointInEdge(p, e, constraintType);
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

        Vertex* CreatePRef(float2 p, Edge* e, ConstraintType constraintType)
        {
            UnityEngine.Debug.Log($"CreatePRef; {constraintType}");
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
                    return InsertPointInEdge(pplus, e, constraintType);

                var pmin = p - offset * dir;
                pointPresent = TryGetPoint(pmin, e, out vertex);

                if (vertex != null)
                    return vertex;

                if (!pointPresent && SplitIsRobust(pmin, e))
                    return InsertPointInEdge(pmin, e, constraintType);
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

        void InsertSegmentNoConstraints(Vertex* a, Vertex* b, Entity id, ConstraintType constraintType)
        {
            UnityEngine.Debug.Log($"InsertSegmentNoConstraints; {constraintType}");
            var c = GetConnection(a, b, constraintType == ConstraintType.Obstacle);

            if (c != null)
            {
                C.TryAdd((IntPtr) c);
                if (!c->IsConstrainedBy(id))
                    c->AddConstraint(id);
                ResetClearance(c);
                c->SetEdgeType((EdgeType)constraintType);
                UnityEngine.Debug.Log($"InsertSegmentNoConstraints - Found edge, setting it to EdgeType: {(EdgeType)constraintType}");
                return;
            }

            var e = GetLeftEdge(a, b->Point, Vertex.IsEdgeTypeMajor((EdgeType)constraintType));

            InfiniteLoopDetection.Reset();
            while (e->Dest != b)
            {
                InfiniteLoopDetection.Register(1000, "InsertSegmentNoConstraints");

                var d = Math.TriArea(a->Point, b->Point, e->Dest->Point);
                var next = d > 0 ? e->RPrev : e->ONext;

                if (d < 0)
                {
                    UnityEngine.Debug.Log("InsertSegmentNoConstraints - Weird thing -> Removing edge");
                    Assert.IsTrue(!e->Constrained);
                    RemoveEdge(e, true, constraintType == ConstraintType.Obstacle);
                }
                else if (d == 0 && e->Dest != a)
                {
                    var t = e->Dest;
                    UnityEngine.Debug.Log("InsertSegmentNoConstraints - Connect");
                    Connect(a, t, id, (EdgeType)constraintType);
                    a = t;
                }

                e = next;
            }

            UnityEngine.Debug.Log("InsertSegmentNoConstraints - Connect 2");
            Connect(a, b, id, (EdgeType)constraintType);
        }

        void Connect(Vertex* a, Vertex* b, Entity id, EdgeType newEdgeType)
        {
            var connection = GetConnection(a, b, Vertex.IsEdgeTypeMajor(newEdgeType));
            if (connection == null)
            {
                UnityEngine.Debug.Log($"Connect - null connection, so creating one with EdgeType: {newEdgeType}");
                V.TryAdd((IntPtr) a);
                V.TryAdd((IntPtr) b);

                connection = Connect(a, b, newEdgeType);
                RetriangulateFace(connection);
                RetriangulateFace(connection->Sym);
            }

            // todo inline wasUnconstrained (so if moves above addconstraint)
            var wasUnconstrained = !connection->Constrained;
            connection->AddConstraint(id);
            if (wasUnconstrained)
                ResetClearance(connection);
            connection->SetEdgeType(newEdgeType);
            C.TryAdd((IntPtr) connection);
        }

        void Connect(Vertex* a, Vertex* b, UnsafeList<Entity> crep, EdgeType newEdgeType)
        {
            var connection = GetConnection(a, b, Vertex.IsEdgeTypeMajor(newEdgeType));
            if (connection == null)
            {
                V.TryAdd((IntPtr) a);
                V.TryAdd((IntPtr) b);

                connection = Connect(a, b, newEdgeType);
                RetriangulateFace(connection);
                RetriangulateFace(connection->Sym);
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
                var te = closest->GetEdge(Vertex.IsEdgeTypeMajor(e->EdgeType));
                do
                {
                    if (te->QuadEdge == e->QuadEdge)
                    {
                        v = closest;
                        break;
                    }

                    te = te->ONext;
                } while (te != closest->GetEdge(Vertex.IsEdgeTypeMajor(e->EdgeType)));

                return true;
            }

            return false;
        }

        static readonly FixedString128Bytes PointOutsideNavmeshMessage = "Trying to add a point outside the navmesh";


        internal void InsertMajorIntoMinor(Vertex* vertMajor0, Vertex* vertMajor1, Entity cid, ConstraintType constraintType = ConstraintType.MajorEdgeInsertedToMinorGraph)
        {
            UnityEngine.Debug.Log($"InsertMajorIntoMinor; {constraintType}");
            Assert.IsTrue(vertMajor0 != null); Assert.IsTrue(vertMajor1 != null);

            var vertMinor0 = InsertPoint(vertMajor0->Point, constraintType, vertMajor0);
            var vertMinor1 = InsertPoint(vertMajor0->Point, constraintType, vertMajor1);
            Assert.IsTrue(vertMinor0 != null && vertMinor1 != null);

            InsertSegment(vertMinor0, vertMinor1, cid, ConstraintType.Terrain);

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


        internal void Insert(float2* points, int start, int amount, Entity cid, float4x4 ltw, ConstraintType constraintType = ConstraintType.Obstacle)
        {
            UnityEngine.Debug.Log($"Insert; {constraintType}");
            Vertex* lastVert = null;
            var end = start + amount;
            Vertex* point = null;

            for (var i = start; i < end; i++)
            {
                var c = Math.Mul2D(ltw, points[i]);
                Assert.IsTrue(_verticesSeq.Length < 5 || Contains(c), PointOutsideNavmeshMessage);
                var vert = InsertPoint(c, constraintType);
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
                    InsertSegment(lastVert, vert, cid, constraintType);
                    point = null;
                }
                lastVert = vert;
            }

            if (point != null)
                ++point->PointConstraints;
        }

        internal void RemoveConstraint(Entity id)
        {
            UnityEngine.Debug.Log("RemoveConstraint");
            _vlist.Clear();
            _elist.Clear();

            Assert.IsTrue(_constraints.ContainsKey(id), "Attempting to remove an unknown or static obstacle");
            var v = (Vertex*) _constraints[id];
            Assert.IsTrue(v->GetEdge(true) != null); // TODO: true
            Assert.IsTrue(v->ConstraintHandles > 0);
            --v->ConstraintHandles;
            _constraints.Remove(id);
            var mark = NextMark;

            _open.Push(v);
            while (_open.Count > 0) // Depth first search for all (Quad)Edges of this constraint
            {
                var vert = _open.Pop();
                var i = vert->GetEdgeEnumerator(true);
                while (i.MoveNext())
                {
                    if (i.Current->IsConstrainedBy(id) && i.Current->Mark != mark)
                    {
                        _elist.Add((IntPtr) i.Current);
                        i.Current->Mark = mark;
                        _open.Push(i.Current->Dest);
                    }
                }
            }

            if (_elist.Length == 0)
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
                    edge->RemoveConstraint(id); // Only remove Entity Id of constraint from edge's list

                    if (!edge->Constrained) // If no other constraints attached to this edge then it can be flipped to satisfy delaunay
                    {
                        V.TryAdd((IntPtr) edge->Org);
                        V.TryAdd((IntPtr) edge->Dest);
                        edge->RefineFailed = false;
                        ResetClearance(edge);
                        _flipStack.Push(edge);
                        FlipQuad();
                    }
                }
            }

            for (var i = 0; i < _vlist.Length; i++)
                RemoveIfEligible((Vertex*) _vlist[i]);
        }

        void RemoveIfEligible(Vertex* v)
        {
            UnityEngine.Debug.Log("RemoveIfEligible");
            if (v->PointConstraints > 0 || v->ConstraintHandles > 0) // If v holds reference to a constraint then don't remove
                return;

            var amount = 0;
            var constrained = stackalloc Edge*[2];

            var e = v->GetEdgeEnumerator(true); // Check if v connects to constraint edges
            while (e.MoveNext())
            {
                if (e.Current->Constrained)
                {
                    if (amount == 2)
                        return;
                    constrained[amount++] = e.Current;
                }
            }

            if (amount == 0) // If v not connected to any constraint edges, remove it, and fix the face affected
            {
                e = v->GetEdgeEnumerator(true);
                while (e.MoveNext())
                    V.TryAdd((IntPtr) e.Current->Dest);
                var face = RemoveVertex(v);
                RetriangulateFace(face);
                return;
            }

            if (amount != 2 || !constrained[0]->ConstraintsEqual(constrained[1])) // If both constrained edges are the exact same (reference same obstacles) then continue
                return;

            var e1 = constrained[0];
            var e2 = constrained[1];
            Assert.IsTrue(e1->Dest != v && e2->Dest != v);
            Assert.IsTrue(e1->Dest != e2->Dest);
            var d1 = e1->Dest->Point;
            var d2 = e2->Dest->Point;
            var collinear = Math.TriArea(d1, d2, v->Point);

            if (collinear == 0) // If both constrained edges are exactly collinear, then remove the collinear vertex which connects them, and fix faces
            {                   // TODO: We remove only one of the 2 edges because the other edge becomes the sole edge? Or maybe because RemoveVertex deals with the oter one anyways?
                e = v->GetEdgeEnumerator(true);
                while (e.MoveNext())
                    V.TryAdd((IntPtr) e.Current->Dest);

                var v1 = e1->Dest;
                var v2 = e2->Dest;
                var crep = e1->QuadEdge->Crep;
                RemoveEdge(e1, false, true);
                RemoveVertex(v);
                UnityEngine.Debug.Log("RemoveIfEligible - EdgeType.None"); // TODO: Assert both edges are the same constraint type and pass it
                var e3 = Connect(v1, v2, EdgeType.None);
                RetriangulateFace(e3);
                RetriangulateFace(e3->Sym);
                e3->QuadEdge->Crep = crep;
            }
            else // Then edges are not exactly collinear, but if they are almost (i.e. Semi) collinear then Remove v
            {
                var t = collinear / math.length(d2 - d1);

                if (collinear > 0)
                {
                    if (t < _collinearMargin && Math.TriArea(d1, d2, e1->DPrev->Org->Point) < 0 && Math.TriArea(d1, d2, e2->DNext->Org->Point) < 0)
                    {
                        e = v->GetEdgeEnumerator(true);
                        while (e.MoveNext())
                            V.TryAdd((IntPtr) e.Current->Dest);
                        RemoveSemiCollinear(v, e1, e2);
                    }
                }
                else if (t > -_collinearMargin && Math.TriArea(d1, d2, e1->DNext->Org->Point) > 0 && Math.TriArea(d1, d2, e2->DPrev->Org->Point) > 0)
                {
                    e = v->GetEdgeEnumerator(true);
                    while (e.MoveNext())
                        V.TryAdd((IntPtr) e.Current->Dest);
                    RemoveSemiCollinear(v, e1, e2);
                }
            }
        }

        void RemoveSemiCollinear(Vertex* v, Edge* e1, Edge* e2)
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
            RetriangulateFace(face1);
            UnityEngine.Debug.Log("RemoveSemiCollinear - ConstraintType.None");
            InsertSegmentNoConstraints(a, b, crep, ConstraintType.None); // TODO: Maybe pass
        }

        void InsertSegmentNoConstraints(Vertex* a, Vertex* b, UnsafeList<Entity> crep, ConstraintType constraintType)
        {
            UnityEngine.Debug.Log("InsertSegmentNoConstraints crep");
            var c = GetConnection(a, b, constraintType == ConstraintType.Obstacle);

            if (c != null)
            {
                C.TryAdd((IntPtr) c);
                c->QuadEdge->Crep = crep;
                ResetClearance(c);
                return;
            }

            var e = GetLeftEdge(a, b->Point, true); // TODO: True

            InfiniteLoopDetection.Reset();
            while (e->Dest != b)
            {
                InfiniteLoopDetection.Register(1000, "InsertSegmentNoConstraints");

                var d = Math.TriArea(a->Point, b->Point, e->Dest->Point);
                var next = d > 0 ? e->RPrev : e->ONext;

                if (d < 0)
                {
                    Assert.IsTrue(!e->Constrained);
                    RemoveEdge(e, true, constraintType == ConstraintType.Obstacle);
                }
                else if (d == 0 && e->Dest != a)
                {
                    var t = e->Dest;
                    UnityEngine.Debug.Log("InsertSegmentNoConstraints crep - EdgeType.None");
                    Connect(a, t, GetCrep(crep), EdgeType.None);
                    a = t;
                }

                e = next;
            }

            UnityEngine.Debug.Log("InsertSegmentNoConstraints crep - EdgeType.None");
            Connect(a, b, crep, EdgeType.None);
        }

        void FlipQuad()
        {
            UnityEngine.Debug.Log("FlipQuad");
            while (_flipStack.Count > 0)
            {
                var edge = _flipStack.Pop();

                if (!edge->Constrained && Math.CircumcircleContains(edge->Org->Point, edge->Dest->Point, edge->ONext->Dest->Point, edge->DNext->Org->Point))
                {
                    _flipStack.Push(edge->OPrev);
                    _flipStack.Push(edge->DNext);
                    _flipStack.Push(edge->Sym->OPrev);
                    _flipStack.Push(edge->Sym->DNext);
                    Swap(edge);
                }
            }
        }

        Edge* CreateEdge(Vertex* a, Vertex* b, EdgeType newEdgeType)
        {
            UnityEngine.Debug.Log("CreateEdge");
            var q = _quadEdges.GetElementPointer(new QuadEdge {Crep = GetCrep(), Id = NextEdgeId, EdgeType = newEdgeType});

            q->Edge0 = new Edge(q, 0);
            q->Edge1 = new Edge(q, 1);
            q->Edge2 = new Edge(q, 2);
            q->Edge3 = new Edge(q, 3);

            q->Edge0.Next = &q->Edge0;
            q->Edge1.Next = &q->Edge3;
            q->Edge2.Next = &q->Edge2;
            q->Edge3.Next = &q->Edge1;

            if (newEdgeType == EdgeType.Obstacle || newEdgeType == EdgeType.ConnectsToObstacle) {
                _modifiedMajorEdges.TryAdd((IntPtr)(&q->Edge0));
            }

            SetEndPoints(&q->Edge0, a, b, Vertex.IsEdgeTypeMajor(newEdgeType));
            return &q->Edge0;
        }

        void RemoveEdge(Edge* e, bool recycleCrep, bool isMajor)
        {
            UnityEngine.Debug.Log("RemoveEdge");
            DestroyedTriangle(e->TriangleId); // Add both triangles to Destroyed list so that we can later check to see if a previously created path needs to be invalidated
            DestroyedTriangle(e->Sym->TriangleId);

            Vertex.VerifyEdgeType(e, isMajor);
            // bool debusMajor = e->EdgeType == EdgeType.Obstacle || e->EdgeType == EdgeType.ConnectsToObstacle || e->EdgeType == EdgeType.ConnectsToObstacleWithMinorTerrains;

            e->Org->RemoveEdge(e, isMajor); // If vertex points to edge as head, then remove it
            e->Dest->RemoveEdge(e->Sym, isMajor);
            Splice(e, e->OPrev); // TODO: I think this is merging edges
            Splice(e->Sym, e->Sym->OPrev);

            var qe = e->QuadEdge; // Remove the associated QuadEdge
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
        Edge* Connect(Edge* a, Edge* b, EdgeType newEdgeType)
        {
            Assert.IsTrue(GetConnection(a->Dest, b->Org, Vertex.IsEdgeTypeMajor(newEdgeType)) == null);
            var result = CreateEdge(a->Dest, b->Org, newEdgeType);
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
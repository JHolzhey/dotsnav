using System.Diagnostics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using DotsNav.Core;
using System.Drawing;
using Unity.Entities.UniversalDelegates;
using System.Linq.Expressions;
using DotsNav.Drawing;
using System;
using System.Runtime.CompilerServices;

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
            UnityEngine.Debug.Assert(!e->IsConstrained && e->EdgeType.HasNoFlagsB(Edge.Type.Obstacle | Edge.Type.Terrain) && !e->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance),
                $"Cannot flip a constrained edge. EdgeType: {e->EdgeType}, e->Constrained: {e->IsConstrained}");
            UnityEngine.Debug.Assert(e->MaterialTypeIndex == e->Sym->MaterialTypeIndex, "Material Types are different!");

            UnityEngine.Debug.Assert(e->MaterialTypeIndex == e->LNext->MaterialTypeIndex && e->LNext->MaterialTypeIndex == e->LPrev->MaterialTypeIndex, "edge Tri has unequal types");
            UnityEngine.Debug.Assert(e->Sym->MaterialTypeIndex == e->Sym->LNext->MaterialTypeIndex && e->Sym->LNext->MaterialTypeIndex == e->Sym->LPrev->MaterialTypeIndex, "edge->Sym Tri has unequal types");

            bool isMajor = Edge.IsEdgeTypeMajor(e->EdgeType);
            Edge.VerifyEdge(e, isMajor);

            if (isMajor) {
                RemoveMajorInMinor(e);
                UnityEngine.Debug.Assert(!AddedOrModifiedMajorEdges.Contains((IntPtr)e->Rot) && !AddedOrModifiedMajorEdges.Contains((IntPtr)e->InvRot));
                if (!AddedOrModifiedMajorEdges.Contains((IntPtr)e->Sym)) {
                    AddedOrModifiedMajorEdges.TryAdd((IntPtr)e);
                }
            }

            e->Org->RemoveEdge(e, isMajor);
            e->Dest->RemoveEdge(e->Sym, isMajor);

            if (isMajor) {
                V.TryAdd((IntPtr) e->Org);
                V.TryAdd((IntPtr) e->Dest);
            }

            var a = e->OPrev;
            var b = e->Sym->OPrev;
            Splice(e, a); // These two cuts ties with original Org and Dest vertices and corresponding edges
            Splice(e->Sym, b);

            Splice(e, a->LNext); // These two merge with the new Org and Dest which are the opposite counterclockwise vertices in the quad
            Splice(e->Sym, b->LNext);
            
            SetEndPoints(e, a->Dest, b->Dest, isMajor);

            if (isMajor) {
                V.TryAdd((IntPtr) a->Dest);
                V.TryAdd((IntPtr) b->Dest);
            }

            DestroyedTriangle(e->TriangleId);
            DestroyedTriangle(e->Sym->TriangleId);

            UnityEngine.Debug.Assert(MathLib.IsEpsEqual(e->MaterialTypeIndex, e->Sym->MaterialTypeIndex, 0.01f), "Material Types are different!");
            NewTriangle(e, e->MaterialTypeIndex);
            NewTriangle(e->Sym, e->MaterialTypeIndex);
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
        void FlipEdges(float2 p, bool isMajor) // Calls Swap
        {
            //UnityEngine.Debug.Log($"FlipEdges: isMajor: {isMajor}");
            ref Collections.PtrStack<Edge> flipStackUsing = ref isMajor ? ref _flipStack : ref _flipStackMinor;
            while (flipStackUsing.Count > 0)
            {
                var edge = flipStackUsing.Pop();

                Assert.IsTrue(Math.Ccw(edge->Org->Point, edge->Dest->Point, p));

                Edge.VerifyEdge(edge, isMajor);

                if (/* !e->Constrained */ !edge->IsEdgeTypeConstrained() && Math.CircumcircleContains(edge->Org->Point, edge->Dest->Point, p, edge->DNext->Org->Point))
                {
                    flipStackUsing.Push(edge->OPrev);
                    flipStackUsing.Push(edge->DNext);
                    Assert.IsTrue(Math.Ccw(edge->OPrev->Org->Point, edge->OPrev->Dest->Point, p));
                    Assert.IsTrue(Math.Ccw(edge->DNext->Org->Point, edge->DNext->Dest->Point, p));
                    Swap(edge);
                }
            }
        }

        // http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.61.3862&rep=rep1&type=pdf
        // TriangulatePseudopolygonDelaunay()
        void RetriangulateFace(Edge* edge, bool isMajor)
        {
            UnityEngine.Debug.Log("RetriangulateFace");
            Assert.IsTrue(edge != null);
            Assert.IsTrue(edge != edge->LNext->LNext);
            // Should not Verify edge because it may not have been given a crep or added cid yet
            if (edge->LNext->LNext->LNext == edge)
            {
                NewTriangle(edge, edge->MaterialTypeIndex);
                return;
            }


            InfiniteLoopDetection.Reset();
            while (!Math.Ccw(edge->Org->Point, edge->Dest->Point, edge->LNext->Dest->Point))
            {
                InfiniteLoopDetection.Register(1000, "RetriangulateFace 0");
                edge = edge->LNext;
                Edge.VerifyEdge(edge);
            }

            var c = edge->LNext;
            var e = c;
            InfiniteLoopDetection.Reset();
            while (true)
            {
                InfiniteLoopDetection.Register(1000, "RetriangulateFace 1");
                e = e->LNext;
                Edge.VerifyEdge(e);
                if (e->LNext == edge)
                    break;
                if (Math.CircumcircleContains(edge->Org->Point, edge->Dest->Point, c->Dest->Point, e->Dest->Point))
                    c = e;
            }

            // All Edges connecting to Major (i.e. an Obstacle) constraint are Clearance edges
            // All Edges connecting to Minor constraints are TerrainSub edges
            Edge.Type newConnectionEdgesType = isMajor ? Edge.Type.Major | Edge.Type.Clearance : Edge.Type.Minor | Edge.Type.Ignore;

            Assert.IsTrue(c != edge);
            var connected = false;
            if (c->LNext->LNext != edge)
            {
                if (isMajor) {
                    V.TryAdd((IntPtr) edge->LPrev->Dest);
                    V.TryAdd((IntPtr) c->LNext->Org);
                }

                var b = Connect(edge->LPrev, c->LNext, newConnectionEdgesType, null);
                b->MaterialTypeIndex = edge->MaterialTypeIndex;
                Edge.VerifyEdge(b);
                RetriangulateFace(b, isMajor);
                connected = true;
            }

            if (c != edge->LNext)
            {
                if (isMajor) {
                    V.TryAdd((IntPtr) c->Dest);
                    V.TryAdd((IntPtr) edge->LNext->Org);
                }

                var a = Connect(c, edge->LNext, newConnectionEdgesType, null);
                a->MaterialTypeIndex = edge->MaterialTypeIndex;
                Edge.VerifyEdge(a);
                RetriangulateFace(a, isMajor);
                connected = true;
            }

            if (connected) {
                NewTriangle(edge, edge->MaterialTypeIndex);
            }
        }

        Edge* RemoveVertex(Vertex* vert, bool isMajor)
        {
            //UnityEngine.Debug.Log("RemoveVertex");
            Assert.IsTrue(vert->GetEdge(isMajor) != null);

            Assert.IsTrue(vert->GetEdge(true) == null || vert->GetEdge(true)->Org == vert);
            Assert.IsTrue(vert->GetEdge(false) == null || vert->GetEdge(false)->Org == vert);

            // If removing Minor vertex, must not be any connected Major edges. But if removing Major edge, it's fine if there are Minor edges, we just won't remove them
            Assert.IsTrue(MathLib.LogicalIf(!isMajor, vert->GetEdge(true) == null));

            var remaining = vert->GetEdge(isMajor)->LNext;
            InfiniteLoopDetection.Reset();
            while (true)
            {
                InfiniteLoopDetection.Register(1000, "RemoveVertex");

                var e = vert->GetEdge(isMajor);
                if (e == null) {
                    break;
                }
                Edge.VerifyEdge(e);
                RemoveEdge(e, true, isMajor);
            }

            if (vert->GetEdge(!isMajor) == null) {
                UnityEngine.Debug.Assert(vert->GetEdge(false) == null && vert->GetEdge(true) == null, "The if statement above is wrong");

                UnityEngine.Debug.Assert(isMajor == false, "I expect that vertices can only be fully destroyed by Minor graph");

                // Since Minor deletes vertices, we should call V.Remove() even if not isMajor
                V.Remove((IntPtr) vert);

                Assert.IsTrue(vert->GetEdge(false) == null && vert->GetEdge(true) == null); // Ensure edges have all been removed
                _qt.Remove(vert);
                var delPos = vert->SeqPos;
                ((Vertex*) _verticesSeq[^1])->SeqPos = delPos;
                _verticesSeq.RemoveAtSwapBack(delPos);
                _vertices.Recycle(vert);
            } else if (isMajor) {
                //UnityEngine.Debug.Log($"Major is calling RemoveVertexIfEligible: {isMajor}");
                RemoveVertexIfEligible(vert, false);
            }
            return remaining;
        }

        /// <summary>
        /// Returns an edge for which the specified point is contained within it's left face. If the point lies
        /// on an edge this edge is returned. If the point lies on a vertex an arbitrary edge with identical origin is returned.
        /// </summary>
        public void FindTrianglesContainingPoint(float2 p, out Edge* edgeMinor, out Edge* edgeMajor)
        {
            edgeMinor = FindTriangleContainingPoint(p, false, out _);
            edgeMajor = null;

            if (edgeMinor->HasMajorEdge) {
                edgeMajor = edgeMinor->MajorEdge;
                return;
            } else if (edgeMinor->LPrev->HasMajorEdge) {
                edgeMajor = edgeMinor->LPrev->MajorEdge;
                return;
            } else if (edgeMinor->LNext->HasMajorEdge) {
                edgeMajor = edgeMinor->LNext->MajorEdge;
                return;
            }

            // Breadth first search for the first majorEdge we can find // TODO: Make this Edge-Based
            _openVertexQueue.Clear();

            int mark = NextMark;
            EnqueueDest(ref _openVertexQueue, edgeMinor, mark);
            EnqueueDest(ref _openVertexQueue, edgeMinor->LPrev, mark);
            EnqueueDest(ref _openVertexQueue, edgeMinor->LNext, mark);
            int numSearches = 0;
            while (_openVertexQueue.Length > 0)
            {
                numSearches++;
                Vertex* vert = _openVertexQueue.Dequeue();
                var i = vert->GetEdgeEnumerator(false);
                while (i.MoveNext())
                {
                    Edge.VerifyEdge(i.Current, false);

                    if (i.Current->HasMajorEdge) {
                        edgeMajor = i.Current->MajorEdge;
                        goto FoundEdgeMajor;
                    } else if (i.Current->Mark != mark) {
                        EnqueueDest(ref _openVertexQueue, i.Current, mark);
                    }
                }
            }
            void EnqueueDest(ref UnsafeCircularQueue<Ptr<Vertex>> openVertexQueue, Edge* e, int mark) {
                e->Mark = mark;
                openVertexQueue.Enqueue(e->Dest);
            }
FoundEdgeMajor:
            // CommonLib.DebugVector(edgeMajor->Org->Point3D, edgeMajor->SegVector3D+new float3(0.007f), UnityEngine.Color.white, 0.008f, 0.03f);
            edgeMajor = FindTriangleContainingPoint(edgeMajor, edgeMajor->Org, p, false, out _);
        }

        public Edge* FindTriangleContainingPoint(float2 p, bool isMajor) => FindTriangleContainingPoint(p, isMajor, out _);

        /// <summary>
        /// Returns an edge for which the specified point is contained within it's left face. If the point lies
        /// on an edge this edge is returned. If the point lies on a vertex an arbitrary edge with identical origin is returned.
        /// </summary>
        /// <param name="collinear">True when the specified point lies on the returned edge</param>
        public Edge* FindTriangleContainingPoint(float2 p, bool isMajor, out bool collinear)
            => FindTriangleContainingPoint(FindClosestVertex(p, isMajor), p, isMajor, out collinear);

        Edge* FindTriangleContainingPoint(Vertex* v, float2 p, bool isMajor, out bool collinear)
            => FindTriangleContainingPoint(v->GetEdge(isMajor), v, p, isMajor, out collinear);

        Edge* FindTriangleContainingPoint(Edge* e, Vertex* v, float2 p, bool isMajor, out bool collinear)
        {
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
        public Vertex* FindClosestVertex(float2 p, bool isMajor)
        {
            Assert.IsTrue(Contains(p), "Trying to find the closest vertex to a point outside the navmesh");
            return _qt.FindClosest(p, isMajor ? Vertex.Type.Major : Vertex.Type.Minor);
        }

        Vertex* InsertPoint(float2 p, Edge.Type newConstraintEdgeType, Vertex* existingMajorVertex = null)
        {
            //UnityEngine.Debug.Log($"InsertPoint; {newConstraintEdgeType}, Vertex.Type: {(Vertex.Type)(newConstraintEdgeType & (Edge.Type.Major | Edge.Type.Minor))}");
            var closest = _qt.FindClosest(p, (Vertex.Type)(newConstraintEdgeType & (Edge.Type.Major | Edge.Type.Minor))); // Extract the Major or Minor bit

            if (math.lengthsq(closest->Point - p) <= _e * _e) {
                //UnityEngine.Debug.Log($"InsertPoint - Found vertex, returning it instead of inserting new. Vertex.Type: {closest->VertexType}");
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
                    return InsertPointInEdge(p, inEdge, newConstraintEdgeType, true, existingMajorVertex);
                }

                return InsertPointInFace(p, e, newConstraintEdgeType, existingMajorVertex);
            }
        }

        Vertex* InsertPointInEdge(float2 point, Edge* edge, Edge.Type newConstraintEdgeType, bool debugIsNewPoint, Vertex* existingMajorVertex = null)
        {
            UnityEngine.Debug.Log($"InsertPointInEdge; {newConstraintEdgeType}");

            bool isMajor = Edge.IsEdgeTypeMajor(newConstraintEdgeType);
            Edge.VerifyEdge(edge, isMajor);

            const int NumFlips = 4;
            ref Collections.PtrStack<Edge> flipStackUsing = ref isMajor ? ref _flipStack : ref _flipStackMinor;
            flipStackUsing.Push(edge->ONext->Sym);
            flipStackUsing.Push(edge->DPrev->Sym);
            flipStackUsing.Push(edge->OPrev);
            flipStackUsing.Push(edge->DNext);

            for (var i = 0; i < NumFlips; i++) {
                Assert.IsTrue(Math.Ccw(flipStackUsing[i]->Org->Point, flipStackUsing[i]->Dest->Point, point));
            }

            DestroyedTriangle(edge->TriangleId);
            DestroyedTriangle(edge->Sym->TriangleId);


            UnityEngine.Debug.Assert(isMajor == Edge.IsEdgeTypeMajor(edge->EdgeType), $"edge->EdgeType: {edge->EdgeType}, newConstraintEdgeType: {newConstraintEdgeType}");
            UnityEngine.Debug.Assert(MathLib.LogicalIf(isMajor, existingMajorVertex == null), $"newConstraintEdgeType: {newConstraintEdgeType}");
            UnityEngine.Debug.Assert(MathLib.LogicalIf(!debugIsNewPoint && newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Obstacle), edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Terrain)), $"edge->EdgeType: {edge->EdgeType}");
            UnityEngine.Debug.Assert(MathLib.LogicalIf(!debugIsNewPoint && newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Terrain), newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Minor)
                && edge->EdgeType.HasAnyFlagsB(Edge.Type.Minor) && edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Terrain | Edge.Type.Clearance)), $"edge->EdgeType: {edge->EdgeType}");
            UnityEngine.Debug.Assert(MathLib.LogicalIf(!debugIsNewPoint && newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Clearance), newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Minor)
                && edge->EdgeType.HasAnyFlagsB(Edge.Type.Minor) && edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Terrain | Edge.Type.Clearance)), $"edge->EdgeType: {edge->EdgeType}");

            Edge.Type newConnectionEdgesType = Edge.Type.None;
            if (isMajor) { // Major Obstacles only intersect with Major Obstacles
                UnityEngine.Debug.Assert(!debugIsNewPoint && edge->EdgeType.HasAllFlagsB(Edge.Type.Major | Edge.Type.Obstacle) && edge->EdgeType == newConstraintEdgeType, $"edge->EdgeType: {edge->EdgeType}");
                newConnectionEdgesType = Edge.Type.Major | Edge.Type.Clearance;
            } else { // is Minor edge
                UnityEngine.Debug.Assert(debugIsNewPoint || (edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Obstacle) || edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance) || edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Terrain))
                    && (newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Obstacle) || newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance) || newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Terrain))
                    , $"edge->EdgeType: {edge->EdgeType}, This should also accept Type.Clearance");
                // Should not be intersecting Minor Obstacle with Minor Clearance
                UnityEngine.Debug.Assert(!(edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance) && newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Obstacle)));
                // Should not be intersecting Minor Clearance with Minor Clearance
                UnityEngine.Debug.Assert(!(edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance) && newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance)));
                UnityEngine.Debug.Assert(MathLib.LogicalIf(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Clearance), edge->EdgeType.HasNoFlagsB(Edge.Type.Clearance)), $"edge->EdgeType: {edge->EdgeType}");
                newConnectionEdgesType = Edge.Type.Minor | Edge.Type.Ignore;
            }
            
            var crep = edge->QuadEdge->Crep;
            var e = edge->OPrev;
            if (isMajor) { // TODO: Should this be no if statement?
                C.Remove((IntPtr) edge);
            }

            Edge* majorEdge = edge->MajorEdge;
            Edge.Type newSplitConstraintEdgesType = edge->EdgeType;
            byte material = edge->MaterialTypeIndex;
            byte symMaterial = edge->Sym->MaterialTypeIndex;
                // CommonLib.DebugSeg(edge->Org->Point3D, edge->Dest->Point3D, UnityEngine.Color.blue, 0.002f, math.INFINITY, 0.065f);
                // CommonLib.DebugSeg(edge->Sym->Org->Point3D, edge->Sym->Dest->Point3D, UnityEngine.Color.cyan, 0.002f, math.INFINITY, 0.065f);

            if (material != symMaterial) {
                UnityEngine.Debug.Log($"material: {material}, symMaterial: {symMaterial}");
            }

            RemoveEdge(edge, false, isMajor);


            var result = existingMajorVertex != null ? existingMajorVertex : CreateVertex(point);
            if (isMajor) {
                V.TryAdd((IntPtr) result);
                V.TryAdd((IntPtr) e->Org);
            }
            var newEdge = CreateEdge(e->Org, result, newSplitConstraintEdgesType, majorEdge);
            newEdge->QuadEdge->Crep = GetCrep(crep);
            Edge.VerifyEdge(newEdge);
            Splice(newEdge, e);

            if (isMajor) {
                V.TryAdd((IntPtr) e->Dest);
                V.TryAdd((IntPtr) newEdge->Sym->Org);
            }

            newEdge = Connect(e, newEdge->Sym, newConnectionEdgesType, null);
            Edge.VerifyEdge(newEdge);
            e = newEdge->OPrev;

            if (isMajor) {
                V.TryAdd((IntPtr) e->Dest);
                V.TryAdd((IntPtr) newEdge->Sym->Org);
            }

            newEdge = Connect(e, newEdge->Sym, newSplitConstraintEdgesType, majorEdge);
            newEdge->QuadEdge->Crep = crep;
            Edge.VerifyEdge(newEdge);
            e = newEdge->OPrev;

            if (isMajor) {
                V.TryAdd((IntPtr) e->Dest);
                V.TryAdd((IntPtr) newEdge->Sym->Org);
            }

            var debugE = Connect(e, newEdge->Sym, newConnectionEdgesType, null);
            Edge.VerifyEdge(debugE);

            var te = result->GetEdge(isMajor);
            NewTriangle(te, material);

                // CommonLib.DebugSeg(te->Org->Point3D, te->Dest->Point3D, UnityEngine.Color.green, 0.002f, math.INFINITY, 0.05f);
            
            te = te->ONext;
            NewTriangle(te, symMaterial);

                // CommonLib.DebugSeg(te->Org->Point3D, te->Dest->Point3D, UnityEngine.Color.white, 0.002f, math.INFINITY, 0.05f);
            
            te = te->ONext;
            NewTriangle(te, symMaterial);

                // CommonLib.DebugSeg(te->Org->Point3D, te->Dest->Point3D, UnityEngine.Color.yellow, 0.002f, math.INFINITY, 0.05f);
            
            te = te->ONext;
            NewTriangle(te, material);

                // CommonLib.DebugSeg(te->Org->Point3D, te->Dest->Point3D, UnityEngine.Color.red, 0.002f, math.INFINITY, 0.05f);
            

            FlipEdges(point, isMajor);
            return result;
        }

        UnsafeList<Entity> GetCrep(UnsafeList<Entity> source)
        {
            var l = GetCrep();
            l.AddRange(source);
            return l;
        }

        UnsafeList<Entity> GetCrep() => _creps.Count > 0 ? _creps.Pop() : new UnsafeList<Entity>(CrepMinCapacity, Allocator.Persistent);

        Vertex* InsertPointInFace(float2 p, Edge* edge, Edge.Type newConstraintEdgeType, Vertex* existingMajorVertex = null)
        {
            UnityEngine.Debug.Log($"InsertPointInFace; {newConstraintEdgeType}");

            bool isMajor = Edge.IsEdgeTypeMajor(newConstraintEdgeType);
            Edge.VerifyEdge(edge, isMajor);

            const int NumFlips = 3;
            ref Collections.PtrStack<Edge> flipStackUsing = ref isMajor ? ref _flipStack : ref _flipStackMinor;
            flipStackUsing.Push(edge->ONext->Sym);
            flipStackUsing.Push(edge);
            flipStackUsing.Push(edge->DPrev->Sym);

            for (var i = 0; i < NumFlips; i++) {
                Assert.IsTrue(Math.Ccw(flipStackUsing[i]->Org->Point, flipStackUsing[i]->Dest->Point, p));
            }

            DestroyedTriangle(edge->TriangleId);

            var result = existingMajorVertex != null ? existingMajorVertex : CreateVertex(p);

            if (isMajor) {
                V.TryAdd((IntPtr) result);
                V.TryAdd((IntPtr) edge->Org);
                V.TryAdd((IntPtr) edge->Dest);
                V.TryAdd((IntPtr) edge->LNext->Dest);
            }

            UnityEngine.Debug.Assert(MathLib.LogicalIf(isMajor, existingMajorVertex == null), $"newConstraintEdgeType: {newConstraintEdgeType}");

            byte material = edge->MaterialTypeIndex;
            Edge.Type newConnectionEdgesType = Edge.Type.None;
            if (isMajor) { // All Edges connecting to Major (i.e. an Obstacle) constraint are Clearance edges
                UnityEngine.Debug.Assert(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Obstacle), $"newConstraintEdgeType: {newConstraintEdgeType}, edge->EdgeType: {edge->EdgeType}");
                newConnectionEdgesType = Edge.Type.Major | Edge.Type.Clearance;
            } else { // All Edges connecting to Minor constraints are TerrainSub edges
                UnityEngine.Debug.Assert(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Terrain | Edge.Type.Obstacle | Edge.Type.Clearance), $"newConstraintEdgeType: {newConstraintEdgeType}, edge->EdgeType: {edge->EdgeType}");
                newConnectionEdgesType = Edge.Type.Minor | Edge.Type.Ignore;
            }

            var newEdge = CreateEdge(edge->Org, result, newConnectionEdgesType, null);
            Splice(newEdge, edge);
            newEdge = Connect(edge, newEdge->Sym, newConnectionEdgesType, null);
            var debugE = Connect(newEdge->OPrev, newEdge->Sym, newConnectionEdgesType, null);

            var te = result->GetEdge(isMajor);
            NewTriangle(te, material);
            te = te->ONext;
            NewTriangle(te, material);
            te = te->ONext;
            NewTriangle(te, material);

            FlipEdges(p, isMajor);
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

        void InsertSegment(Vertex* a, Vertex* b, Entity cid, Edge.Type newConstraintEdgeType, Edge* majorEdge)
        {
            //UnityEngine.Debug.Log($"InsertSegment; {newConstraintEdgeType}");
            var dir = math.normalize(b->Point - a->Point);
            InsertSegmentRecursive(a, b, cid, dir, a->Point, b->Point, newConstraintEdgeType, majorEdge);
        }

        void InsertSegmentRecursive(Vertex* a, Vertex* b, Entity cid, float2 dir, float2 start, float2 end, Edge.Type newConstraintEdgeType, Edge* majorEdge)
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
                        InsertSegmentRecursive(p0.Vertex, p1.Vertex, cid, dir, start, end, newConstraintEdgeType, majorEdge);
                    } else {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - neither modified, neither found existing");
                        InsertSegmentNoCrossConstraints(p0.Vertex, p1.Vertex, cid, newConstraintEdgeType, majorEdge);
                    }
                }
                else if (p0.Modified && !p1.Modified)
                {
                    if (GetSupport(p0.After + _e / 2 * dir, p1.P - _e * dir, dir, out var p))
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - p0 modified, insert point");
                        var after = InsertPoint(p, newConstraintEdgeType);
                        InsertSegmentRecursive(after, p1.Vertex, cid, dir, start, end, newConstraintEdgeType, majorEdge);
                        _edgeSearch.Search(p0.Vertex, after, cid, newConstraintEdgeType, majorEdge);
                    }
                    else
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - p0 modified");
                        _edgeSearch.Search(p0.Vertex, p1.Vertex, cid, newConstraintEdgeType, majorEdge);
                    }
                }
                else if (!p0.Modified) // p1 modified
                {
                    if (GetSupport(p1.Before - _e / 2 * dir, p0.P + _e * dir, -dir, out var p))
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - p1 modified, insert point");
                        var before = InsertPoint(p, newConstraintEdgeType);
                        InsertSegmentRecursive(p0.Vertex, before, cid, dir, start, end, newConstraintEdgeType, majorEdge);
                        _edgeSearch.Search(before, p1.Vertex, cid, newConstraintEdgeType, majorEdge);
                    }
                    else
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - p1 modified");
                        _edgeSearch.Search(p0.Vertex, p1.Vertex, cid, newConstraintEdgeType, majorEdge);
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
                        InsertSegmentRecursive(v0, v1, cid, dir, start, end, newConstraintEdgeType, majorEdge);
                        _edgeSearch.Search(p0.Vertex, v0, cid, newConstraintEdgeType, majorEdge);
                        _edgeSearch.Search(v1, p1.Vertex, cid, newConstraintEdgeType, majorEdge);
                    }
                    else
                    {
                        UnityEngine.Debug.Log("InsertSegmentRecursive - both modified");
                        _edgeSearch.Search(p0.Vertex, p1.Vertex, cid, newConstraintEdgeType, majorEdge);
                    }
                }

                a = p1.Vertex;
                _insertedPoints.Add(p1);
            }
        }

        Point GetNextPoint(Vertex* a, Vertex* b, float2 start, float2 end, Edge.Type newConstraintEdgeType)
        {
            //UnityEngine.Debug.Log($"GetNextPoint; {newConstraintEdgeType}");
            InfiniteLoopDetection.Reset();

            bool isMajor = Edge.IsEdgeTypeMajor(newConstraintEdgeType);

            var e = GetLeftEdge(a, b->Point, isMajor);
            while (e->Dest != b)
            {
                InfiniteLoopDetection.Register(1000, "GetNextPoint");

                var d = Math.TriArea(a->Point, b->Point, e->Dest->Point);


                UnityEngine.Debug.Assert(Edge.IsEdgeTypeMajor(e->EdgeType) == isMajor);
                Edge.VerifyEdge(e, isMajor);
                if (isMajor) {
                    UnityEngine.Debug.Assert(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Obstacle), $"newConstraintEdgeType: {newConstraintEdgeType}, edge->EdgeType: {e->EdgeType}");
                } else {
                    UnityEngine.Debug.Assert(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Terrain | Edge.Type.Obstacle | Edge.Type.Clearance), $"newConstraintEdgeType: {newConstraintEdgeType}, edge->EdgeType: {e->EdgeType}");
                }
                // TODO: If newConstraint is an Obstacle or a Clearance, and e is a Clearance then should isEdgeConstrained be false?
                // Does it change anything? Improve performance?


                if (d < 0 && e->IsEdgeTypeConstrained() /* e->Constrained */)
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
                        //UnityEngine.Debug.Log("GetNextPoint - CreatePRef");
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

                    UnityEngine.Debug.Assert(MathLib.LogicalIf(newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Clearance | Edge.Type.Obstacle), !e->EdgeType.HasAnyFlagsB(Edge.Type.Clearance)), $"edge->EdgeType: {e->EdgeType}");
                    var vert = InsertPointInEdge(p, e, newConstraintEdgeType, false);
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
            //UnityEngine.Debug.Log($"CreatePRef; {newConstraintEdgeType}");
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
                    return InsertPointInEdge(pplus, e, newConstraintEdgeType, false);

                var pmin = p - offset * dir;
                pointPresent = TryGetPoint(pmin, e, out vertex);

                if (vertex != null)
                    return vertex;

                if (!pointPresent && SplitIsRobust(pmin, e))
                    return InsertPointInEdge(pmin, e, newConstraintEdgeType, false);
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
        void InsertSegmentNoCrossConstraints(Vertex* a, Vertex* b, Entity cid, Edge.Type newConstraintEdgeType, Edge* majorEdge)
        {
            //UnityEngine.Debug.Log($"InsertSegmentNoCrossConstraints; {newConstraintEdgeType}");
            bool isMajor = Edge.IsEdgeTypeMajor(newConstraintEdgeType);
            var c = GetConnection(a, b, isMajor);

            if (c != null)
            {
                UnityEngine.Debug.Log($"InsertSegmentNoCrossConstraints - Found existing edge");
                if (isMajor) {
                    C.TryAdd((IntPtr) c);
                }
                if (!c->IsConstrainedBy(cid)) {
                    c->AddConstraint(cid);
                }
                ResetClearance(c);
                OverwriteEdgeType(c, newConstraintEdgeType, majorEdge);
                Edge.VerifyEdge(c);
                return;
            }

            var e = GetLeftEdge(a, b->Point, isMajor);

            InfiniteLoopDetection.Reset();
            while (e->Dest != b)
            {
                InfiniteLoopDetection.Register(1000, "InsertSegmentNoCrossConstraints");

                var d = Math.TriArea(a->Point, b->Point, e->Dest->Point);
                var next = d > 0 ? e->RPrev : e->ONext;

                if (d < 0)
                {
                    UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints - Weird thing -> Removing edge");
                    Assert.IsTrue(!e->IsConstrained);
                    RemoveEdge(e, true, isMajor);
                }
                else if (d == 0 && e->Dest != a)
                {
                    var t = e->Dest;
                    UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints - Connect 1");
                    Connect(a, t, cid, newConstraintEdgeType, majorEdge);
                    a = t;
                }

                e = next;
                Edge.VerifyEdge(e);
            }

            UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints - Connect 2");
            Connect(a, b, cid, newConstraintEdgeType, majorEdge);
        }

        internal static void SetEdgeTypeMajor(Edge* edge, Edge.Type newEdgeType, Edge* majorEdge) {
            Edge.Type existingEdgeType = edge->EdgeType;

            // Debug: // This might be fine:
            if (edge->MajorEdge != null && majorEdge != null) {
                UnityEngine.Debug.Assert(edge->ContainsMajorEdge(majorEdge),
                    $"edge->MajorEdge: {(long)edge->MajorEdge}, majorEdge: {(long)majorEdge}, edge->MajorEdgeType: {edge->MajorEdge->EdgeType}, majorEdge->EdgeType: {majorEdge->EdgeType}");
            }

            UnityEngine.Debug.Assert(Edge.IsEdgeTypeMajor(existingEdgeType) && Edge.IsEdgeTypeMajor(newEdgeType), $"edgeType: {existingEdgeType}, newEdgeType: {newEdgeType}");
            
            UnityEngine.Debug.Assert(existingEdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance) && newEdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance)
                , $"edgeType: {existingEdgeType}, newEdgeType: {newEdgeType}");

            UnityEngine.Debug.Assert(MathLib.LogicalIf(edge->IsConstrained, newEdgeType.HasAllFlagsB(Edge.Type.Obstacle)));
            UnityEngine.Debug.Assert(MathLib.LogicalIf(newEdgeType.HasAllFlagsB(Edge.Type.Clearance), !edge->IsConstrained));

            edge->SetEdgeType(newEdgeType);

            edge->MajorEdge = majorEdge;

            // Go through all minor edges referencing this major edge and Overwrite their edge type as well
            Edge* prevEdge = null;
            Vertex* currentMinorOrg = edge->Org;
            while (currentMinorOrg != edge->Dest)
            {
                if (currentMinorOrg->GetEdge(false) == null) {
                    break;
                }
                var i = currentMinorOrg->GetEdgeEnumerator(false);
                while (i.MoveNext())
                {
                    if (i.Current->ContainsMajorEdge(edge) && i.Current->Sym != prevEdge)
                    {
                        SetEdgeTypeMinor(i.Current, Edge.Type.Minor | (newEdgeType & ~Edge.Type.Major), edge);
                        currentMinorOrg = i.Current->Dest;
                        prevEdge = i.Current;
                        break;
                    }
                }
                if (currentMinorOrg == edge->Org) {
                    break;
                }
            }

            Edge.VerifyEdge(edge);
            //UnityEngine.Debug.Log($"SetEdgeTypeMajor, newEdgeType: {newEdgeType}, EdgeType before: {existingEdgeType}, EdgeType after: {edge->EdgeType}");
        }

        internal static void SetEdgeTypeMinor(Edge* edge, Edge.Type newEdgeType, Edge* majorEdge) {
            Edge.Type existingEdgeType = edge->EdgeType;

            // Debug: // This might be fine:
            if (edge->MajorEdge != null && majorEdge != null) {
                UnityEngine.Debug.Assert(edge->ContainsMajorEdge(majorEdge),
                    $"edge->MajorEdge: {(long)edge->MajorEdge}, majorEdge: {(long)majorEdge}, edge->MajorEdgeType: {edge->MajorEdge->EdgeType}, majorEdge->EdgeType: {majorEdge->EdgeType}");
            }

            UnityEngine.Debug.Assert(!Edge.IsEdgeTypeMajor(existingEdgeType) && !Edge.IsEdgeTypeMajor(newEdgeType), $"edgeType: {existingEdgeType}, newEdgeType: {newEdgeType}");
            

            UnityEngine.Debug.Assert(existingEdgeType.HasAnyFlagsB(Edge.Type.Terrain | Edge.Type.Ignore | Edge.Type.Obstacle | Edge.Type.Clearance)
                && newEdgeType.HasAnyFlagsB(Edge.Type.Terrain | Edge.Type.Ignore | Edge.Type.Obstacle | Edge.Type.Clearance), $"edgeType: {existingEdgeType}, newEdgeType: {newEdgeType}");

            UnityEngine.Debug.Assert(MathLib.LogicalIf(edge->MajorEdge != null, existingEdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance)));
            UnityEngine.Debug.Assert(MathLib.LogicalIf(majorEdge != null, newEdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance)));

            UnityEngine.Debug.Assert(MathLib.LogicalIf(edge->IsConstrained, newEdgeType.HasAllFlagsB(Edge.Type.Terrain)));
            UnityEngine.Debug.Assert(MathLib.LogicalIf(newEdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance | Edge.Type.Ignore), !edge->IsConstrained));
            

            edge->SetEdgeType(newEdgeType);

            UnityEngine.Debug.Assert(MathLib.LogicalIf(existingEdgeType.HasAnyFlagsB(Edge.Type.Clearance), newEdgeType.HasNoFlagsB(Edge.Type.Obstacle))
                , $"edgeType: {existingEdgeType}, newEdgeType: {newEdgeType}, this is probably fine, clearance is often overwritten by obstacle");

            edge->MajorEdge = majorEdge;

            Edge.VerifyEdge(edge);
            //UnityEngine.Debug.Log($"SetEdgeTypeMinor, newEdgeType: {newEdgeType}, EdgeType before: {existingEdgeType}, EdgeType after: {edge->EdgeType}");
        }

        internal static void OverwriteEdgeType(Edge* edge, Edge.Type newConstraintEdgeType, Edge* majorEdge) {
            Edge.Type existingEdgeType = edge->EdgeType;

            UnityEngine.Debug.Assert(Edge.IsEdgeTypeMajor(existingEdgeType) == Edge.IsEdgeTypeMajor(newConstraintEdgeType), $"edgeType: {existingEdgeType}, newConstraintEdgeType: {newConstraintEdgeType}");
            
            if (Edge.IsEdgeTypeMajor(existingEdgeType)) {
                UnityEngine.Debug.Assert(existingEdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance) && newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Obstacle)
                    , $"edgeType: {existingEdgeType}, newConstraintEdgeType: {newConstraintEdgeType}");
                UnityEngine.Debug.Assert(MathLib.LogicalIf(existingEdgeType.HasAnyFlagsB(Edge.Type.Obstacle), newConstraintEdgeType.HasNoFlagsB(Edge.Type.Clearance))
                    , $"edgeType: {existingEdgeType}, newConstraintEdgeType: {newConstraintEdgeType}"); // Clearance cannot overwrite an Obstacle

                SetEdgeTypeMajor(edge, newConstraintEdgeType, majorEdge);

            } else {
                if (existingEdgeType.HasAnyFlagsB(Edge.Type.Terrain | Edge.Type.Ignore)) {
                    // All overwrite Terrain
                    SetEdgeTypeMinor(edge, newConstraintEdgeType, majorEdge);
                } else if (existingEdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance)) {
                    if (newConstraintEdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance)) {
                        UnityEngine.Debug.Assert(edge->ContainsMajorEdge(majorEdge));
                    } else {
                        UnityEngine.Debug.Assert(existingEdgeType.HasAnyFlagsB(Edge.Type.Terrain));
                        UnityEngine.Debug.Assert(edge->MajorEdge != null && majorEdge == null);
                    }
                    // Minor Terrain cannot overwrite Minor Obstacle or Clearance, so do nothing
                    // Also, Minor Obstacle would never try to overwrite Minor Clearance and vice-versa because they would be removed first
                }
            }

            Edge.VerifyEdge(edge);
            //UnityEngine.Debug.Log($"OverwriteEdgeType, newConstraintEdgeType: {newConstraintEdgeType}, EdgeType before: {existingEdgeType}, EdgeType after: {edge->EdgeType}");
        }

        void Connect(Vertex* a, Vertex* b, Entity cid, Edge.Type newConstraintEdgeType, Edge* majorEdge)
        {
            bool isMajor = Edge.IsEdgeTypeMajor(newConstraintEdgeType);
            var connection = GetConnection(a, b, isMajor);
            if (connection == null)
            {
                //UnityEngine.Debug.Log($"Connect - null connection, so creating one with EdgeType: {newConstraintEdgeType}");
                if (isMajor) {
                    V.TryAdd((IntPtr) a);
                    V.TryAdd((IntPtr) b);
                }

                connection = Connect(a, b, newConstraintEdgeType, majorEdge);
                connection->MaterialTypeIndex = MaxMaterialType(connection->LNext->MaterialTypeIndex, connection->LPrev->MaterialTypeIndex);
                connection->Sym->MaterialTypeIndex = MaxMaterialType(connection->Sym->LNext->MaterialTypeIndex, connection->Sym->LPrev->MaterialTypeIndex);
                RetriangulateFace(connection, isMajor);
                RetriangulateFace(connection->Sym, isMajor);
            } else {
                OverwriteEdgeType(connection, newConstraintEdgeType, majorEdge);
            }

            // todo inline wasUnconstrained (so if moves above addconstraint)
            var wasUnconstrained = !connection->IsConstrained;
            connection->AddConstraint(cid);
            if (wasUnconstrained) {
                ResetClearance(connection);
            }
            if (isMajor) {
                C.TryAdd((IntPtr) connection);
            }

            Edge.VerifyEdge(connection);
        }

        void Connect(Vertex* a, Vertex* b, UnsafeList<Entity> crep, Edge.Type newConstraintEdgeType, Edge* majorEdge, byte material1, byte material2)
        {
            bool isMajor = Edge.IsEdgeTypeMajor(newConstraintEdgeType);
            var connection = GetConnection(a, b, Edge.IsEdgeTypeMajor(newConstraintEdgeType));
            if (connection == null)
            {
                if (isMajor) {
                    V.TryAdd((IntPtr) a);
                    V.TryAdd((IntPtr) b);
                }

                connection = Connect(a, b, newConstraintEdgeType, majorEdge);


                    byte material = MaxMaterialType(connection->LNext->MaterialTypeIndex, connection->LPrev->MaterialTypeIndex);
                    byte symMaterial = MaxMaterialType(connection->Sym->LNext->MaterialTypeIndex, connection->Sym->LPrev->MaterialTypeIndex);
                    // UnityEngine.Debug.Assert(material == material1, $"material: {material}, material1: {material1}");
                    // UnityEngine.Debug.Assert(symMaterial == material2, $"symMaterial: {symMaterial}, material2: {material2}");

                // connection->MaterialTypeIndex = material1 /* material; // 1*/;
                // connection->Sym->MaterialTypeIndex = material2 /* symMaterial; // 1*/;

                RetriangulateFace(connection, isMajor);
                RetriangulateFace(connection->Sym, isMajor);
                UnityEngine.Debug.Log($"Connect crep - Creating new connection");
            } else {
                UnityEngine.Debug.Log($"Connect crep - Found existing connection");
                // SetTriangleMaterialTypeIndex(connection, material1);
                // SetTriangleMaterialTypeIndex(connection->Sym, material2);
                OverwriteEdgeType(connection, newConstraintEdgeType, majorEdge);
            }

            BFSEdgesSetMaterialType(connection, material1 /* 1 */, Entity.Null, true);
            BFSEdgesSetMaterialType(connection->Sym, material2 /* 1 */, Entity.Null, true);

            connection->QuadEdge->Crep = crep;
            ResetClearance(connection);
            if (isMajor) {
                C.TryAdd((IntPtr) connection);
            }

            Edge.VerifyEdge(connection);
        }
    
        bool TryGetPoint(float2 p, Edge* e, out Vertex* v)
        {
            //UnityEngine.Debug.Log("TryGetPoint");
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


        internal void InsertMajorInMinor(Edge* edgeMajor)
        {
            Edge.Type newConstraintEdgeType = Edge.Type.Minor | (edgeMajor->EdgeType & ~Edge.Type.Major); // Force to be Minor Edge
            UnityEngine.Debug.Assert(newConstraintEdgeType.HasNoFlagsB(Edge.Type.Major) && newConstraintEdgeType.HasAllFlagsB(Edge.Type.Minor), $"delete this redundant check; {newConstraintEdgeType}");
            Edge.VerifyEdgeType(newConstraintEdgeType, false);
            //UnityEngine.Debug.Log($"InsertMajorInMinor; {newConstraintEdgeType}");
            Assert.IsTrue(edgeMajor != null);
            Edge.VerifyEdge(edgeMajor);

            Vertex* vertMajor0 = edgeMajor->Org;
            Vertex* vertMajor1 = edgeMajor->Dest;
            Assert.IsTrue(vertMajor0->ContainsEdge(edgeMajor, true) && vertMajor1->ContainsEdge(edgeMajor->Sym, true));

            var vertMinor0 = InsertPoint(vertMajor0->Point, newConstraintEdgeType, vertMajor0);
            var vertMinor1 = InsertPoint(vertMajor1->Point, newConstraintEdgeType, vertMajor1);
            Assert.IsTrue(vertMinor0 == vertMajor0 && vertMinor1 == vertMajor1 && vertMinor0 != null && vertMinor1 != null);

            InsertSegment(vertMinor0, vertMinor1, MinorObstacleCid, newConstraintEdgeType, edgeMajor);

            // Point constraints are already dealt with by Clearance edges
        }

        bool SetMaterialIfPolygon(Entity cid, byte materialTypeIndex) {
            Vertex* firstVertex = (Vertex*) _constraints[cid];

            // Vertex-based Depth first search for all (Quad)Edges of this constraint, figuring out cw/ccw, and then queueing the first face for BFS below
            Edge* prevEdge = null;
            Vertex* currentOrg = firstVertex;
            int numSearches = 0;

            float clockWiseSum = 0;
            while (true)
            {
                numSearches++;
                // CommonLib.CreatePrimitive(UnityEngine.PrimitiveType.Sphere, currentOrg->Point3D, new float3(0.035f, 0.01f, 0.035f), UnityEngine.Color.white);
                int numOutgoingConstraintEdges = 0;
                Edge* newPrevEdge = null;
                var i = currentOrg->GetEdgeEnumerator(false);
                while (i.MoveNext())
                {                    
                    if (i.Current->IsConstrainedBy(cid)) {
                        numOutgoingConstraintEdges++;

                        if (i.Current->Sym != prevEdge) { // prevEdge may be unneccesary in other spot with IsPrimary

                            float2 p1 = currentOrg->Point;
                            float2 p2 = i.Current->Dest->Point;

                            clockWiseSum += (p2.x - p1.x) * (p2.y + p1.y);

                            currentOrg = i.Current->Dest;
                            newPrevEdge = i.Current; // Not setting prevEdge because it will break subsequent iterations
                        }
                    }
                }
                if (numOutgoingConstraintEdges != 2) {
                    return false;
                }

                prevEdge = newPrevEdge;

                // If the following hits, we have reached the end and will break
                if (currentOrg == firstVertex || numSearches > 100) {
                    Assert.IsTrue(numSearches < 100, $"{numSearches}");
                    prevEdge = clockWiseSum > 0 ? prevEdge->Sym : prevEdge;

                    BFSEdgesSetMaterialType(prevEdge, materialTypeIndex, cid, false);

                    // CommonLib.DebugSeg(prevEdge->Org->Point3D, prevEdge->Dest->Point3D, UnityEngine.Color.blue, 0.025f, math.INFINITY, 0.1f);
                    break;
                }
            }
            return true;
        }

        void BFSEdgesSetMaterialType(Edge* initialEdge, byte materialTypeIndex, Entity boundaryCid, bool isStopIfSameIndex, int blah = 0) // If boundaryCid == Entity.Null, BFS stops at any constrained edge
        {
            _openEdgeQueue.Clear();

            int mark = NextMark;
            SetTriangleMaterialTypeIndex(initialEdge, materialTypeIndex);
            _openEdgeQueue.Enqueue(initialEdge->ONext);
            _openEdgeQueue.Enqueue(initialEdge->DPrev);

            if (isStopIfSameIndex) {
                UnityEngine.Color initialColor = MaterialTypes[materialTypeIndex].color;
                initialColor = (initialColor.ToFloat4() * 0.5f).ToColor();
                // CommonLib.DebugSeg(initialEdge->Org->Point3D, initialEdge->Dest->Point3D, initialColor, 0.025f, math.INFINITY, 0.13f);
                // CommonLib.CreateDebugUI(blah, MathLib.Midpoint(initialEdge->Org->Point3D, initialEdge->Dest->Point3D) + math.up());
            }

            while (_openEdgeQueue.Length > 0)
            {
                Edge* edge = _openEdgeQueue.Dequeue();
                if (edge->Mark != mark && (boundaryCid != Entity.Null ? !edge->IsConstrainedBy(boundaryCid) : !edge->IsConstrained)) {
                    SetTriangleMaterialTypeIndex(edge, materialTypeIndex);
                    /* if (isStopIfSameIndex && edge->MaterialTypeIndex == materialTypeIndex && edge->Sym->MaterialTypeIndex == materialTypeIndex) {
                        UnityEngine.Debug.Assert(edge->Sym->MaterialTypeIndex == materialTypeIndex);
                        UnityEngine.Debug.Assert(edge->MaterialTypeIndex == edge->LNext->MaterialTypeIndex && edge->LNext->MaterialTypeIndex == edge->LPrev->MaterialTypeIndex);
                        UnityEngine.Color color = MaterialTypes[edge->MaterialTypeIndex].color;
                        color = (color.ToFloat4() * 0.2f).ToColor();
                        // CommonLib.DebugSeg(edge->Org->Point3D, edge->Dest->Point3D, color, 0.025f, math.INFINITY, 0.1f);
                        // CommonLib.CreateDebugUI(blah, MathLib.Midpoint(edge->Org->Point3D, edge->Dest->Point3D) + math.up());
                        continue;
                    } */
                    if (isStopIfSameIndex) {
                        // CommonLib.DebugSeg(edge->Org->Point3D, edge->Dest->Point3D, MaterialTypes[edge->MaterialTypeIndex].color, 0.025f, math.INFINITY, 0.07f);
                        // CommonLib.CreateDebugUI(blah, MathLib.Midpoint(edge->Org->Point3D, edge->Dest->Point3D) + math.up());
                    }
                    // CommonLib.DebugSeg(edge->Org->Point3D, edge->Dest->Point3D, UnityEngine.Color.yellow, 0.025f, math.INFINITY, 0.1f);
                    _openEdgeQueue.Enqueue(edge->ONext);
                    _openEdgeQueue.Enqueue(edge->DPrev);
                    edge->Mark = mark;
                }
            }
            // bool IsTriangleMarked(Edge* e) => e->Mark != mark && e->LNext->Mark != mark && e->LPrev->Mark != mark;
        }

        internal void InsertMajor(float2* points, int start, int amount, Entity cid, float4x4 ltw, Edge.Type newConstraintEdgeType = Edge.Type.Obstacle) {
            Edge.VerifyEdgeType(Edge.Type.Major | newConstraintEdgeType, true);
            Insert(points, start, amount, cid, ltw, Edge.Type.Major | newConstraintEdgeType);
        }

        internal void InsertMinor(float2* points, int start, int amount, Entity cid, float4x4 ltw, Edge.Type newConstraintEdgeType = Edge.Type.Terrain) {
            Edge.VerifyEdgeType(Edge.Type.Minor | newConstraintEdgeType, false);
            Insert(points, start, amount, cid, ltw, Edge.Type.Minor | newConstraintEdgeType);
        }

        internal void InsertMinorCost(float2* points, int start, int amount, Entity cid, byte materialTypeIndex, float4x4 ltw, Edge.Type newConstraintEdgeType = Edge.Type.Terrain)
        {
            InsertMinor(points, start, amount, cid, ltw, newConstraintEdgeType);
            bool isPolygon = SetMaterialIfPolygon(cid, materialTypeIndex);
            if (!isPolygon) {
                UnityEngine.Debug.LogWarning($"Non polygonal constraint set. Could make this intended in the future?");
            }
        }
        
        void Insert(float2* points, int start, int amount, Entity cid, float4x4 ltw, Edge.Type newConstraintEdgeType)
        {
            //UnityEngine.Debug.Log($"Insert; {newConstraintEdgeType}");
            Vertex* lastVert = null;
            var end = start + amount;
            Vertex* point = null;

            for (var i = start; i < end; i++)
            {
                var c = Math.Mul2D(ltw, points[i]);
                Assert.IsTrue(_verticesSeq.Length < 5 || Contains(c), PointOutsideNavmeshMessage);
                var vert = InsertPoint(c, newConstraintEdgeType);
                Assert.IsTrue(vert != null);

                if (i == start)
                {
                    ++vert->ConstraintHandles;
                    _constraints[cid] = (IntPtr) vert;
                    point = vert;
                }

                if (lastVert != null && vert != lastVert)
                {
                    InsertSegment(lastVert, vert, cid, newConstraintEdgeType, null);
                    point = null;
                }
                lastVert = vert;
            }

            if (point != null) {
                ++point->PointConstraints;
            }
        }

        void AssertDestDoesntContainMajorEdge(Edge* removedMajorEdge)
        {
            var i = removedMajorEdge->Dest->GetEdgeEnumerator(false);
            while (i.MoveNext()) {
                Edge.VerifyEdge(i.Current);
                if (i.Current->ContainsMajorEdge(removedMajorEdge)) {
                    Assert.IsTrue(false, "Dest shouldn't contain it");
                }
            }
        }

        // NOTE: Point constraints are removed automatically in the Minor graph by RemoveConstraint which will call RemoveVertex
        void RemoveMajorInMinor(Edge* removedMajorEdge)
        {
            //UnityEngine.Debug.Log($"RemoveMajorInMinor; majorEdgeType: {removedMajorEdge->EdgeType}");
            Assert.IsTrue(removedMajorEdge->Org != null && removedMajorEdge->Dest != null, "Horrible");
            UnityEngine.Debug.Assert(removedMajorEdge->EdgeType.HasAllFlagsB(Edge.Type.Major | Edge.Type.Obstacle) || removedMajorEdge->EdgeType.HasAllFlagsB(Edge.Type.Major | Edge.Type.Clearance)
                , $"removedMajorEdge->EdgeType: {removedMajorEdge->EdgeType}");

            _vlistMinor.Clear();
            _elistMinor.Clear();

            if (removedMajorEdge->Org->GetEdge(false) == null || removedMajorEdge->Dest->GetEdge(false) == null) { // If no Minor edges attached yet then early-out
                return;
            }
            Edge* prevEdge = null;
            Vertex* currentMinorOrg = removedMajorEdge->Org;
            while (currentMinorOrg != removedMajorEdge->Dest) // Depth first search for all (Quad)Edges that reference removedMajorEdge as their MajorEdge
            {
                var i = currentMinorOrg->GetEdgeEnumerator(false);
                while (i.MoveNext())
                {
                    Edge.VerifyEdge(i.Current, false);
                    if (i.Current->ContainsMajorEdge(removedMajorEdge) && i.Current->Sym != prevEdge)
                    {
                        _elistMinor.Add((IntPtr) i.Current);
                        currentMinorOrg = i.Current->Dest;
                        prevEdge = i.Current;
                        break;
                    }
                }
                if (currentMinorOrg == removedMajorEdge->Org) { // Then removedMajorEdge is not in the Minor graph, so return
                    AssertDestDoesntContainMajorEdge(removedMajorEdge);
                    return;
                }
            }

            //UnityEngine.Debug.Log("Resetting Minor edges that reference this Major edge back to Terrain or TerrainSub");

            Assert.IsTrue(_elistMinor.Length > 0);

            var firstEdge = (Edge*) _elistMinor[0];
            if (firstEdge->Org->PointConstraints == 0) {
                _vlistMinor.Add((IntPtr) firstEdge->Org);
            }
            for (var i = 0; i < _elistMinor.Length; i++) // Add vertices of minor edges to list for potential removal
            {
                var edge = (Edge*) _elistMinor[i];
                if (edge->Dest->PointConstraints == 0) {
                    _vlistMinor.Add((IntPtr) edge->Dest);
                }

                UnityEngine.Debug.Assert(MathLib.LogicalIf(edge->Org->PointConstraints == 0, _vlistMinor.Contains((IntPtr) edge->Org)), "This vertex should already be added");
            }

            {   // Debug to make sure _vlistMinor is valid:
                for (var i = 0; i < _vlistMinor.Length; i++) {
                    for (var j = 0; j < _vlistMinor.Length; j++) {
                        if (i != j && _vlistMinor[i] == _vlistMinor[j]) { Assert.IsTrue(false, "Duplicate in _vlistMinor"); }
                    }
                }
                UnityEngine.Debug.Assert((MathLib.LogicalIf(removedMajorEdge->Org->PointConstraints == 0, (Vertex*) _vlistMinor[0] == removedMajorEdge->Org)
                    && MathLib.LogicalIf(removedMajorEdge->Dest->PointConstraints == 0, (Vertex*) _vlistMinor[_vlistMinor.Length - 1] == removedMajorEdge->Dest))
                    || (MathLib.LogicalIf(removedMajorEdge->Org->PointConstraints == 0, (Vertex*) _vlistMinor[_vlistMinor.Length - 1] == removedMajorEdge->Org)
                    && MathLib.LogicalIf(removedMajorEdge->Dest->PointConstraints == 0, (Vertex*) _vlistMinor[0] == removedMajorEdge->Dest)),
                    $"removedMajorEdge->Org: {(long)removedMajorEdge->Org}, removedMajorEdge->Dest: {(long)removedMajorEdge->Dest}, _vlistMinor[_vlistMinor.Length - 1]: {(long)_vlistMinor[_vlistMinor.Length - 1]}, _vlistMinor[0]: {(long)_vlistMinor[0]}");
            }

            for (var i = 0; i < _elistMinor.Length; i++) // Remove minor Edges
            {
                var edge = (Edge*) _elistMinor[i];

                UnityEngine.Debug.Assert(!Edge.IsEdgeTypeMajor(edge->EdgeType), $"edge->EdgeType: {edge->EdgeType}, removedMajorEdge->EdgeType: {removedMajorEdge->EdgeType}");
                // This may be fine because an Obstacle could have been first turned into a Clearance:
                // UnityEngine.Debug.Assert((edge->EdgeType & ~Edge.Type.Minor) == (removedMajorEdge->EdgeType & ~Edge.Type.Major)
                //     , $"edge->EdgeType: {edge->EdgeType}, removedMajorEdge->EdgeType: {removedMajorEdge->EdgeType}, This is probably fine");

                UnityEngine.Debug.Assert(edge->EdgeType.HasAllFlagsB(Edge.Type.Minor) && edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance),
                    $"edge->EdgeType: {edge->EdgeType}, removedMajorEdge->EdgeType: {removedMajorEdge->EdgeType}");

                ResetClearance(edge); // TODO: Should do this?

                edge->MajorEdge = null;

                if (edge->IsConstrained) {
                    SetEdgeTypeMinor(edge, Edge.Type.Minor | Edge.Type.Terrain, null);
                } else {
                    SetEdgeTypeMinor(edge, Edge.Type.Minor | Edge.Type.Ignore, null);
                    Edge.VerifyEdge(edge, false);

                    _flipStackMinor.Push(edge);
                    FlipQuad(false);
                }

                // Old from RemoveConstraint, keeping for reference and because might want V and Refine eventually
                /* if (!edge->Constrained)
                {
                    V.TryAdd((IntPtr) edge->Org);
                    V.TryAdd((IntPtr) edge->Dest);
                    edge->RefineFailed = false;
                    ResetClearance(edge);

                    _flipStack.Push(edge);
                    FlipQuad();
                } */
            }

            for (var i = 0; i < _vlistMinor.Length; i++) {
                RemoveVertexIfEligible((Vertex*) _vlistMinor[i], false);
            }
        }

        void DFSConstraintEdges(Vertex* initialVertex, int mark, bool isMajor, Entity cid, ref UnsafeList<IntPtr> constraintEdges) {
            Assert.IsTrue(_openStack.Count == 0 && mark == _mark);
            _openStack.Push(initialVertex);
            while (_openStack.Count > 0) // Depth first search for all (Quad)Edges of this constraint
            {
                var vert = _openStack.Pop();
                var i = vert->GetEdgeEnumerator(isMajor);
                while (i.MoveNext())
                {
                    Edge.VerifyEdge(i.Current, isMajor);
                    if (i.Current->IsConstrainedBy(cid) && i.Current->Mark != mark)
                    {
                        constraintEdges.Add((IntPtr) i.Current);
                        i.Current->Mark = mark;
                        _openStack.Push(i.Current->Dest);
                    }
                }
            }
        }


        internal void RemoveConstraintMajor(Entity cid, Edge.Type constraintEdgeType = Edge.Type.Obstacle) {
            Edge.VerifyEdgeType(Edge.Type.Major | constraintEdgeType, true);
            RemoveConstraint(cid, true, Edge.Type.Major | constraintEdgeType, Edge.Type.Major | Edge.Type.Clearance);
        }

        internal void RemoveConstraintMinor(Entity cid, Edge.Type constraintEdgeType = Edge.Type.Terrain) {
            Edge.VerifyEdgeType(Edge.Type.Minor | constraintEdgeType, false);
            RemoveConstraint(cid, false, Edge.Type.Minor | constraintEdgeType, Edge.Type.Minor | Edge.Type.Ignore);
        }

        void RemoveConstraint(Entity cid, bool isMajor, Edge.Type constraintEdgeType, Edge.Type newReplacementEdgeType)
        {
            //UnityEngine.Debug.Log("RemoveConstraint");
            _vlist.Clear();
            _elist.Clear();

            Assert.IsTrue(_constraints.ContainsKey(cid), "Attempting to remove an unknown or static obstacle");
            Vertex* v = (Vertex*) _constraints[cid];
            Assert.IsTrue(v->GetEdge(isMajor) != null);
            Assert.IsTrue(v->ConstraintHandles > 0);
            --v->ConstraintHandles;
            _constraints.Remove(cid);

            int mark = NextMark;

            DFSConstraintEdges(v, mark, isMajor, cid, ref _elist);

            if (_elist.Length == 0) // If no edges to this constraint, then this must be a point constraint
            {
                Assert.IsTrue(isMajor, "PointConstraints can only exist in the Major graph, doesn't make sense in the Minor (Terrain) graph, plus breaks things");
                Assert.IsTrue(v->PointConstraints > 0);
                if (--v->PointConstraints == 0) {
                    _vlist.Add((IntPtr) v);
                }
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

                    UnityEngine.Debug.Assert(isMajor == Edge.IsEdgeTypeMajor(edge->EdgeType) && Edge.IsEdgeTypeMajor(edge->EdgeType) == Edge.IsEdgeTypeMajor(constraintEdgeType)
                        , $"edge->EdgeType: {edge->EdgeType}, constraintEdgeType: {constraintEdgeType}");
                    // Either edge is a Major Obstacle or Minor Terrain, or edge is a Minor Terrain but overwritten by a Minor Obstacle or Minor Clearance
                    UnityEngine.Debug.Assert(edge->EdgeType.HasAllFlagsB(constraintEdgeType) || edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Obstacle)
                        || edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance), $"edge->EdgeType: {edge->EdgeType}, constraintEdgeType: {constraintEdgeType}");
                    UnityEngine.Debug.Assert(MathLib.LogicalIf(!Edge.IsEdgeTypeMajor(edge->EdgeType) && edge->MajorEdge != null,
                        edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Obstacle) || edge->EdgeType.HasAllFlagsB(Edge.Type.Minor | Edge.Type.Clearance)), $"edge->EdgeType: {edge->EdgeType}");

                    edge->RemoveConstraint(cid); // Only remove Entity Id of removed obstacle from edge's list

                    if (!edge->IsConstrained) // If no other constraints attached to this edge then it can be flipped to satisfy delaunay
                    {
                        // TODO: RefineFailed for Minor? ResetClearance?
                        edge->RefineFailed = false;
                        ResetClearance(edge);

                        if (isMajor) {
                            V.TryAdd((IntPtr) edge->Org);
                            V.TryAdd((IntPtr) edge->Dest);
                            SetEdgeTypeMajor(edge, newReplacementEdgeType, edge->MajorEdge); // There are no more constraints attached so replace with Major Clearance edge

                            _flipStack.Push(edge);
                        } else {
                            if (edge->MajorEdge != null) { // There are no more constraints so Terrain is gone but could still be Minor to a Major Obstacle or Clearance
                                UnityEngine.Debug.Assert(edge->MajorEdge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance), $"edge->MajorEdge->EdgeType: {edge->MajorEdge->EdgeType}");
                                UnityEngine.Debug.Assert((edge->EdgeType & ~Edge.Type.Minor) == (edge->MajorEdge->EdgeType & ~Edge.Type.Major),
                                    $"edge->EdgeType: {edge->EdgeType}, edge->MajorEdge->EdgeType: {edge->MajorEdge->EdgeType}");

                                SetEdgeTypeMinor(edge, Edge.Type.Minor | (edge->MajorEdge->EdgeType & ~Edge.Type.Major), edge->MajorEdge);
                            } else { // If no Major, just replace with Minor TerrainSub
                                SetEdgeTypeMinor(edge, newReplacementEdgeType, null);
                            }
                            
                            _flipStackMinor.Push(edge);
                        }
                        byte minMaterialType = MinMaterialType(edge->MaterialTypeIndex, edge->Sym->MaterialTypeIndex);
                        SetTriangleMaterialTypeIndex(edge, minMaterialType);
                        SetTriangleMaterialTypeIndex(edge->Sym, minMaterialType);
                        FlipQuad(isMajor);
                    }
                    Edge.VerifyEdge(edge, isMajor);
                }
            }

            for (var i = 0; i < _vlist.Length; i++) {
                RemoveVertexIfEligible((Vertex*) _vlist[i], isMajor, i);
            }
        }

        void RemoveVertexIfEligible(Vertex* v, bool isMajor, int blah = 0)
        {
            UnityEngine.Debug.Log("RemoveVertexIfEligible");
            if (v->PointConstraints > 0 || v->ConstraintHandles > 0) { // If v is a point constraint or if v holds reference to a constraint then don't remove
                return;
            }

            if (!isMajor && v->GetEdge(true) != null) { // Can't remove Vertex from Minor if there are any Major edges attached. Major graph will take care of it
                return;
            }

            var amount = 0;
            var constrained = stackalloc Edge*[2];

            var e = v->GetEdgeEnumerator(isMajor); // Check if v connects to constraint edges
            while (e.MoveNext())
            {
                Edge.VerifyEdge(e.Current, isMajor);

                // if (e.Current->Constrained)
                if (e.Current->IsEdgeTypeConstrained())
                {
                    if (amount == 2) // If v connects to more than 2 constraints then we know we can't remove it so just return
                        return;
                    constrained[amount++] = e.Current;
                }
            }

            if (amount == 0) // If v not connected to any constraint edges, remove it, and fix the face affected
            {
                UnityEngine.Debug.Log("Remove in face");
                e = v->GetEdgeEnumerator(isMajor);
                byte minMaterialType = byte.MaxValue;
                while (e.MoveNext()) {
                    if (isMajor) {
                        V.TryAdd((IntPtr) e.Current->Dest);
                    }
                    minMaterialType = MinMaterialType(MinMaterialType(minMaterialType, e.Current->Sym->MaterialTypeIndex), e.Current->MaterialTypeIndex);
                }
                var face = RemoveVertex(v, isMajor);
                face->MaterialTypeIndex = minMaterialType; // 5
                // face->MaterialTypeIndex = MinMaterialType(face->LNext->MaterialTypeIndex, face->LPrev->MaterialTypeIndex); // TODO: Not sure if this will work

                RetriangulateFace(face, isMajor);
                return;
            }

            // If both constrained edges are the exact same (reference same obstacles) then continue
            // Third condition only applies to Minor graph; if vertex connects Minor Obstacle and Minor Terrain, don't Remove it
            if (amount != 2 || !constrained[0]->ConstraintsEqual(constrained[1]) || constrained[0]->EdgeType != constrained[1]->EdgeType)
                return;

            var e1 = constrained[0];
            var e2 = constrained[1];
            UnityEngine.Debug.Assert(e1->EdgeType == e2->EdgeType, $"e1->EdgeType: {e1->EdgeType}, e2->EdgeType: {e2->EdgeType}");
            UnityEngine.Debug.Assert(e1->ContainsMajorEdge(e2->MajorEdge) && e2->ContainsMajorEdge(e1->MajorEdge), $"e1->EdgeType: {e1->EdgeType}, e2->EdgeType: {e2->EdgeType}");
            Assert.IsTrue(e1->Dest != v && e2->Dest != v);
            Assert.IsTrue(e1->Dest != e2->Dest);
            var d1 = e1->Dest->Point;
            var d2 = e2->Dest->Point;
            var collinear = Math.TriArea(d1, d2, v->Point);

            if (collinear == 0) // If both constrained edges are exactly collinear, then remove the collinear vertex which connects them, and fix faces
            {                   // Not sure if we remove only one of the 2 edges because the other edge becomes the sole edge? Or maybe because RemoveVertex deals with the other one anyways? Probably the first one
                if (isMajor) {
                    e = v->GetEdgeEnumerator(isMajor);
                    while (e.MoveNext()) {
                        V.TryAdd((IntPtr) e.Current->Dest);
                    }
                }
                UnityEngine.Debug.LogError($"collinear == 0");
                UnityEngine.Debug.Assert(e1->ContainsMajorEdge(e2->MajorEdge) && e2->ContainsMajorEdge(e1->MajorEdge));
                UnityEngine.Debug.Assert(e1->MaterialTypeIndex == e2->Sym->MaterialTypeIndex && e1->Sym->MaterialTypeIndex == e2->MaterialTypeIndex, 
                    $"collinear - e1->MaterialTypeIndex (yellow): {e1->MaterialTypeIndex}, e2->MaterialTypeIndex (green): {e2->MaterialTypeIndex}, e1->Sym->MaterialTypeIndex (red): {e1->Sym->MaterialTypeIndex}, e2->Sym->MaterialTypeIndex (blue): {e2->Sym->MaterialTypeIndex}");
                var v1 = e1->Dest;
                var v2 = e2->Dest;
                var crep = e1->QuadEdge->Crep;
                byte sameMaterial1 = e1->MaterialTypeIndex;
                byte sameMaterial2 = e2->MaterialTypeIndex;
                Edge* sameMajorEdge = e1->MajorEdge;
                Edge.Type sameEdgeType = e1->EdgeType;
                RemoveEdge(e1, false, isMajor);
                RemoveVertex(v, isMajor);
                var e3 = Connect(v1, v2, sameEdgeType, sameMajorEdge);
                e3->MaterialTypeIndex = sameMaterial2;
                e3->Sym->MaterialTypeIndex = sameMaterial1;
                RetriangulateFace(e3, isMajor);
                RetriangulateFace(e3->Sym, isMajor);
                e3->QuadEdge->Crep = crep;
            }
            else // Then edges are not exactly collinear, but if they are almost (i.e. Semi) collinear then Remove v
            {
                var t = collinear / math.length(d2 - d1);

                if (collinear > 0)
                {
                    if (t < _collinearMargin && Math.TriArea(d1, d2, e1->DPrev->Org->Point) < 0 && Math.TriArea(d1, d2, e2->DNext->Org->Point) < 0)
                    {
                        if (isMajor) {
                            e = v->GetEdgeEnumerator(isMajor);
                            while (e.MoveNext()) {
                                V.TryAdd((IntPtr) e.Current->Dest);
                            }
                        }
                        UnityEngine.Debug.Log($"Collinearity: {t}");
                        RemoveSemiCollinear(v, e1, e2, isMajor, blah);
                    }
                }
                else if (t > -_collinearMargin && Math.TriArea(d1, d2, e1->DNext->Org->Point) > 0 && Math.TriArea(d1, d2, e2->DPrev->Org->Point) > 0)
                {
                    if (isMajor) {
                        e = v->GetEdgeEnumerator(isMajor);
                        while (e.MoveNext()) {
                            V.TryAdd((IntPtr) e.Current->Dest);
                        }
                    }
                    UnityEngine.Debug.Log($"Collinearity: {t}");
                    RemoveSemiCollinear(v, e1, e2, isMajor, blah);
                }
            }
        }

        void RemoveSemiCollinear(Vertex* v, Edge* e1, Edge* e2, bool isMajor, int blah = 0)
        {
            UnityEngine.Debug.Log($"RemoveSemiCollinear");
            UnityEngine.Debug.Assert(e1->ContainsMajorEdge(e2->MajorEdge) && e2->ContainsMajorEdge(e1->MajorEdge));
            UnityEngine.Debug.Assert(e1->MaterialTypeIndex == e2->Sym->MaterialTypeIndex && e1->Sym->MaterialTypeIndex == e2->MaterialTypeIndex, 
                $"e1->MaterialTypeIndex (yellow): {e1->MaterialTypeIndex}, e2->MaterialTypeIndex (green): {e2->MaterialTypeIndex}, e1->Sym->MaterialTypeIndex (red): {e1->Sym->MaterialTypeIndex}, e2->Sym->MaterialTypeIndex (blue): {e2->Sym->MaterialTypeIndex}");

            if (!(e1->MaterialTypeIndex == e2->Sym->MaterialTypeIndex && e1->Sym->MaterialTypeIndex == e2->MaterialTypeIndex)) {
                // CommonLib.DebugSeg(e1->Org->Point3D, e1->Dest->Point3D, UnityEngine.Color.yellow, 0.01f, math.INFINITY, 0.035f);
                // CommonLib.DebugSeg(e1->Sym->Org->Point3D, e1->Sym->Dest->Point3D, UnityEngine.Color.red, 0.01f, math.INFINITY, 0.035f);
                // CommonLib.DebugSeg(e2->Org->Point3D, e2->Dest->Point3D, UnityEngine.Color.green, 0.01f, math.INFINITY, 0.035f);
                // CommonLib.DebugSeg(e2->Sym->Org->Point3D, e2->Sym->Dest->Point3D, UnityEngine.Color.blue, 0.01f, math.INFINITY, 0.035f);
            }

            byte sameMaterial1 = e1->MaterialTypeIndex;
            byte sameMaterial2 = e2->MaterialTypeIndex;
            Edge* sameMajorEdge = e1->MajorEdge;
            Edge.Type sameEdgeType = e1->EdgeType;
            var crep = GetCrep(e1->QuadEdge->Crep);
            var a = e1->Dest;
            var b = e2->Dest;
            Edge.VerifyEdge(e1, isMajor);
            Edge.VerifyEdge(e2, isMajor);
            e1->QuadEdge->Crep.Clear();
            e2->QuadEdge->Crep.Clear();
            if (isMajor) { // TODO: Set the MajorEdges null here, since they could be flipped and kept and will no longer be in right place. Major edge should be set in InsertNoCross
                SetEdgeTypeMajor(e1, Edge.Type.Major | Edge.Type.Clearance, e1->MajorEdge); // TODO: Why are we doing this? I think because these edges could be flipped away and not removed
                SetEdgeTypeMajor(e2, Edge.Type.Major | Edge.Type.Clearance, e2->MajorEdge);
                _flipStack.Push(e1);
                _flipStack.Push(e2);
            } else {
                SetEdgeTypeMinor(e1, Edge.Type.Minor | Edge.Type.Ignore, null);
                SetEdgeTypeMinor(e2, Edge.Type.Minor | Edge.Type.Ignore, null);
                _flipStackMinor.Push(e1);
                _flipStackMinor.Push(e2);
            }
            // SetTriangleMaterialTypeIndex(e1, 6); // 0 instead
            // SetTriangleMaterialTypeIndex(e2, 6);
            // SetTriangleMaterialTypeIndex(e1->Sym, 6); // 0 instead
            // SetTriangleMaterialTypeIndex(e2->Sym, 6);

            Edge.VerifyEdge(e1, isMajor);
            Edge.VerifyEdge(e2, isMajor);
            FlipQuad(isMajor); // First satisfy the delaunay of collinear edges then remove the vertex and retriangulate, and finally reinsert the constraint
            var face1 = RemoveVertex(v, isMajor); // e1 and e2 may have been flipped away from v and therefore not removed
            // UnityEngine.Debug.Assert(e1->MaterialTypeIndex == face1->MaterialTypeIndex, $"e1->MaterialTypeIndex: {e1->MaterialTypeIndex}, face1->MaterialTypeIndex: {face1->MaterialTypeIndex}");

            RetriangulateFace(face1, isMajor);
            UnityEngine.Debug.Log($"blah: {blah}");
            // face1->MaterialTypeIndex = 0; //(byte)blah;
            // CommonLib.DebugSeg(face1->Org->Point3D, face1->Dest->Point3D, UnityEngine.Color.blue, 0.01f, math.INFINITY, 0.035f);
            InsertSegmentNoCrossConstraints(a, b, crep, sameEdgeType, sameMajorEdge, sameMaterial2, sameMaterial1, blah);
        }

        void InsertSegmentNoCrossConstraints(Vertex* a, Vertex* b, UnsafeList<Entity> crep, Edge.Type newConstraintEdgeType, Edge* majorEdge, byte material1, byte material2, int blah = 0)
        {
            //UnityEngine.Debug.Log("InsertSegmentNoCrossConstraints crep +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            bool isMajor = Edge.IsEdgeTypeMajor(newConstraintEdgeType);
            var c = GetConnection(a, b, isMajor);

            if (c != null)
            {
                UnityEngine.Debug.Log($"InsertSegmentNoCrossConstraints crep - Found connection");
                if (isMajor) {
                    C.TryAdd((IntPtr) c);
                }
                c->QuadEdge->Crep = crep;

                    byte material = MinMaterialType(c->LNext->MaterialTypeIndex, c->LPrev->MaterialTypeIndex);
                    byte symMaterial = MinMaterialType(c->Sym->LNext->MaterialTypeIndex, c->Sym->LPrev->MaterialTypeIndex);
                    // UnityEngine.Debug.Assert(material == material1, $"material: {material}, material1: {material1}");
                    // UnityEngine.Debug.Assert(symMaterial == material2, $"symMaterial: {symMaterial}, material2: {material2}");

                // SetTriangleMaterialTypeIndex(c, /* material); // */material1);
                // SetTriangleMaterialTypeIndex(c->Sym, /* symMaterial); // */material2);

                BFSEdgesSetMaterialType(c, material1 /* 2 */, Entity.Null, true, blah);
                BFSEdgesSetMaterialType(c->Sym, material2 /* 4 */, Entity.Null, true, blah);

                ResetClearance(c);
                OverwriteEdgeType(c, newConstraintEdgeType, majorEdge);
                return;
            }

            var e = GetLeftEdge(a, b->Point, isMajor);

            InfiniteLoopDetection.Reset();
            while (e->Dest != b)
            {
                InfiniteLoopDetection.Register(1000, "InsertSegmentNoCrossConstraints");

                var d = Math.TriArea(a->Point, b->Point, e->Dest->Point);
                var next = d > 0 ? e->RPrev : e->ONext;

                if (d < 0)
                {
                    Assert.IsTrue(!e->IsConstrained);
                    UnityEngine.Debug.Log($"InsertSegmentNoCrossConstraints crep - RemoveEdge");
                    RemoveEdge(e, true, isMajor);
                }
                else if (d == 0 && e->Dest != a)
                {
                    UnityEngine.Debug.Log($"InsertSegmentNoCrossConstraints crep - Connect 1");
                    var t = e->Dest;
                    Connect(a, t, GetCrep(crep), newConstraintEdgeType, majorEdge, material1, material2);
                    a = t;
                }

                e = next;
            }

            UnityEngine.Debug.Log($"InsertSegmentNoCrossConstraints crep - Connect 2");
            Connect(a, b, crep, newConstraintEdgeType, majorEdge, material1, material2);
        }

        void FlipQuad(bool isMajor)
        {
            //UnityEngine.Debug.Log("FlipQuad");
            ref Collections.PtrStack<Edge> flipStackUsing = ref isMajor ? ref _flipStack : ref _flipStackMinor;
            while (flipStackUsing.Count > 0)
            {
                var edge = flipStackUsing.Pop();

                Edge.VerifyEdge(edge, isMajor);

                if (/* !edge->Constrained */ !edge->IsEdgeTypeConstrained() && Math.CircumcircleContains(edge->Org->Point, edge->Dest->Point, edge->ONext->Dest->Point, edge->DNext->Org->Point))
                {
                    flipStackUsing.Push(edge->OPrev);
                    flipStackUsing.Push(edge->DNext);
                    flipStackUsing.Push(edge->Sym->OPrev);
                    flipStackUsing.Push(edge->Sym->DNext);
                    Swap(edge);
                }
            }
        }

        Edge* CreateEdge(Vertex* a, Vertex* b, Edge.Type newEdgeType, Edge* majorEdge)
        {
            //UnityEngine.Debug.Log("CreateEdge");
            var q = _quadEdges.GetElementPointer(new QuadEdge {Crep = GetCrep(), Id = NextEdgeId, EdgeType = newEdgeType});

            q->Edge0 = new Edge(q, 0);
            q->Edge1 = new Edge(q, 1);
            q->Edge2 = new Edge(q, 2);
            q->Edge3 = new Edge(q, 3);

            q->Edge0.Next = &q->Edge0;
            q->Edge1.Next = &q->Edge3;
            q->Edge2.Next = &q->Edge2;
            q->Edge3.Next = &q->Edge1;

            if (Edge.IsEdgeTypeMajor(newEdgeType)) {
                UnityEngine.Debug.Assert(!AddedOrModifiedMajorEdges.Contains((IntPtr)(&q->Edge1)) && !AddedOrModifiedMajorEdges.Contains((IntPtr)(&q->Edge2)) && !AddedOrModifiedMajorEdges.Contains((IntPtr)(&q->Edge3)));
                AddedOrModifiedMajorEdges.TryAdd((IntPtr)(&q->Edge0));
            }

            SetEndPoints(&q->Edge0, a, b, Edge.IsEdgeTypeMajor(newEdgeType));

            q->MajorEdge = majorEdge;

            return &q->Edge0;
        }

        void RemoveEdge(Edge* e, bool recycleCrep, bool isMajor)
        {
            //UnityEngine.Debug.Log("RemoveEdge");
            DestroyedTriangle(e->TriangleId); // Add both triangles to Destroyed list so that we can later check to see if a previously created path needs to be invalidated
            DestroyedTriangle(e->Sym->TriangleId);

            Edge.VerifyEdge(e, isMajor);

            UnityEngine.Debug.Assert(e->Org->ContainsEdge(e, isMajor) && e->Dest->ContainsEdge(e->Sym, isMajor));

            if (isMajor) {
                RemoveMajorInMinor(e);
                AddedOrModifiedMajorEdges.Remove((IntPtr) e);
                AddedOrModifiedMajorEdges.Remove((IntPtr) e->Sym);
            }

            e->Org->RemoveEdge(e, isMajor); // If vertex points to edge as head, then remove it
            e->Dest->RemoveEdge(e->Sym, isMajor);
            Splice(e, e->OPrev); // I think this is merging edges
            Splice(e->Sym, e->Sym->OPrev);

            var qe = e->QuadEdge; // Remove the associated QuadEdge and maybe recycle its crep
            if (recycleCrep)
            {
                qe->Crep.Clear();
                _creps.Push(qe->Crep);
            }
            qe->Delete();

            _quadEdges.Recycle(qe);
        }

        void DestroyedTriangle(int tri)
        {
            DestroyedTriangles.TryAdd(tri);
        }

        Vertex* CreateVertex(float2 p)
        {
            //UnityEngine.Debug.Log("CreateVertex");
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
            Edge.VerifyEdge(a);
            Edge.VerifyEdge(b);
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

        void NewTriangle(Edge* e, byte materialTypeIndex) // Just inits TriangleIds
        {
            var tid = NextTriangleId;
            e->TriangleId = tid;
            e->LNext->TriangleId = tid;
            e->LPrev->TriangleId = tid;

            SetTriangleMaterialTypeIndex(e, materialTypeIndex);
        }

        void SetTriangleMaterialTypeIndex(Edge* e, byte materialTypeIndex) {
            e->MaterialTypeIndex = materialTypeIndex;
            e->LNext->MaterialTypeIndex = materialTypeIndex;
            e->LPrev->MaterialTypeIndex = materialTypeIndex;
        }
    }
}
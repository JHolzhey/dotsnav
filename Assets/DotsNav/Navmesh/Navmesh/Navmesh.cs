using System;
using System.Diagnostics;
using DotsNav.Collections;
using DotsNav.Navmesh.Data;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace DotsNav.Navmesh
{
    /// <summary>
    /// Provides access to edges and vertices in the triangulation
    /// </summary>
    public unsafe partial struct Navmesh
    {
        const int CrepMinCapacity = 1; // todo what's a good number, shrink when reusing?

        public float2 Extent;
        public int Vertices => _verticesSeq.Length;

        float _e;
        float _collinearMargin;

        UnsafeParallelHashMap<Entity, IntPtr> _constraints;
        BlockPool<Vertex> _vertices;
        BlockPool<QuadEdge> _quadEdges;
        internal UnsafeList<IntPtr> _verticesSeq; // Accessible for debug

        internal HashSet<IntPtr> V; // All vertices adjacent to modified edges
        internal HashSet<IntPtr> C; // All edges constrained during insertion

        QuadTree _qt;
        EdgeSearch _edgeSearch;
        PtrStack<Edge> _flipStack;
        PtrStack<Edge> _flipStackMinor;
        UnsafeList<Point> _insertedPoints;
        PtrStack<Vertex> _openStack;
        UnsafePtrQueue<Vertex> _openQueue;
        UnsafeList<IntPtr> _vlist;
        UnsafeList<IntPtr> _elist;
        UnsafeList<IntPtr> _vlistMinor;
        UnsafeList<IntPtr> _elistMinor;
        Stack<UnsafeList<Entity>> _creps;
        internal HashSet<int> DestroyedTriangles;
        Deque<IntPtr> _refinementQueue;

        internal HashSet<IntPtr> AddedOrModifiedMajorEdges;

        internal readonly static Entity MinorObstacleCid = new Entity{Index = int.MinValue, Version = int.MinValue};

        int _mark;
        int _edgeId;
        int _triangleId;

        int NextMark => ++_mark;
        int NextEdgeId => ++_edgeId;
        int NextTriangleId => ++_triangleId;
        internal bool IsEmpty => Vertices == 8;

        internal Navmesh(NavmeshComponent component)
        {
            Assert.IsTrue(math.all(component.Size > 0));
            Assert.IsTrue(component.ExpectedVerts > 0);

            Extent = component.Size / 2;
            _e = component.MergePointsDistance;
            _collinearMargin = component.CollinearMargin;

            const int blockSize = 128;
            var initialBlocks = (int) math.ceil((float) component.ExpectedVerts / blockSize);
            _vertices = new BlockPool<Vertex>(blockSize, initialBlocks, Allocator.Persistent);
            _verticesSeq = new UnsafeList<IntPtr>(component.ExpectedVerts, Allocator.Persistent);
            _quadEdges = new BlockPool<QuadEdge>(3 * blockSize, initialBlocks, Allocator.Persistent);
            _constraints = new UnsafeParallelHashMap<Entity, IntPtr>(component.ExpectedVerts, Allocator.Persistent);
            V = new HashSet<IntPtr>(16, Allocator.Persistent);
            C = new HashSet<IntPtr>(16, Allocator.Persistent);
            _edgeSearch = new EdgeSearch(100, 100, Allocator.Persistent);
            _qt = new QuadTree(math.max(component.Size.x, component.Size.y), 100, 10, Allocator.Persistent);
            _flipStack = new PtrStack<Edge>(32, Allocator.Persistent);
            _flipStackMinor = new PtrStack<Edge>(32, Allocator.Persistent);
            _insertedPoints = new UnsafeList<Point>(64, Allocator.Persistent);
            _openStack = new PtrStack<Vertex>(64, Allocator.Persistent);
            _openQueue = new UnsafePtrQueue<Vertex>(10, Allocator.Persistent);
            _vlist = new UnsafeList<IntPtr>(64, Allocator.Persistent);
            _elist = new UnsafeList<IntPtr>(64, Allocator.Persistent);
            _vlistMinor = new UnsafeList<IntPtr>(64, Allocator.Persistent);
            _elistMinor = new UnsafeList<IntPtr>(64, Allocator.Persistent);
            _creps = new Stack<UnsafeList<Entity>>(2*component.ExpectedVerts, Allocator.Persistent);
            for (int i = 0; i < 2 * component.ExpectedVerts; i++)
                _creps.Push(new UnsafeList<Entity>(CrepMinCapacity, Allocator.Persistent));
            DestroyedTriangles = new HashSet<int>(64, Allocator.Persistent);
            _refinementQueue = new Deque<IntPtr>(24, Allocator.Persistent);

            AddedOrModifiedMajorEdges = new HashSet<IntPtr>(64, Allocator.Persistent);

            _mark = default;
            _edgeId = default;
            _triangleId = default;

            BuildBoundingBoxes();
        }

        void BuildBoundingBoxes()
        {
            // Setup and Initialize Navmesh bounds, must do it manually because Insert will not work with an empty plane
            var bmin = -Extent - 1;
            var bmax = Extent + 1;

            var bottomLeft = CreateVertex(bmin);
            var bottomRight = CreateVertex(new float2(bmax.x, bmin.y));
            var topRight = CreateVertex(bmax);
            var topleft = CreateVertex(new float2(bmin.x, bmax.y));

            var bottom = CreateEdge(bottomLeft, bottomRight, Edge.Type.Major | Edge.Type.Obstacle, null);
            var right = CreateEdge(bottomRight, topRight, Edge.Type.Major | Edge.Type.Obstacle, null);
            var top = CreateEdge(topRight, topleft, Edge.Type.Major | Edge.Type.Obstacle, null);
            var left = CreateEdge(topleft, bottomLeft, Edge.Type.Major | Edge.Type.Obstacle, null);

            bottom->AddConstraint(Entity.Null);
            right->AddConstraint(Entity.Null);
            top->AddConstraint(Entity.Null);
            left->AddConstraint(Entity.Null);

            Splice(bottom->Sym, right);
            Splice(right->Sym, top);
            Splice(top->Sym, left);
            Splice(left->Sym, bottom);

            Connect(right, bottom, Edge.Type.Major | Edge.Type.Clearance, null);

            

            var bottomMinor = CreateEdge(bottomLeft, bottomRight, Edge.Type.Minor | Edge.Type.Obstacle, bottom);
            var rightMinor = CreateEdge(bottomRight, topRight, Edge.Type.Minor | Edge.Type.Obstacle, right);
            var topMinor = CreateEdge(topRight, topleft, Edge.Type.Minor | Edge.Type.Obstacle, top);
            var leftMinor = CreateEdge(topleft, bottomLeft, Edge.Type.Minor | Edge.Type.Obstacle, left);

            bottomMinor->AddConstraint(MinorObstacleCid); // Could also just not Add a Constraint
            rightMinor->AddConstraint(MinorObstacleCid);
            topMinor->AddConstraint(MinorObstacleCid);
            leftMinor->AddConstraint(MinorObstacleCid);

            Splice(bottomMinor->Sym, rightMinor);
            Splice(rightMinor->Sym, topMinor);
            Splice(topMinor->Sym, leftMinor);
            Splice(leftMinor->Sym, bottomMinor);

            Connect(rightMinor, bottomMinor, Edge.Type.Minor | Edge.Type.Ignore, null);



            AddedOrModifiedMajorEdges.Clear();

            UnityEngine.Debug.Log($"Starting insert: =======================================================================================");

            var bounds = stackalloc float2[] {-Extent, new float2(Extent.x, -Extent.y), Extent, new float2(-Extent.x, Extent.y), -Extent};
            InsertMajor(bounds, 0, 5, Entity.Null, float4x4.identity, Edge.Type.Obstacle);

            UnityEngine.Debug.Log($"_modifiedMajorEdges.Length: {AddedOrModifiedMajorEdges.Length} =======================================================================================");
            foreach (IntPtr e in AddedOrModifiedMajorEdges) {
                Edge* edge = (Edge*)e;
                if (!Contains(edge->Org->Point) || !Contains(edge->Dest->Point)) {
                    continue;
                }
                InsertMajorInMinor(edge);
            }

            AddedOrModifiedMajorEdges.Clear();
        }

        internal void Dispose()
        {
            var e = GetEdgeEnumerator(true);
            while (e.MoveNext())
                e.Current->QuadEdge->Crep.Dispose();

            var eMinor = GetEdgeEnumerator(false);
            while (eMinor.MoveNext())
                eMinor.Current->QuadEdge->Crep.Dispose();

            for (int i = 0; i < _creps.Count; i++)
                _creps[i].Dispose();
            _creps.Dispose();

            _vertices.Dispose();
            _verticesSeq.Dispose();
            _constraints.Dispose();
            _quadEdges.Dispose();
            V.Dispose();
            C.Dispose();
            _qt.Dispose();
            _edgeSearch.Dispose();
            _flipStack.Dispose();
            _insertedPoints.Dispose();
            _openStack.Dispose();
            _openQueue.Dispose();
            _vlist.Dispose();
            _elist.Dispose();
            _vlistMinor.Dispose();
            _elistMinor.Dispose();
            DestroyedTriangles.Dispose();
            _refinementQueue.Dispose();
            AddedOrModifiedMajorEdges.Dispose();
        }

        public bool Contains(float2 p) => Math.Contains(p, -Extent, Extent);

        /// <summary>
        /// Allows enumeration of all edges in the navmesh
        /// </summary>
        /// <param name="sym">Set to true to enumerate symetric edges, i.e. enumerate edge(x,y) and edge(y,x)</param>
        public EdgeEnumerator GetEdgeEnumerator(bool isMajor, bool sym = false) => new(_verticesSeq, Extent, isMajor, sym);
    }
}
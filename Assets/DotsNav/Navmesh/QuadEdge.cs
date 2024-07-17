using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Debug = UnityEngine.Debug;

namespace DotsNav.Navmesh
{
    /// <summary>
    /// Represents an undirected edge in the navmesh. For more information see:
    /// <para>Primitives for the Manipulation of General Subdivisions and the Computation of Voronoi Diagrams, Guibas and Stolfi (1985).</para>
    /// <para>http://sccg.sk/~samuelcik/dgs/quad_edge.pdf</para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    struct QuadEdge
    {
        public Edge Edge0;
        public Edge Edge1;
        public Edge Edge2;
        public Edge Edge3;
        public UnsafeList<Entity> Crep;
        public int Mark;
        public int Id;
        public bool RefineFailed;
        
        unsafe Edge* _majorEdge;
        public unsafe Edge* MajorEdge {
            get => _majorEdge;
            set {
                _majorEdge = value != null ? (MathLib.IsSameDir(Edge0.SegVector, value->SegVector) ? value : value->Sym) : null; // Primary edge and MajorEdge face the same direction for faster lookup
                VerifyMajorEdge();
            }
        }

        [System.Diagnostics.Conditional("UNITY_ASSERTIONS")]
        public unsafe void VerifyMajorEdge() {
            if (_majorEdge != null) {
                Debug.Assert(MathLib.LogicalIf(math.length(Edge0.SegVector) > 0.1f, MathLib.IsParallel(Edge0.SegVector.XOY(), _majorEdge->SegVector.XOY(), 0.001f)),
                    $"Major edge is not parallel, edgeLength: {math.length(Edge0.SegVector)}");

                if (!Edge0.EdgeType.IsOverwritten()) {
                    Debug.Assert(MathLib.IsSameDir(Edge0.SegVector, _majorEdge->SegVector));
                    Debug.Assert(MathLib.IsSameDir(Edge0.Sym->SegVector, _majorEdge->Sym->SegVector));
                }

                Debug.Assert(!Edge0.EdgeType.IsMajor() && _majorEdge->EdgeType.IsMajor(), $"Edge0.EdgeType: {Edge0.EdgeType}, _majorEdge->EdgeType: {_majorEdge->EdgeType}");
                Debug.Assert(Edge0.MainEdgeType == _majorEdge->MainEdgeType, $"Edge0.EdgeType: {Edge0.EdgeType}, _majorEdge->EdgeType: {_majorEdge->EdgeType}");
                Debug.Assert(_majorEdge->Org != null && _majorEdge->Dest != null, $"Org: {_majorEdge->Org != null}, Dest: {_majorEdge->Dest != null}");
            }
        }

        public unsafe void Delete() { // Debug
            Edge0.Org = null;
            Edge1.Org = null;
            Edge2.Org = null;
            Edge3.Org = null;
            Edge0.TriangleId = -10;
            Edge1.TriangleId = -10;
            Edge2.TriangleId = -10;
            Edge3.TriangleId = -10;
            Edge0.TriangleMaterial = byte.MaxValue;
            Edge1.TriangleMaterial = byte.MaxValue;
            Edge2.TriangleMaterial = byte.MaxValue;
            Edge3.TriangleMaterial = byte.MaxValue;
            Edge0.TriangleSlopeCost = -1;
            Edge1.TriangleSlopeCost = -1;
            Edge2.TriangleSlopeCost = -1;
            Edge3.TriangleSlopeCost = -1;
        }

        public Edge.Type EdgeType;
    }
}
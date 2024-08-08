using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Debug = UnityEngine.Debug;

namespace DotsNav.Navmesh
{
    /// <summary>
    /// A vertex in the triangulation. Updating the navmesh invalidates this structure.
    /// </summary>
    public unsafe struct Vertex : IRefable
    {
        [System.Flags]
        public enum Type : byte {
            None = 0,
            OnTerrain = 1,
            Major = 1 << 4, // Casted to and from Edge.Major / Edge.Minor
            Minor = 1 << 5,
        }

        /// <summary>
        /// Returns the position of this vertex
        /// </summary>
        public float2 Point { get; internal set; }
        public float Height { get; internal set; }
        public readonly float3 Point3D => Point.XOY(Height);
        
        // public float Height { get; internal set; }

        Edge* EdgeMajor;
        Edge* EdgeMinor;
        public Type VertexType {
            get {
                Type thisType = Type.None;
                thisType.SetFlagsB(Type.Major, EdgeMajor != null);
                thisType.SetFlagsB(Type.Minor, EdgeMinor != null);
                // if (EdgeMajor != null) { thisType |= Type.Major; }
                // if (EdgeMinor != null) { thisType |= Type.Minor; }
                return thisType;
            }
        }

        internal Edge* GetEdge(bool isMajor) => isMajor ? EdgeMajor : EdgeMinor;
        internal int SeqPos;
        internal int Mark; // Used when potentially removing Constraint Vertices (depth-first search), to mark already visited Vertices // TODO: I don't think this is true
        internal int PointConstraints;
        internal int ConstraintHandles;

        public Vertex(float2 point, int seqPos) : this() {
            Point = point;
            SeqPos = seqPos;
            Height = 0f;
        }

        /// <summary>
        /// Allows for the enumeration of all edges that share this vertex as their origin:
        /// <para>for (var e = v.GetEdgeEnumerator(); e.Current != null; e.MoveNext())</para>
        /// </summary>
        public EdgeEnumerator GetEdgeEnumerator(bool isMajor = true)
        {
            fixed (Vertex* p = &this)
                return new EdgeEnumerator(p, isMajor);
        }

        public bool ContainsEdge(Edge* e, bool isMajor) {
            EdgeEnumerator edgeEnumerator = GetEdgeEnumerator(isMajor);
            while (edgeEnumerator.MoveNext()) {
                if (edgeEnumerator.Current == e) {
                    return true;
                }
            }
            return false;
        }

        internal void RemoveEdge(Edge* e, bool isMajor) {
            Edge.VerifyEdge(e, isMajor);
            if (isMajor) {
                EdgeMajor = e->ONext == e ? null : e->ONext;
            } else {
                EdgeMinor = e->ONext == e ? null : e->ONext;
            }
        }
        internal void AddEdge(Edge* e, bool isMajor) {
            Edge.VerifyEdgeType(e->EdgeType, isMajor);
            if (isMajor) {
                EdgeMajor = e;
            } else {
                EdgeMinor = e;
            }
        }

        public override string ToString() =>
            $"{Point.x:F}, {Point.y:F}";

        public bool IsValid() {
            return true;
        }

        /// <summary>
        /// Allows for enumerating all edges that share this vertex as their origin. Updating the navmesh invalidates this structure.
        /// </summary>
        public struct EdgeEnumerator
        {
            /// <summary>
            /// Current edge being enumerated. Null when all edges have been enumerated.
            /// </summary>
            public Edge* Current { get; private set; }
            readonly Edge* _start;
            bool _started;

            bool _debugIsMajor;

            internal EdgeEnumerator(Vertex* v, bool isMajor)
            {
                Assert.IsTrue(v != null);
                _debugIsMajor = isMajor;
                _start = v->GetEdge(isMajor);
                _started = false;
                Current = null;
            }

            public bool MoveNext()
            {
                if (!_started)
                {
                    Current = _start;
                    _started = true;
                    return Current != null;
                }

                
                if (Current != null)
                {
                    Edge.VerifyEdgeType(Current->EdgeType, _debugIsMajor);

                    Current = Current->ONext;
                    
                    if (Current != _start)
                        return true;

                    Current = null;
                }

                return false;
            }
        }
    }
}
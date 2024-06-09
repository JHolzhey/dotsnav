using System.Diagnostics;
using Unity.Mathematics;

namespace DotsNav.Navmesh
{
    /// <summary>
    /// A vertex in the triangulation. Updating the navmesh invalidates this structure.
    /// </summary>
    public unsafe struct Vertex
    {
        [System.Flags]
        public enum Type : byte {
            None = 0,
            Major = 1, // Casted to and from Edge.Major / Edge.Minor
            Minor = 1 << 1,
        }
        /// <summary>
        /// Returns the position of this vertex
        /// </summary>
        public float2 Point { get; internal set; }

        Edge* MajorEdge;
        Edge* MinorEdge;
        public Type VertexType { // TODO: Make this branchless somehow
            get {
                Type thisType = Type.None;
                if (MajorEdge != null) { thisType |= Type.Major; }
                if (MinorEdge != null) { thisType |= Type.Minor; }
                return thisType;
            }
        }


        internal Edge* GetEdge(bool isMajor) => isMajor ? MajorEdge : MinorEdge;
        internal int SeqPos;
        internal int Mark; // Used when potentially removing Constraint Vertices (depth-first search), to mark already visited Vertices // TODO: I don't think this is true
        internal int PointConstraints;
        internal int ConstraintHandles;

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
            DotsNav.Navmesh.Edge.VerifyEdge(e, isMajor);
            if (isMajor) {
                MajorEdge = e->ONext == e ? null : e->ONext;
            } else {
                MinorEdge = e->ONext == e ? null : e->ONext;
            }
        }
        internal void AddEdge(Edge* e, bool isMajor) {
            DotsNav.Navmesh.Edge.VerifyEdgeType(e->EdgeType, isMajor);
            if (isMajor) {
                MajorEdge = e;
            } else {
                MinorEdge = e;
            }
        }

        public override string ToString() =>
            $"{Point.x:F}, {Point.y:F}";

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

            internal EdgeEnumerator(Vertex* v, bool isMajor = true)
            {
                Assert.IsTrue(v != null);
                _debugIsMajor = isMajor;
                _start = isMajor ? v->MajorEdge : v->MinorEdge;
                _started = false;
                Current = null;
            }

            public bool MoveNext()
            {
                if (!_started)
                {
                    Current = _start;
                    _started = true;
                    return true; // TODO: Just do Current != null
                }

                
                if (Current != null)
                {
                    DotsNav.Navmesh.Edge.VerifyEdgeType(Current->EdgeType, _debugIsMajor);

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
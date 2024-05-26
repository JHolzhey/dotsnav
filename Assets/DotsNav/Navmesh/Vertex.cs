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
        public enum Type {
            None = 0,
            Major = 1,
            Minor = 1 << 1,
        }
        /// <summary>
        /// Returns the position of this vertex
        /// </summary>
        public float2 Point { get; internal set; }

        Edge* Edge;
        public Type type {
            get {
                Type thisType = Type.None;
                if (Edge != null) { thisType |= Type.Major; }
                if (MinorEdge != null) { thisType |= Type.Minor; }
                return thisType;
            }
        }
        Edge* MinorEdge;

        // internal void HackyDisconnectMinorEdge() => MinorEdge = null;


        internal Edge* GetEdge(bool isMajor) => isMajor ? Edge : MinorEdge;
        internal int SeqPos;
        internal int Mark; // Used when potentially removing Constraint Vertices (depth-first search), to mark already visited Vertices
        internal int PointConstraints;
        internal int ConstraintHandles;

        public bool IsSpecial;

        public static unsafe bool IsEdgeTypeMajor(EdgeType edgetype) {
            if (edgetype == EdgeType.Obstacle || edgetype == EdgeType.ConnectsToObstacle || edgetype == EdgeType.ConnectsToObstacleWithMinorTerrainConnects) {
                return true;
            } else if (edgetype == EdgeType.Terrain || edgetype == EdgeType.ConnectsToTerrain || edgetype == EdgeType.ConnectsToTerrainWithMajorConnectsToObstacle
                || edgetype == EdgeType.Ignore) {
                return false;
            } else { UnityEngine.Debug.Assert(false); return true; }
        }

        public static unsafe void VerifyEdgeType(Edge* edge, bool isMajor) {
            UnityEngine.Debug.Assert((isMajor && (edge->EdgeType == EdgeType.Obstacle || edge->EdgeType == EdgeType.ConnectsToObstacle || edge->EdgeType == EdgeType.ConnectsToObstacleWithMinorTerrainConnects))
                || (!isMajor && (edge->EdgeType == EdgeType.Terrain || edge->EdgeType == EdgeType.ConnectsToTerrain || edge->EdgeType == EdgeType.ConnectsToTerrainWithMajorConnectsToObstacle))
                , $"isMajor: {isMajor}, EdgeType: {edge->EdgeType}");
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

        internal void RemoveEdge(Edge* e, bool isMajor) {
            VerifyEdgeType(e, isMajor);
            if (isMajor) {
                Edge = e->ONext == e ? null : e->ONext;
            } else {
                MinorEdge = e->ONext == e ? null : e->ONext;
            }
        }
        internal void AddEdge(Edge* e, bool isMajor) {
            VerifyEdgeType(e, isMajor);
            if (isMajor) {
                Edge = e;
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
                _start = isMajor ? v->Edge : v->MinorEdge;
                _started = false;
                Current = null;
            }

            public bool MoveNext()
            {
                if (!_started)
                {
                    Current = _start;
                    _started = true;
                    return true;
                }

                
                if (Current != null)
                {
                    VerifyEdgeType(Current, _debugIsMajor);

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
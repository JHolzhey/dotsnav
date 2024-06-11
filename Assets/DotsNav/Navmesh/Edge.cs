using Unity.Entities;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEditor;

namespace DotsNav.Navmesh
{
    /// <summary>
    /// A directed edge in the triangulation. Traversal operators assume counter clockwise rotation,
    /// e.g. ONext will return the first edge in counter clockwise order around the origin, and OPrev will return the first edge in clockwise order.
    /// Updating the navmesh invalidates this structure.
    /// </summary>
    public unsafe struct Edge
    {
        [System.Flags]
        public enum Type : byte {
            None = 0,
            Major = 1, // Casted to and from Vertex.Major / Vertex.Minor
            Minor = 1 << 1,

            Obstacle = 1 << 2,
            Clearance = 1 << 3,

            Terrain = 1 << 4,
            Ignore = 1 << 5, // Consider renaming to Ignore
        }

        public static readonly Dictionary<Type, UnityEngine.Color> EdgeColors = new Dictionary<Type, UnityEngine.Color> {
            { Type.Major | Type.Obstacle, UnityEngine.Color.red },
            { Type.Major | Type.Clearance, UnityEngine.Color.magenta },

            { Type.Minor | Type.Obstacle, UnityEngine.Color.blue },
            { Type.Minor | Type.Clearance, UnityEngine.Color.yellow },
            { Type.Minor | Type.Terrain, UnityEngine.Color.green },
            { Type.Minor | Type.Ignore, UnityEngine.Color.cyan },
        };

        public Type EdgeType => QuadEdge->EdgeType;
        internal void SetEdgeType(Type fixedEdgeType) {
            QuadEdge->EdgeType = fixedEdgeType;
        }

        public bool IsObstacle => EdgeType.HasAllFlagsB(Type.Obstacle);
        public bool IsClearance => EdgeType.HasAllFlagsB(Type.Clearance);
        public bool IsTerrain => EdgeType.HasAllFlagsB(Type.Terrain);

        public unsafe bool IsEdgeTypeConstrained() {
            VerifyEdgeType(EdgeType);
            return (EdgeType.HasAllFlagsB(Type.Major) && EdgeType.HasAnyFlagsB(Type.Obstacle))
                || (EdgeType.HasAllFlagsB(Type.Minor) && EdgeType.HasAnyFlagsB(Type.Terrain | Type.Obstacle | Type.Clearance));
        }

        public static unsafe bool IsEdgeTypeMajor(Type edgeType) {
            UnityEngine.Debug.Assert((edgeType.HasAnyFlagsB(Type.Major) && edgeType.HasNoFlagsB(Type.Minor))
                || (edgeType.HasNoFlagsB(Type.Major) && edgeType.HasAnyFlagsB(Type.Minor)), $"EdgeType: {edgeType}"); // Edge type must be either Major or Minor, cannot be both
            return edgeType.HasAnyFlagsB(Type.Major);
        }

        // TODO: Make conditional
        public static unsafe void VerifyEdgeType(Type edgeType) => VerifyEdgeType(edgeType, IsEdgeTypeMajor(edgeType));
        public static unsafe void VerifyEdgeType(Type edgeType, bool isMajor) {
            UnityEngine.Debug.Assert((isMajor && edgeType.HasAnyFlagsB(Type.Major) && edgeType.HasNoFlagsB(Type.Minor) && edgeType.HasAnyFlagsB(Type.Obstacle | Type.Clearance))
                || (!isMajor && edgeType.HasAnyFlagsB(Type.Minor) && edgeType.HasNoFlagsB(Type.Major) && edgeType.HasAnyFlagsB(Type.Obstacle | Type.Clearance | Type.Terrain | Type.Ignore))
                , $"isMajor: {isMajor}, edgeType: {edgeType}");
        }

        public static unsafe void VerifyEdge(Edge* e) {
            if (IsEdgeTypeMajor(e->EdgeType)) {
                UnityEngine.Debug.Assert(MathLib.LogicalIff(e->EdgeType.HasAnyFlagsB(Type.Obstacle), e->Constrained), $"e->EdgeType: {e->EdgeType}, e->Constrained: {e->Constrained}");
            } else {
                UnityEngine.Debug.Assert(MathLib.LogicalIff(e->EdgeType.HasAnyFlagsB(Type.Terrain), e->Constrained), $"e->EdgeType: {e->EdgeType}, e->Constrained: {e->Constrained}");

                if (e->EdgeType.HasAnyFlagsB(Type.Obstacle | Type.Clearance)) {
                    UnityEngine.Debug.Assert(e->MajorEdge != null, "Minor Obstaces and Clearances must have Major edges");
                    if (e->MajorEdge != null) { // This is just to prevent null exception
                        UnityEngine.Debug.Assert((e->EdgeType & ~ Type.Minor) == (e->MajorEdge->EdgeType & ~Type.Major), $"e->EdgeType: {e->EdgeType}, e->MajorEdge->EdgeType: {e->MajorEdge->EdgeType}");
                    }
                }
            }
            if (e->MajorEdge != null) {
                UnityEngine.Debug.Assert((e->EdgeType & ~ Type.Minor) == (e->MajorEdge->EdgeType & ~Type.Major), $"e->EdgeType: {e->EdgeType}, e->MajorEdge->EdgeType: {e->MajorEdge->EdgeType}");
            }
        }
        public static unsafe void VerifyEdge(Edge* e, bool isMajor) {
            VerifyEdgeType(e->EdgeType, isMajor);
            VerifyEdge(e);
        }

        internal readonly QuadEdge* QuadEdge;
        internal Edge* Next;
        readonly int _indexInQuadEdge;
        float _clearanceLeft;
        float _clearanceRight;

        internal Edge(QuadEdge* quadEdge, int indexInQuadEdge) : this()
        {
            QuadEdge = quadEdge;
            _indexInQuadEdge = indexInQuadEdge;
            _clearanceLeft = -1;
            _clearanceRight = -1;
        }

        /// <summary>
        /// Returns the origin vertex.
        /// </summary>
        public Vertex* Org { get; internal set; }

        /// <summary>
        /// Returns the destination vertex.
        /// </summary>
        public Vertex* Dest => Sym->Org;

        /// <summary>
        /// Returns a value unique to this edge's quadedge, i.e. this value will be the same for edge(x,y) and edge(y,x).
        /// This value is not stable and can change when updating the navmesh.
        /// </summary>
        public int QuadEdgeId => QuadEdge->Id;

        /// <summary>
        /// Returns a stricly increasing value unique to this edge's left face, i.e. this value will be the same for this edge,
        /// LNext and LPrev, with the last triangle created having a higher value than any previously created triangle.
        /// </summary>
        public int TriangleId { get; internal set; }

        /// <summary>
        /// Returns the amount of clearance when traversing this edge while moving left.
        /// </summary>
        public float ClearanceLeft
        {
            get
            {
                if (_clearanceLeft == -1)
                    _clearanceLeft = Navmesh.GetLocalClearance(OPrev->Dest->Point, Org->Point, Dest->Point, DNext);
                return _clearanceLeft;
            }
            internal set => _clearanceLeft = value;
        }

        /// <summary>
        /// Returns the amount of clearance when traversing this edge while moving right.
        /// </summary>
        public float ClearanceRight
        {
            get
            {
                if (_clearanceRight == -1)
                    _clearanceRight = Navmesh.GetLocalClearance(ONext->Dest->Point, Org->Point, Dest->Point, DPrev->Sym);
                return _clearanceRight;
            }
            internal set => _clearanceRight = value;
        }
        
        public float DebugNonCalcClearanceLeft => _clearanceLeft;
        public float DebugNonCalcClearanceRight => _clearanceRight;

        /// <summary>
        /// True for one of a quadedge's two directed edges, i.e. true for either edge(x,y) or edge(y,x).
        /// This value is not stable and can change when updating the navmesh.
        /// </summary>
        public bool IsPrimary => _indexInQuadEdge == 0;

        internal bool RefineFailed
        {
            get => QuadEdge->RefineFailed;
            set => QuadEdge->RefineFailed = value;
        }

        internal int Mark // Used when removing Constraint Edges (depth-first search), to mark already visited QuadEdges
        {
            get => QuadEdge->Mark;
            set => QuadEdge->Mark = value;
        }

        public float2 SegVector => Dest->Point - Org->Point;

        public Edge* MajorEdge
        {
            get => QuadEdge->MajorEdge;
            internal set => QuadEdge->MajorEdge = value;
        }
        public bool HasMajorEdge => MajorEdge != null;
        public Edge* GetMajorEdgeSameDir() {
            UnityEngine.Debug.Assert(MathLib.IsParallel(SegVector.XOY(), MajorEdge->SegVector.XOY(), 0.001f));
            UnityEngine.Debug.Assert(MathLib.AreVectorsSameDir(SegVector, (IsPrimary ? MajorEdge : MajorEdge->Sym)->SegVector));
            return IsPrimary ? MajorEdge : MajorEdge->Sym;
        }

        public ReadOnly<Entity> Constraints => new(QuadEdge->Crep.Ptr, QuadEdge->Crep.Length);
        public bool Constrained => QuadEdge->Crep.Length > 0;
        
        // public bool IsBarrier => QuadEdge->Crep.Length > 0;
        public bool IsConstrainedBy(Entity id) => QuadEdge->Crep.Contains(id);
        public bool ConstraintsEqual(Edge* edge) => EdgeType == edge->EdgeType && QuadEdge->Crep.SequenceEqual(edge->QuadEdge->Crep);

        internal void AddConstraint(Entity id) { if (id.Index != int.MinValue) { QuadEdge->Crep.InsertSorted(id); } } // else { UnityEngine.Debug.Log("Not adding Major Constraint"); } }
        internal void RemoveConstraint(Entity id) => QuadEdge->Crep.Remove(id);

        /// <summary>
        /// Returns the symmetric edge.
        /// </summary>
        public Edge* Sym => GetEdge((_indexInQuadEdge + 2) & 3);

        /// <summary>
        /// Returns the dual-edge pointing from right to left.
        /// </summary>
        internal Edge* Rot => GetEdge((_indexInQuadEdge + 1) & 3);

        /// <summary>
        /// Returns the dual-edge pointing from left to right.
        /// </summary>
        internal Edge* InvRot => GetEdge((_indexInQuadEdge + 3) & 3); // Internal for Debug sake

        Edge* GetEdge(int i) => (Edge*) ((byte*) QuadEdge + i * sizeof(Edge));

        /// <summary>
        /// Returns the next edge about the origin with the same origin.
        /// </summary>
        public Edge* ONext => Next;

        /// <summary>
        /// Returns the previous edge about the origin with the same origin.
        /// </summary>
        public Edge* OPrev => Rot->ONext->Rot;

        /// <summary>
        /// Returns the next edge about the Right face with the same right face.
        /// </summary>
        public Edge* RNext => Rot->ONext->InvRot;

        /// <summary>
        /// Returns the previous edge about the Right face with the same right face.
        /// </summary>
        public Edge* RPrev => Sym->ONext;

        /// <summary>
        /// Returns the next edge about the destination with the same desination.
        /// </summary>
        public Edge* DNext => Sym->ONext->Sym;

        /// <summary>
        /// Returns the previous edge about the destination with the same destination.
        /// </summary>
        public Edge* DPrev => InvRot->ONext->InvRot;

        /// <summary>
        /// Returns the next edge about the left face with the same left face.
        /// </summary>
        public Edge* LNext => InvRot->ONext->Rot;

        /// <summary>
        /// Returns the previous edge about the left face with the same left face.
        /// </summary>
        public Edge* LPrev => ONext->Sym;

        public override string ToString()
            => $"Edge: {Org->ToString()} => {Dest->ToString()}";
    }
}
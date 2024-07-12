using Unity.Entities;
using System.Collections.Generic;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace DotsNav.Navmesh
{
    /// <summary>
    /// A directed edge in the triangulation. Traversal operators assume counter clockwise rotation,
    /// e.g. ONext will return the first edge in counter clockwise order around the origin, and OPrev will return the first edge in clockwise order.
    /// Updating the navmesh invalidates this structure.
    /// </summary>
    public unsafe struct Edge : IRefable
    {
        [System.Flags]
        public enum Type : byte {
            None = 0,
            Major = 1, // Casted to and from Vertex.Major / Vertex.Minor
            Minor = 1 << 1,

            Obstacle = 1 << 2,
            Clearance = 1 << 3,

            Terrain = 1 << 4,
            Ignore = 1 << 5,

            Overwritten = 1 << 6, // Type could be:   Minor | [Clearance|Obstacle] | Overwritten

            // MajorConstrained = Obstacle,
            // MinorConstrained = Terrain | Obstacle | Clearance,
        }

        public static readonly Dictionary<Type, Color> EdgeColors = new Dictionary<Type, Color> {
            { Type.Major | Type.Obstacle, Color.red },
            { Type.Major | Type.Clearance, Color.magenta },

            { Type.Minor | Type.Obstacle, Color.blue },
            { Type.Minor | Type.Clearance, Color.yellow },
            { Type.Minor | Type.Terrain, Color.green },
            { Type.Minor | Type.Ignore, Color.cyan },
        };

        internal readonly QuadEdge* QuadEdge;
        internal Edge* Next;
        readonly int _indexInQuadEdge; // TODO: Could make byte
        float _clearanceLeft;
        float _clearanceRight;

        /// <summary>
        /// Returns the origin vertex.
        /// </summary>
        public Vertex* Org { get; internal set; }

        /// <summary>
        /// Returns cost of traversing this edge's left face, i.e. this value will be the same for this edge, LNext and LPrev.
        /// </summary>
        public byte TriangleMaterial { get; internal set; }

        /// <summary>
        /// Returns a strictly increasing value unique to this edge's left face, i.e. this value will be the same for this edge,
        /// LNext and LPrev, with the last triangle created having a higher value than any previously created triangle.
        /// </summary>
        public int TriangleId { get; internal set; }

        /// <summary>
        /// Returns the slope cost of traversing this edge's left face, i.e. this value will be the same for this edge, LNext and LPrev.
        /// </summary>
        public float TriangleSlopeCost { get; internal set; }


        internal Edge(QuadEdge* quadEdge, int indexInQuadEdge)
        {
            Next = null;
            Org = null;
            QuadEdge = quadEdge;
            _indexInQuadEdge = indexInQuadEdge;
            _clearanceLeft = -1;
            _clearanceRight = -1;
            TriangleId = -1;
            TriangleSlopeCost = 1f;
            TriangleMaterial = 0;

#if UNITY_ASSERTIONS
            IsMajorEdgeOverwritten = false;
#endif
        }

        /// <summary>
        /// Returns the destination vertex.
        /// </summary>
        public Vertex* Dest => Sym->Org;

        /// <summary>
        /// Returns a value unique to this edge's quadedge, i.e. this value will be the same for edge(x,y) and edge(y,x).
        /// This value is not stable and can change when updating the navmesh.
        /// </summary>
        public int QuadEdgeId => QuadEdge->Id;

        public readonly Type EdgeType => QuadEdge->EdgeType;
        public readonly Type MainEdgeType { 
            get {
                Debug.Assert((QuadEdge->EdgeType & ~(Type.Major | Type.Minor)).HasNoFlagsB(Type.Major | Type.Minor));
                return QuadEdge->EdgeType & ~(Type.Major | Type.Minor);
            }
        }

        internal void SetEdgeType(Type fixedEdgeType) {
            QuadEdge->EdgeType = fixedEdgeType;
        }

        public unsafe bool IsEdgeTypeConstrained() {
            VerifyEdgeType(EdgeType);
            return (EdgeType.HasAllFlagsB(Type.Major) && EdgeType.HasAnyFlagsB(Type.Obstacle))
                || (EdgeType.HasAllFlagsB(Type.Minor) && EdgeType.HasAnyFlagsB(Type.Terrain | Type.Obstacle | Type.Clearance));
        }

        public static unsafe bool IsEdgeTypeMajor(Type edgeType) {
            Debug.Assert((edgeType.HasAnyFlagsB(Type.Major) && edgeType.HasNoFlagsB(Type.Minor))
                || (edgeType.HasNoFlagsB(Type.Major) && edgeType.HasAnyFlagsB(Type.Minor)), $"EdgeType: {edgeType}"); // Edge type must be either Major or Minor, cannot be both
            return edgeType.HasAllFlagsB(Type.Major);
        }

        // TODO: Make conditional
        public static unsafe void VerifyEdgeType(Type edgeType) => VerifyEdgeType(edgeType, IsEdgeTypeMajor(edgeType));
        public static unsafe void VerifyEdgeType(Type edgeType, bool isMajor) {
            Debug.Assert((isMajor && edgeType.HasAnyFlagsB(Type.Major) && edgeType.HasNoFlagsB(Type.Minor) && edgeType.HasAnyFlagsB(Type.Obstacle | Type.Clearance))
                || (!isMajor && edgeType.HasAnyFlagsB(Type.Minor) && edgeType.HasNoFlagsB(Type.Major) && edgeType.HasAnyFlagsB(Type.Obstacle | Type.Clearance | Type.Terrain | Type.Ignore))
                , $"isMajor: {isMajor}, edgeType: {edgeType}");
        }

        public static unsafe void VerifyEdge(Edge* e) {
            if (IsEdgeTypeMajor(e->EdgeType)) {
                Debug.Assert(MathLib.LogicalIff(e->EdgeType.HasAnyFlagsB(Type.Obstacle), e->IsConstrained), $"e->EdgeType: {e->EdgeType}, e->Constrained: {e->IsConstrained}");
            } else {
                // The following is not true because a Minor Terrain could be overwitten by a Minor Clearance | Obstacle
                //Debug.Assert(MathLib.LogicalIff(e->EdgeType.HasAnyFlagsB(Type.Terrain), e->IsConstrained), $"e->EdgeType: {e->EdgeType}, e->Constrained: {e->IsConstrained}");

                if (e->EdgeType.HasAnyFlagsB(Type.Obstacle | Type.Clearance)) {
                    Debug.Assert(e->MajorEdge != null, "Minor Obstaces and Clearances must have Major edges");
                    Debug.Assert(e->MajorEdge != null && (e->EdgeType & ~ Type.Minor) == (e->MajorEdge->EdgeType & ~Type.Major), $"e->EdgeType: {e->EdgeType}, e->MajorEdge->EdgeType: {e->MajorEdge->EdgeType}");
                }
            }
            if (e->MajorEdge != null) {
                Debug.Assert((e->EdgeType & ~ Type.Minor) == (e->MajorEdge->EdgeType & ~Type.Major), $"e->EdgeType: {e->EdgeType}, e->MajorEdge->EdgeType: {e->MajorEdge->EdgeType}");
            }
        }
        public static unsafe void VerifyEdge(Edge* e, bool isMajor) {
            VerifyEdgeType(e->EdgeType, isMajor);
            VerifyEdge(e);
        }

        public float CalcSlopeCost() => -FaceTriangle().Plane.CalcWalkingSlopeCost();

        public Triangle FaceTriangle() => new Triangle(LNext->Org->Point3D, Org->Point3D, LPrev->Org->Point3D);
        public Seg Seg() => new Seg(Org->Point3D, Dest->Point3D);

        public float2 SegVector => Dest->Point - Org->Point;
        public float3 SegVector3D => Dest->Point3D - Org->Point3D;


        /// <summary>
        /// Returns the amount of clearance when traversing this edge while moving left.
        /// </summary>
        public float ClearanceLeft
        {
            get
            {
                if (_clearanceLeft == -1) {
                    Debug.Assert(MainEdgeType != Type.Obstacle && OPrev->MainEdgeType != Type.Obstacle, "Attempting to traverse through Obstacle");
                    _clearanceLeft = Navmesh.GetLocalClearance(OPrev->Dest->Point, Org->Point, Dest->Point, DNext);
                }
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
                if (_clearanceRight == -1) {
                    Debug.Assert(MainEdgeType != Type.Obstacle && LPrev->MainEdgeType != Type.Obstacle, "Attempting to traverse through Obstacle");
                    _clearanceRight = Navmesh.GetLocalClearance(ONext->Dest->Point, Org->Point, Dest->Point, DPrev->Sym);
                }
                return _clearanceRight;
            }
            internal set => _clearanceRight = value;
        }
        
        public float DebugRawClearanceLeft => _clearanceLeft;
        public float DebugRawClearanceRight => _clearanceRight;

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

        internal Edge* MajorEdge
        {
            get => QuadEdge->MajorEdge;
            set => QuadEdge->MajorEdge = value;
        }
        public bool HasMajorEdge => MajorEdge != null;
        public bool ContainsMajorEdge(Edge* edgeMajor) => HasMajorEdge && (MajorEdge == edgeMajor || MajorEdge == edgeMajor->Sym);
        public Edge* GetMajorEdge() { // Assumes MajorEdge exists. Returns MajorEdge facing same direction as this edge
            QuadEdge->VerifyMajorEdge();
            return IsPrimary ? MajorEdge : MajorEdge->Sym;
        }

#if UNITY_ASSERTIONS
        public bool IsMajorEdgeOverwritten;
#endif

        public ReadOnly<Entity> Constraints => new(QuadEdge->Crep.Ptr, QuadEdge->Crep.Length);
        public bool IsConstrained => QuadEdge->Crep.Length > 0;
        
        public bool IsConstrainedBy(Entity id) => QuadEdge->Crep.Contains(id);
        public bool ConstraintsEqual(Edge* edge) => EdgeType == edge->EdgeType && QuadEdge->Crep.SequenceEqual(edge->QuadEdge->Crep);

        internal void AddConstraint(Entity id) { if (id.Index != int.MinValue) { QuadEdge->Crep.InsertSorted(id); } } // else { Debug.Log("Not adding Major Constraint"); } }
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

        public bool IsValid() {
            return true;
        }
    }
}
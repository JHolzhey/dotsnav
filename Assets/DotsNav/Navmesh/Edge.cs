using Unity.Entities;
using System.Collections.Generic;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace DotsNav.Navmesh
{

    public static class EdgeTypeExtensions {
        public static Edge.Type Main(this Edge.Type edgeType) => edgeType & Edge.Type.MainMask;
        public static bool IsConstrained(this Edge.Type edgeType) => (edgeType.IsMajor() && edgeType.HasAnyFlagsB(Edge.Type.Obstacle))
                || (!edgeType.IsMajor() && edgeType.HasAnyFlagsB(Edge.Type.Terrain | Edge.Type.Obstacle | Edge.Type.Clearance));

        // public static bool IsMajor(this Edge.Type edgeType) => edgeType.HasAnyFlagsB(Edge.Type.Major);
        public static bool IsMajor(this Edge.Type edgeType) { /* Debug.Log($"Test: {edgeType.Test(Edge.Type.Clearance)}"); */ return edgeType.HasAnyFlagsB(Edge.Type.Major); }
        public static bool IsOverwritten(this Edge.Type edgeType) => edgeType.HasAnyFlagsB(Edge.Type.Overwritten);
        public static void SetOverwritten(ref this Edge.Type edgeType, bool value) => edgeType.SetFlagTest(Edge.Type.Overwritten, value);

        public static Color GetDebugColor(this Edge.Type edgeType) => EdgeDebugColors[edgeType];
        public static readonly Dictionary<Edge.Type, Color> EdgeDebugColors = new Dictionary<Edge.Type, Color> {
            { Edge.Type.Major | Edge.Type.Obstacle, Color.red },
            { Edge.Type.Major | Edge.Type.Clearance, Color.magenta },

            { Edge.Type.Minor | Edge.Type.Obstacle, Color.blue },
            { Edge.Type.Minor | Edge.Type.Clearance, Color.yellow },

            { Edge.Type.Minor | Edge.Type.Obstacle | Edge.Type.Overwritten, 0.5f * Color.blue },
            { Edge.Type.Minor | Edge.Type.Clearance | Edge.Type.Overwritten, 0.5f * Color.yellow },

            { Edge.Type.Minor | Edge.Type.Terrain, Color.green },
            { Edge.Type.Minor | Edge.Type.Ignore, Color.cyan },
        };
    }

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
            Minor = 0,

            Obstacle = 1,
            Clearance = 1 << 1,

            Terrain = 1 << 2,
            Ignore = 1 << 3,

            MainMask = 0x0F, // TODO: Better bit mask creation

            Major = 1 << 4, // Casted to and from Vertex.Major / Vertex.Minor
            Overwritten = 1 << 5, // Type could be:   Minor | [Clearance|Obstacle] | Overwritten

            // MajorConstrained = Obstacle,
            // MinorConstrained = Terrain | Obstacle | Clearance,
        }

        internal readonly QuadEdge* QuadEdge;
        internal Edge* Next;
        internal readonly byte _indexInQuadEdge; // TODO: Made internal for debug
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


        internal Edge(QuadEdge* quadEdge, byte indexInQuadEdge)
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
        public readonly Type MainEdgeType => QuadEdge->EdgeType.Main();

        public void SetOverwritten(bool value) {
            QuadEdge->EdgeType.SetOverwritten(value);
        }

        internal void SetEdgeType(Type edgeType) {
            QuadEdge->EdgeType = edgeType;
        }

        // TODO: Make conditional
        public static unsafe void VerifyEdgeType(Type edgeType) => VerifyEdgeType(edgeType, edgeType.IsMajor());
        public static unsafe void VerifyEdgeType(Type edgeType, bool isMajor) {
            Debug.Assert((isMajor && edgeType.HasAnyFlagsB(Type.Major) && edgeType.HasNoFlagsB(Type.Minor) && edgeType.HasAnyFlagsB(Type.Obstacle | Type.Clearance))
                || (!isMajor && edgeType.HasAnyFlagsB(Type.Minor) && edgeType.HasNoFlagsB(Type.Major) && edgeType.HasAnyFlagsB(Type.Obstacle | Type.Clearance | Type.Terrain | Type.Ignore))
                , $"isMajor: {isMajor}, edgeType: {edgeType}");
        }

        public static unsafe void VerifyEdge(Edge* e) {
            if (e->EdgeType.IsMajor()) {
                Debug.Assert(MathLib.LogicalIff(e->EdgeType.HasAnyFlagsB(Type.Obstacle), e->IsConstrained), $"e->EdgeType: {e->EdgeType}, e->Constrained: {e->IsConstrained}");
            } else {
                // The following is not true because a Minor Terrain could be overridden by a Minor Clearance | Obstacle
                //Debug.Assert(MathLib.LogicalIff(e->EdgeType.HasAnyFlagsB(Type.Terrain), e->IsConstrained), $"e->EdgeType: {e->EdgeType}, e->Constrained: {e->IsConstrained}");

                Debug.Assert(MathLib.LogicalIff(e->EdgeType.HasAnyFlagsB(Type.Obstacle | Type.Clearance), e->MajorEdge != null), "Minor Obstaces and Clearances must have Major edges");
            }
            if (e->MajorEdge != null) { Debug.Assert(e->MainEdgeType == e->MajorEdge->MainEdgeType, $"e->EdgeType: {e->EdgeType}, e->MajorEdge->EdgeType: {e->MajorEdge->EdgeType}"); }
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
        
        public readonly float DebugRawClearanceLeft => _clearanceLeft;
        public readonly float DebugRawClearanceRight => _clearanceRight;

        /// <summary>
        /// True for one of a quadedge's two directed edges, i.e. true for either edge(x,y) or edge(y,x).
        /// This value is not stable and can change when updating the navmesh.
        /// </summary>
        public readonly bool IsPrimary => _indexInQuadEdge == 0;

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

        // TODO: Do edgeMajor->QuadEdge or edgeMajor->QuadEdgeId
        public bool ContainsMajorEdge(Edge* edgeMajor) => MajorEdge == edgeMajor || MajorEdge == edgeMajor->Sym; // Purposefully doesn't check HasMajorEdge
        public Edge* GetMajorEdge() => IsPrimary ? MajorEdge : MajorEdge->Sym; // Assumes MajorEdge exists. Returns MajorEdge facing same direction as this edge

        public ReadOnly<Entity> Constraints => new(QuadEdge->Crep.Ptr, QuadEdge->Crep.Length);
        public readonly bool IsConstrained => QuadEdge->Crep.Length > 0;
        
        public bool IsConstrainedBy(Entity cid) => QuadEdge->Crep.Contains(cid);
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
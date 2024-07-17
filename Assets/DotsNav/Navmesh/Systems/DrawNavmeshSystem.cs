using DotsNav.Drawing;
using DotsNav.Navmesh.Data;
using DotsNav.Systems;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Entities.UniversalDelegates;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace DotsNav.Navmesh.Systems
{
    //[BurstDiscard]
    [UpdateInGroup(typeof(DotsNavDrawingSystemGroup))]
    partial class DrawNavmeshSystem : SystemBase
    {
        const int numDebugTris = 5000;
        GameObject[] debugTris = new GameObject[numDebugTris];
        int prevOutputTrianglesLength = 0;
        NativeList<Pair<Ptr<Edge>, float4>> outputTriangles = new NativeList<Pair<Ptr<Edge>, float4>>(10, Allocator.Persistent);
        NativeHashSet<int> closedMinorTriangles = new NativeHashSet<int>(10, Allocator.Persistent);

        [BurstDiscard]
        protected override void OnCreate()
        {
            for (int i = 0; i < debugTris.Length; i++) { debugTris[i] = CommonLib.CreatePrimitive(PrimitiveType.Quad, float3.zero, new float3(1f), default); }
        }

        [BurstDiscard]
        protected override void OnDestroy() {}

        [BurstDiscard]
        protected override void OnUpdate()
        {
            CheckedStateRef.Dependency.Complete();

            if (outputTriangles.Length != prevOutputTrianglesLength) unsafe {
                for (int i = 0; i < outputTriangles.Length; i++) {
                    debugTris[i].transform.position = float3.zero;
                    Triangle tri = outputTriangles[i].first.p->FaceTriangle();
                    CommonLib.DebugTriangle(debugTris[i], tri, (outputTriangles[i].second * (1f / -tri.Plane.CalcWalkingSlopeCost())).ToColor());
                }
                for (int i = outputTriangles.Length; i < numDebugTris; i++) {
                    debugTris[i].transform.position = new float3(1000f);
                }
            }

            prevOutputTrianglesLength = outputTriangles.Length;
            outputTriangles.Clear();
            closedMinorTriangles.Clear();

            new DrawNavmeshJob{ outputMinorTriangles = outputTriangles, closedMinorTriangles = closedMinorTriangles }.Schedule();
            DotsNavRenderer.Handle.Data = JobHandle.CombineDependencies(DotsNavRenderer.Handle.Data, CheckedStateRef.Dependency);
        }


        public static void DrawPoint(ref NativeList<Line> lines, float2 pos, Color color, float size = .01f)
        {
            var diag = size / 2.8284f;
            var hor = size / 2;
            lines.Add(new Line(pos + new float2(-diag, -diag), pos + new float2(diag, diag), color));
            lines.Add(new Line(pos + new float2(-diag, diag), pos + new float2(diag, -diag), color));
            lines.Add(new Line(pos + new float2(-hor, 0), pos + new float2(hor, 0), color));
            lines.Add(new Line(pos + new float2(0, -hor), pos + new float2(0, hor), color));
        }

        public unsafe static void DrawEdgeTriangle(ref NativeList<Line> lines, Edge* edge, ReadOnly<NavmeshMaterialType> materialTypes, float size = .05f)
        {
            Triangle tri = edge->FaceTriangle().RoughInset(0.1f);

            lines.Add(new Line(tri.p0, tri.p1, materialTypes[edge->TriangleMaterial].color));
            lines.Add(new Line(tri.p1, tri.p2, materialTypes[edge->LPrev->TriangleMaterial].color));
            lines.Add(new Line(tri.p2, tri.p0, materialTypes[edge->LNext->TriangleMaterial].color));
        }

        //[BurstCompile]
        unsafe partial struct DrawNavmeshJob : IJobEntity
        {
            public NativeList<Pair<Ptr<Edge>, float4>> outputMinorTriangles;
            public NativeHashSet<int> closedMinorTriangles;

            [BurstDiscard]
            void Execute(NavmeshComponent navmesh, LocalToWorld ltw, NavmeshDrawComponent data)
            {
                if (data.DrawMode == DrawMode.None || navmesh.Navmesh == null)
                    return;

                var lines = new NativeList<Line>(navmesh.Navmesh->Vertices * 3, Allocator.Temp);

                var enumeratorMinor = navmesh.Navmesh->GetEdgeEnumerator(false);

                /* foreach (IntPtr vertex in navmesh.Navmesh->_verticesSeq) { // TODO: Commented out for performance reasons
                    Vertex* vert = (Vertex*) vertex;
                    if (vert->GetEdge(false) == null) {
                        DrawPoint(ref lines, vert->Point, Color.red);
                        Debug.Assert(false, "Major graph should not contain vertices that Minor graph doesn't have");
                    }
                } */

                int numMinorEdges = 0;
                int minorLefts = 0;
                int minorRights = 0;
                while (enumeratorMinor.MoveNext())
                {
                    var edge = enumeratorMinor.Current;
                    Debug.Assert(edge->_indexInQuadEdge == 0 || edge->_indexInQuadEdge == 1, $"edge->_indexInQuadEdge: {edge->_indexInQuadEdge}");
                    numMinorEdges++;

                    Debug.Assert(!edge->EdgeType.IsMajor(), $"edge->EdgeType: {edge->EdgeType}");

                    if (!(edge->TriangleId == edge->LNext->TriangleId && edge->LNext->TriangleId == edge->LPrev->TriangleId)) {
                        Debug.Assert(false, $"Unequal TriangleId, edge = {edge->TriangleId}, edge->LNext = {edge->LNext->TriangleId}, edge->LPrev = {edge->LPrev->TriangleId}");
                    }
                    if (!(edge->Sym->TriangleId == edge->Sym->LNext->TriangleId && edge->Sym->LNext->TriangleId == edge->Sym->LPrev->TriangleId)) {
                        Debug.Assert(false, $"Unequal TriangleId, edge->Sym = {edge->Sym->TriangleId}, edge->Sym->LNext = {edge->Sym->LNext->TriangleId}, edge->Sym->LPrev = {edge->Sym->LPrev->TriangleId}");
                    }

                    if (!(edge->TriangleMaterial == edge->LNext->TriangleMaterial && edge->LNext->TriangleMaterial == edge->LPrev->TriangleMaterial)) {
                        Debug.Assert(false, $"Unequal tri material, edge = {edge->TriangleMaterial}, edge->LNext = {edge->LNext->TriangleMaterial}, edge->LPrev = {edge->LPrev->TriangleMaterial}");
                        DrawEdgeTriangle(ref lines, edge, navmesh.Navmesh->MaterialTypes);
                    }
                    if (!(edge->Sym->TriangleMaterial == edge->Sym->LNext->TriangleMaterial && edge->Sym->LNext->TriangleMaterial == edge->Sym->LPrev->TriangleMaterial)) {
                        Debug.Assert(false, $"Unequal tri material, edge->Sym = {edge->Sym->TriangleMaterial}, edge->Sym->LNext = {edge->Sym->LNext->TriangleMaterial}, edge->Sym->LPrev = {edge->Sym->LPrev->TriangleMaterial}");
                        DrawEdgeTriangle(ref lines, edge->Sym, navmesh.Navmesh->MaterialTypes);
                    }

                    if (!closedMinorTriangles.Contains(edge->TriangleId)) {
                        closedMinorTriangles.Add(edge->TriangleId);
                        outputMinorTriangles.Add(new (edge, navmesh.Navmesh->MaterialTypes[edge->TriangleMaterial].color.ToFloat4()));
                    }
                    if (!closedMinorTriangles.Contains(edge->Sym->TriangleId)) {
                        closedMinorTriangles.Add(edge->Sym->TriangleId);
                        outputMinorTriangles.Add(new (edge->Sym, navmesh.Navmesh->MaterialTypes[edge->Sym->TriangleMaterial].color.ToFloat4()));
                    }


                    if (data.DrawMode == DrawMode.None || data.DrawMode == DrawMode.ConstrainedNoTerrain || data.DrawMode == DrawMode.BothNoTerrain
                        || (data.DrawMode == DrawMode.Constrained && !edge->IsConstrained))
                        // && edge->IsConstrained && edge->QuadEdge->Crep.Length == 1 && edge->QuadEdge->Crep[0] == Entity.Null)
                        continue;

                    Color c = edge->EdgeType.GetDebugColor();
                    
                    var a = math.transform(ltw.Value, edge->Org->Point.ToXxY());
                    var b = math.transform(ltw.Value, edge->Dest->Point.ToXxY());

                    Debug.Assert(!MathLib.IsZero(b - a));
                    float3 direction = math.normalizesafe(b - a);
                    float3 tangent = MathLib.CalcTangentToNormal(direction);
                    float3 leftOffset = 0.0005f * tangent * (edge->HasMajorEdge ? 1f : 0);
                    lines.Add(new Line(a + leftOffset + direction*0.001f, b + leftOffset - direction*0.001f, c));


                    // Asserting Major Edges:
                    if (edge->EdgeType.IsMajorInMinor()) {
                        float3 minorMidpoint = MathLib.Midpoint(a, b);

                        edge->QuadEdge->VerifyMajorEdge();
                        
                        var a_Major = math.transform(ltw.Value, edge->MajorEdge->Org->Point.ToXxY());
                        var b_Major = math.transform(ltw.Value, edge->MajorEdge->Dest->Point.ToXxY());
                        float3 majorMidpoint = MathLib.Midpoint(a_Major, b_Major);

                        float dot = math.dot(tangent, minorMidpoint - majorMidpoint);
                        float edgeLength = math.length(b - a);

                        // If dot too large draw line between and midpoints
                        if (!MathLib.LogicalIf(edgeLength > 0.1f, MathLib.IsEpsEqual(dot, 0f, 0.001f))) { // this 0.02f check is here because very small edges can rightfully be unaligned
                            // Debug.Assert(MathLib.IsEpsEqual(dot, 0.005f, 0.001f), $"dot: {dot}, edge length: {edgeLength}");
                            // lines.Add(new Line(minorMidpoint, majorMidpoint, Color.white));
                            // DrawPoint(ref lines, minorMidpoint.xz, Color.white, 0.03f);
                            // DrawPoint(ref lines, majorMidpoint.xz, Color.grey, 0.03f);
                            lines.Add(new Line(a + leftOffset*1.1f + direction*0.005f, b + leftOffset*1.1f - direction*0.005f, c));
                            lines.Add(new Line(a + leftOffset*1.2f + direction*0.005f, b + leftOffset*1.2f - direction*0.005f, c));
                            lines.Add(new Line(a + leftOffset*0.9f + direction*0.005f, b + leftOffset*1.3f - direction*0.005f, c));
                            lines.Add(new Line(a + leftOffset*0.9f + direction*0.005f, b + leftOffset*1.4f - direction*0.005f, c));
                        }
                    }

                    if (false) {
                        if (edge->DebugRawClearanceLeft != -1) {
                            minorLefts++;
                            Debug.Log($"Major edge->DebugNonCalcClearanceLeft: {edge->DebugRawClearanceLeft}");
                        }
                        if (edge->DebugRawClearanceRight != -1) {
                            minorRights++;
                            Debug.Log($"Minor edge->DebugNonCalcClearanceRight: {edge->DebugRawClearanceRight}");
                        }
                    }
                }
                // Debug.Log($"numMinorEdges: {numMinorEdges}");

                // Debug.Log($"Minor Lefts: {minorLefts}");
                // Debug.Log($"Minor Rights: {minorRights}");




                var enumerator = navmesh.Navmesh->GetEdgeEnumerator(true);

                /* foreach (IntPtr vertex in navmesh.Navmesh->_verticesSeq) { // TODO: Commented out for performance reasons
                    Vertex* vert = (Vertex*) vertex;
                    if (vert->GetEdge(true) == null) {
                        // This is expected since Minor graph will have Terrain vertices
                        DrawPoint(ref lines, vert->Point, Color.green);
                    }
                } */

                int majorLefts = 0;
                int majorRights = 0;
                while (enumerator.MoveNext())
                {
                    var edge = enumerator.Current;
                    Debug.Assert(edge->_indexInQuadEdge == 0 || edge->_indexInQuadEdge == 1, $"edge->_indexInQuadEdge: {edge->_indexInQuadEdge}");

                    Debug.Assert(edge->EdgeType.IsMajor(), $"edge->EdgeType: {edge->EdgeType}");

                    if (data.DrawMode == DrawMode.None || (data.DrawMode == DrawMode.Constrained && !edge->IsConstrained))
                        continue;

                    Color c = edge->EdgeType.GetDebugColor();
                        
                    var a = math.transform(ltw.Value, edge->Org->Point.ToXxY());
                    var b = math.transform(ltw.Value, edge->Dest->Point.ToXxY());

                    float3 direction = math.normalizesafe(b - a);
                    lines.Add(new Line(a + direction*0.001f, b - direction*0.001f, c));

                    if (false) {
                        if (edge->DebugRawClearanceLeft != -1) {
                            majorLefts++;
                            Debug.Log($"Major edge->DebugNonCalcClearanceLeft: {edge->DebugRawClearanceLeft}");
                        }
                        if (edge->DebugRawClearanceRight != -1) {
                            majorRights++;
                            Debug.Log($"Major edge->DebugNonCalcClearanceRight: {edge->DebugRawClearanceRight}");
                        }
                    }
                }

                // Debug.Log($"Major Lefts: {majorLefts}");
                // Debug.Log($"Major Rights: {majorRights}");


                Line.Draw(lines.AsArray());
            }
        }
    }
}
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
        GameObject[] debugTris;

        [BurstDiscard]
        protected override void OnCreate()
        {
            debugTris = new GameObject[500];
            for (int i = 0; i < debugTris.Length; i++) { debugTris[i] = CommonLib.CreatePrimitive(PrimitiveType.Quad, float3.zero, new float3(1f), default); }
        }

        [BurstDiscard]
        protected override void OnDestroy()
        {
        }

        [BurstDiscard]
        protected override void OnUpdate()
        {
            NativeList<Pair<Triangle, Color>> outputTriangles = new NativeList<Pair<Triangle, Color>>(10, Allocator.TempJob);

            new DrawNavmeshJob{ outputMinorTriangles = outputTriangles }.Schedule();
            DotsNavRenderer.Handle.Data = JobHandle.CombineDependencies(DotsNavRenderer.Handle.Data, CheckedStateRef.Dependency);

            CheckedStateRef.Dependency.Complete();
            for (int i = 0; i < outputTriangles.Length; i++) {
                debugTris[i].transform.position = float3.zero;
                CommonLib.DebugTriangle(debugTris[i], outputTriangles[i].first, outputTriangles[i].second);
            }
            for (int i = outputTriangles.Length; i < 500; i++) {
                debugTris[i].transform.position = new float3(1000f);
            }
            outputTriangles.Dispose();
        }


        public static void DrawPoint(ref NativeList<Line> lines, float2 pos, Color color, float size = .05f)
        {
            var diag = size / 2.8284f;
            var hor = size / 2;
            lines.Add(new Line(pos + new float2(-diag, -diag), pos + new float2(diag, diag), color));
            lines.Add(new Line(pos + new float2(-diag, diag), pos + new float2(diag, -diag), color));
            lines.Add(new Line(pos + new float2(-hor, 0), pos + new float2(hor, 0), color));
            lines.Add(new Line(pos + new float2(0, -hor), pos + new float2(0, hor), color));
        }

        public unsafe static void DrawEdgeTriangle(ref NativeList<Line> lines, Edge* edge, UnsafeList<NavmeshMaterialType> materialTypes, float size = .05f)
        {
            Triangle tri = edge->Get3DTriangle().RoughInset(0.1f);

            lines.Add(new Line(tri.p0, tri.p1, materialTypes[edge->MaterialTypeIndex].color));
            lines.Add(new Line(tri.p1, tri.p2, materialTypes[edge->LPrev->MaterialTypeIndex].color));
            lines.Add(new Line(tri.p2, tri.p0, materialTypes[edge->LNext->MaterialTypeIndex].color));
        }

        //[BurstCompile]
        unsafe partial struct DrawNavmeshJob : IJobEntity
        {
            public NativeList<Pair<Triangle, Color>> outputMinorTriangles;

            [BurstDiscard]
            void Execute(NavmeshComponent navmesh, LocalToWorld ltw, NavmeshDrawComponent data)
            {
                if (data.DrawMode == DrawMode.None || navmesh.Navmesh == null)
                    return;

                var lines = new NativeList<Line>(navmesh.Navmesh->Vertices * 3, Allocator.Temp);

                UnsafeHashSet<int> closedMinorTriangles = new UnsafeHashSet<int>(10, Allocator.Temp);

                var enumeratorMinor = navmesh.Navmesh->GetEdgeEnumerator(false);

                foreach (IntPtr vertex in navmesh.Navmesh->_verticesSeq) {
                    Vertex* vert = (Vertex*) vertex;
                    if (vert->GetEdge(false) == null) {
                        DrawPoint(ref lines, vert->Point, Color.red);
                        Debug.Assert(false, "Major graph should not contain vertices that Minor graph doesn't have");
                    }
                }

                int minorLefts = 0;
                int minorRights = 0;
                while (enumeratorMinor.MoveNext())
                {
                    var edge = enumeratorMinor.Current;

                    Debug.Assert(!Edge.IsEdgeTypeMajor(edge->EdgeType), $"edge->EdgeType: {edge->EdgeType}");
                    Debug.Assert(edge->TriangleCost == 1f || edge->TriangleCost == 1f * 0.5f, $"edge->TriangleCost: {edge->TriangleCost}");

                    if (data.DrawMode == DrawMode.Constrained && !edge->IsConstrained)
                        continue;

                    Edge.EdgeColors.TryGetValue(edge->EdgeType, out Color c);


                    
                    var a = math.transform(ltw.Value, edge->Org->Point.ToXxY());
                    var b = math.transform(ltw.Value, edge->Dest->Point.ToXxY());

                    float3 tangent = MathLib.CalcTangentToNormal(math.normalizesafe(b - a));
                    float3 leftOffset = 0.005f * MathLib.CalcTangentToNormal(math.normalizesafe(b - a));
                    lines.Add(new Line(a + leftOffset, b + leftOffset, c));

                    if (!(edge->MaterialTypeIndex == edge->LNext->MaterialTypeIndex && edge->LNext->MaterialTypeIndex == edge->LPrev->MaterialTypeIndex)) {
                        Debug.Assert(false, $"Unequal tri material, edge = {edge->MaterialTypeIndex}, edge->LNext = {edge->LNext->MaterialTypeIndex}, edge->LPrev = {edge->LPrev->MaterialTypeIndex}");
                        DrawEdgeTriangle(ref lines, edge, navmesh.Navmesh->MaterialTypes);
                    }
                    if (!(edge->Sym->MaterialTypeIndex == edge->Sym->LNext->MaterialTypeIndex && edge->Sym->LNext->MaterialTypeIndex == edge->Sym->LPrev->MaterialTypeIndex)) {
                        Debug.Assert(false, $"Unequal tri material, edge->Sym = {edge->Sym->MaterialTypeIndex}, edge->Sym->LNext = {edge->Sym->LNext->MaterialTypeIndex}, edge->Sym->LPrev = {edge->Sym->LPrev->MaterialTypeIndex}");
                        DrawEdgeTriangle(ref lines, edge->Sym, navmesh.Navmesh->MaterialTypes);
                    }
                    if (!closedMinorTriangles.Contains(edge->TriangleId)) {
                        closedMinorTriangles.Add(edge->TriangleId);
                        outputMinorTriangles.Add(new Pair<Triangle, Color>(edge->Get3DTriangle(), navmesh.Navmesh->MaterialTypes[edge->MaterialTypeIndex].color));
                    } else if (!closedMinorTriangles.Contains(edge->Sym->TriangleId)) {
                        closedMinorTriangles.Add(edge->Sym->TriangleId);
                        outputMinorTriangles.Add(new Pair<Triangle, Color>(edge->Sym->Get3DTriangle(), navmesh.Navmesh->MaterialTypes[edge->Sym->MaterialTypeIndex].color));
                    }

                    // Asserting Major Edges:
                    if (edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance)) {
                        float3 minorMidpoint = MathLib.Midpoint(a + leftOffset, b + leftOffset);

                        Edge* majorEdge = edge->MajorEdge;
                        Debug.Assert((edge->EdgeType & ~Edge.Type.Minor) == (majorEdge->EdgeType & ~Edge.Type.Major), $"edge->EdgeType: {edge->EdgeType}, majorEdge->EdgeType: {majorEdge->EdgeType}");
                        Debug.Assert(majorEdge->Org != null && majorEdge->Dest != null, $"Org: {majorEdge->Org != null}, Dest: {majorEdge->Dest != null}");
                        var a_Major = math.transform(ltw.Value, majorEdge->Org->Point.ToXxY());
                        var b_Major = math.transform(ltw.Value, majorEdge->Dest->Point.ToXxY());
                        float3 majorMidpoint = MathLib.Midpoint(a_Major, b_Major);

                        float dot = math.dot(tangent, minorMidpoint - majorMidpoint);

                        // if dot too large draw line between
                        if (!MathLib.IsEpsEqual(dot, 0.005f, 0.001f)) {
                            Debug.Assert(MathLib.IsEpsEqual(dot, 0.005f, 0.001f), $"Length: {dot}");
                            lines.Add(new Line(minorMidpoint, majorMidpoint, Color.grey));
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

                // Debug.Log($"Minor Lefts: {minorLefts}");
                // Debug.Log($"Minor Rights: {minorRights}");




                var enumerator = navmesh.Navmesh->GetEdgeEnumerator(true);

                foreach (IntPtr vertex in navmesh.Navmesh->_verticesSeq) {
                    Vertex* vert = (Vertex*) vertex;
                    if (vert->GetEdge(true) == null) {
                        // This is expected since Minor graph will have Terrain vertices
                        DrawPoint(ref lines, vert->Point, Color.green);
                    }
                }

                int majorLefts = 0;
                int majorRights = 0;
                while (enumerator.MoveNext())
                {
                    var edge = enumerator.Current;

                    Debug.Assert(Edge.IsEdgeTypeMajor(edge->EdgeType), $"edge->EdgeType: {edge->EdgeType}");

                    if (data.DrawMode == DrawMode.Constrained && !edge->IsConstrained)
                        continue;

                    Edge.EdgeColors.TryGetValue(edge->EdgeType, out Color c);

                    // if (edge->Constrained) { c = data.ConstrainedColor; c.a += 30; } else { c = data.UnconstrainedColor; }
                        
                    var a = math.transform(ltw.Value, edge->Org->Point.ToXxY());
                    var b = math.transform(ltw.Value, edge->Dest->Point.ToXxY());

                    lines.Add(new Line(a, b, c));

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
using DotsNav.Drawing;
using DotsNav.Navmesh.Data;
using DotsNav.Systems;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace DotsNav.Navmesh.Systems
{
    //[BurstDiscard]
    [UpdateInGroup(typeof(DotsNavDrawingSystemGroup))]
    partial struct DrawNavmeshSystem : ISystem
    {
        [BurstDiscard]
        public void OnCreate(ref SystemState state)
        {
        }

        [BurstDiscard]
        public void OnDestroy(ref SystemState state)
        {
        }

        [BurstDiscard]
        public void OnUpdate(ref SystemState state)
        {
            new DrawNavmeshJob().Schedule();
            DotsNavRenderer.Handle.Data = JobHandle.CombineDependencies(DotsNavRenderer.Handle.Data, state.Dependency);
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

        //[BurstCompile]
        unsafe partial struct DrawNavmeshJob : IJobEntity
        {
            [BurstDiscard]
            void Execute(NavmeshComponent navmesh, LocalToWorld ltw, NavmeshDrawComponent data)
            {
                if (data.DrawMode == DrawMode.None || navmesh.Navmesh == null)
                    return;

                var lines = new NativeList<Line>(navmesh.Navmesh->Vertices * 3, Allocator.Temp);


                var enumeratorMinor = navmesh.Navmesh->GetEdgeEnumerator(false);

                while (true && enumeratorMinor.MoveNext())
                {
                    var edge = enumeratorMinor.Current;
                    if (edge == null) {
                        DrawPoint(ref lines, enumeratorMinor.CurrentVertex->Point, Color.white);
                        continue;
                    }
                    Debug.Assert(!Edge.IsEdgeTypeMajor(edge->EdgeType), $"edge->EdgeType: {edge->EdgeType}");

                    if (data.DrawMode == DrawMode.Constrained && !edge->Constrained)
                        continue;

                    if (!Edge.EdgeColors.TryGetValue(edge->EdgeType, out Color c)) {
                        c = Color.white;
                    }
                    
                    var a = math.transform(ltw.Value, edge->Org->Point.ToXxY());
                    var b = math.transform(ltw.Value, edge->Dest->Point.ToXxY());

                    float3 leftOffset = 0.005f * MathLib.CalcTangentToNormal(math.normalizesafe(b - a));

                    lines.Add(new Line(a + leftOffset, b + leftOffset, c));

                    if (edge->EdgeType.HasAnyFlagsB(Edge.Type.Obstacle | Edge.Type.Clearance)) {
                        float3 minorMidpoint = MathLib.Midpoint(a + leftOffset, b + leftOffset);

                        Edge* majorEdge = edge->MajorEdge;
                        Debug.Assert((edge->EdgeType & ~Edge.Type.Minor) == (majorEdge->EdgeType & ~Edge.Type.Major), $"edge->EdgeType: {edge->EdgeType}, majorEdge->EdgeType: {majorEdge->EdgeType}");
                        Debug.Assert(majorEdge->Org != null && majorEdge->Dest != null, $"Org: {majorEdge->Org != null}, Dest: {majorEdge->Dest != null}");
                        var a_Major = math.transform(ltw.Value, majorEdge->Org->Point.ToXxY());
                        var b_Major = math.transform(ltw.Value, majorEdge->Dest->Point.ToXxY());
                        float3 majorMidpoint = MathLib.Midpoint(a_Major, b_Major);

                        Debug.Assert(MathLib.IsEpsEqual(math.length(minorMidpoint - majorMidpoint), 0.005f, 0.001f), $"Length: {math.length(minorMidpoint - majorMidpoint)}");
                    }
                }

                var enumerator = navmesh.Navmesh->GetEdgeEnumerator(true);

                while (enumerator.MoveNext())
                {
                    var edge = enumerator.Current;
                    if (edge == null) {
                        DrawPoint(ref lines, enumeratorMinor.CurrentVertex->Point, Color.white);
                        continue;
                    }
                    Debug.Assert(Edge.IsEdgeTypeMajor(edge->EdgeType), $"edge->EdgeType: {edge->EdgeType}");

                    if (data.DrawMode == DrawMode.Constrained && !edge->Constrained)
                        continue;

                    if (!Edge.EdgeColors.TryGetValue(edge->EdgeType, out Color c)) {
                        c = Color.gray;
                    }

                    // if (edge->Constrained) { c = data.ConstrainedColor; c.a += 30; } else { c = data.UnconstrainedColor; }
                        
                    var a = math.transform(ltw.Value, edge->Org->Point.ToXxY());
                    var b = math.transform(ltw.Value, edge->Dest->Point.ToXxY());

                    lines.Add(new Line(a, b, c));
                }

                Line.Draw(lines.AsArray());
            }
        }
    }
}
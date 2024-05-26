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

        //[BurstCompile]
        unsafe partial struct DrawNavmeshJob : IJobEntity
        {
            [BurstDiscard]
            void Execute(NavmeshComponent navmesh, LocalToWorld ltw, NavmeshDrawComponent data)
            {
                if (data.DrawMode == DrawMode.None || navmesh.Navmesh == null)
                    return;

                var lines = new NativeList<Line>(navmesh.Navmesh->Vertices * 3, Allocator.Temp);
                var enumerator = navmesh.Navmesh->GetEdgeEnumerator(true);

                while (enumerator.MoveNext())
                {
                    var edge = enumerator.Current;
                    if (data.DrawMode == DrawMode.Constrained && !edge->Constrained)
                        continue;

                    Color c = DebugColors.EdgeColors[(int)edge->EdgeType];

                    // if (edge->Constrained) { c = data.ConstrainedColor; c.a += 30; } else { c = data.UnconstrainedColor; }
                        
                    var a = math.transform(ltw.Value, edge->Org->Point.ToXxY());
                    var b = math.transform(ltw.Value, edge->Dest->Point.ToXxY());

                    lines.Add(new Line(a, b, c));
                }

                var enumeratorMinor = navmesh.Navmesh->GetEdgeEnumerator(false);

                while (enumeratorMinor.MoveNext())
                {
                    var edge = enumeratorMinor.Current;
                    if (data.DrawMode == DrawMode.Constrained && !edge->Constrained)
                        continue;

                    Color c = DebugColors.EdgeColors[(int)edge->EdgeType];
                        
                    var a = math.transform(ltw.Value, edge->Org->Point.ToXxY());
                    var b = math.transform(ltw.Value, edge->Dest->Point.ToXxY());

                    float3 leftOffset = 0.005f * MathLib.CalcTangentToNormal(b - a);

                    lines.Add(new Line(a + leftOffset, b + leftOffset, c));
                }

                Line.Draw(lines);
            }
        }
    }
}
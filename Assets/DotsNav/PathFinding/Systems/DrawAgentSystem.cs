using DotsNav.Data;
using DotsNav.Drawing;
using DotsNav.PathFinding.Data;
using DotsNav.Systems;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Mathematics.math;
using static DotsNav.Math;
using Unity.Entities.UniversalDelegates;

namespace DotsNav.PathFinding.Systems
{
    [BurstCompile]
    [UpdateInGroup(typeof(DotsNavDrawingSystemGroup))]
    partial struct DrawAgentSystem : ISystem
    {
        ComponentLookup<LocalToWorld> _localToWorldLookup;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            _localToWorldLookup = state.GetComponentLookup<LocalToWorld>(true);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        {
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            _localToWorldLookup.Update(ref state);
            new DrawAgentJob { LtwLookup = _localToWorldLookup }.Schedule();
            DotsNavRenderer.Handle.Data = JobHandle.CombineDependencies(DotsNavRenderer.Handle.Data, state.Dependency);
        }

        [BurstCompile]
        partial struct DrawAgentJob : IJobEntity
        {
            [ReadOnly] public ComponentLookup<LocalToWorld> LtwLookup;
            
            void Execute(AgentComponent agent, AgentDrawComponent settings, DynamicBuffer<PathSegmentElement> path, NavmeshAgentComponent navmesh)
            {
                if (!settings.Draw || path.Length == 0)
                    return;

                var ltw = LtwLookup[navmesh.Navmesh].Value;

                var lines = new NativeList<Line>(Allocator.Temp);
                var color = settings.Color;
                if (color.a < 10)
                    color.a += 10;

                float prevRadius = agent.Radius.max;
                for (int j = 0; j < path.Length; j++)
                {
                    var segment = path[j];
                    float radius = j == path.Length - 1 ? agent.Radius.max : segment.Radius; // If last segment then use max radius instead
                    var perpFrom = normalize(PerpCcw(segment.To - segment.From)) * prevRadius;
                    var perpTo = normalize(PerpCcw(segment.To - segment.From)) * radius;
                    lines.Add(new Line(transform(ltw, (segment.From + perpFrom).ToXxY()), transform(ltw, (segment.To + perpTo).ToXxY()), color));
                    lines.Add(new Line(transform(ltw, (segment.From - perpFrom).ToXxY()), transform(ltw, (segment.To - perpTo).ToXxY()), color));

                    Arrow.Draw(transform(ltw, segment.From.ToXxY()), transform(ltw, segment.To.ToXxY()), 0.02f, color);

                    // Makes a rectangle to signify a soldier unit
                    if (j == 0) {
                        float3 paraDepth = normalize(segment.To.ToXxY() - segment.From.ToXxY()) * agent.Radius.min * agent.Depth;
                        float3 a = (segment.From + perpFrom).ToXxY();
                        float3 b = (segment.From - perpFrom).ToXxY();
                        lines.Add(new Line(a, b, color));

                        lines.Add(new Line(a, a - paraDepth, color));
                        lines.Add(new Line(b, b - paraDepth, color));
                        lines.Add(new Line(a - paraDepth, b - paraDepth, color));
                    }

                    prevRadius = radius;
                }

                var up = rotate(ltw, new float3(0, 1, 0));

                for (int j = 1; j < path.Length; j++)
                {
                    var f = path[j - 1].To;
                    var segment = path[j];
                    var c = segment.Corner;
                    var t = segment.From;
                    var a = (Angle) Angle(t - c) - Angle(f - c);
                    Arc.Draw(lines, transform(ltw, c.ToXxY()), up, rotate(ltw, 2 * (f - c).ToXxY()), a, color, segment.Radius, settings.Delimit);
                }

                // Radius.min
                var arm = rotate(ltw, new float3(0, 0, agent.Radius.min));
                Arc.Draw(lines, transform(ltw, path[0].From.ToXxY()), up, arm, 2 * PI, color * 0.5f, agent.Radius.min, settings.Delimit);
                Arc.Draw(lines, transform(ltw, path[^1].To.ToXxY()), up, arm, 2 * PI, color * 0.5f, agent.Radius.min, settings.Delimit);

                // Radius.max
                arm = rotate(ltw, new float3(0, 0, agent.Radius.max));
                Arc.Draw(lines, transform(ltw, path[0].From.ToXxY()), up, arm, 2 * PI, color, agent.Radius.max, settings.Delimit);
                Arc.Draw(lines, transform(ltw, path[^1].To.ToXxY()), up, arm, 2 * PI, color, agent.Radius.max, settings.Delimit);

                Line.Draw(lines);
            }
        }
    }
}
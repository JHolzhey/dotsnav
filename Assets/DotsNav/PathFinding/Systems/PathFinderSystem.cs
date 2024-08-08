using DotsNav.Data;
using DotsNav.Navmesh.Data;
using DotsNav.Navmesh.Systems;
using DotsNav.PathFinding.Data;
using DotsNav.Systems;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

// All changes in this file should be discarded

namespace DotsNav.PathFinding.Systems
{
    [UpdateInGroup(typeof(DotsNavSystemGroup))]
    [UpdateAfter(typeof(UpdateNavmeshSystem))]
    public partial class PathFinderSystem : SystemBase
    {
        NativeList<Entity> _buffer;
        NativeQueue<Entity> _queue;

        protected override void OnCreate()
        {
            RequireForUpdate<PathFinderComponent>();
            RequireForUpdate<PathFinderSystemStateComponent>();
            _buffer = new NativeList<Entity>(Allocator.Persistent);
            _queue = new NativeQueue<Entity>(Allocator.Persistent);
        }

        protected override void OnDestroy()
        {
            _buffer.Dispose();
            _queue.Dispose();
        }

        protected override void OnUpdate()
        {
            var data = GetSingleton<PathFinderComponent>();
            var resources = GetSingleton<PathFinderSystemStateComponent>();
            var destroyed = GetBufferLookup<DestroyedTriangleElement>(true);
            var buffer = _buffer;
            var queue = _queue;
            var writer = queue.AsParallelWriter();

            Entities
                .WithName("GatherAgentsToRecalculate")
                .WithReadOnly(destroyed)
                .ForEach((Entity entity, int nativeThreadIndex, NavmeshAgentComponent navmesh, ref PathQueryComponent query, ref DynamicBuffer<TriangleElement> triangles) =>
                {
                    if (query.State == PathQueryState.PathFound)
                    {
                        var seq0 = destroyed[navmesh.Navmesh].Reinterpret<int>();
                        var seq1 = triangles.Reinterpret<int>();

                        if (SortedSequencesContainAnIdenticalElement(seq0, seq1)) // TODO: Only check this if navmesh has updated? Or maybe even if destroyed has changed?
                            query.State = PathQueryState.Invalidated;
                    }

                    if ((query.State & data.RecalculateFlags & ~PathQueryState.Inactive) != 0)
                        writer.Enqueue(entity);

                })
                .Run();

            Job
                .WithCode(() =>
                {
                    buffer.Clear();
                    while (queue.TryDequeue(out var e))
                        buffer.Add(e);
                })
                .Run();


            /* Dependency =  */new FindPathJob
                {
                    Entities = buffer.AsArray(),
                    NavmeshAgents = GetComponentLookup<NavmeshAgentComponent>(true),
                    Navmeshes = GetComponentLookup<NavmeshComponent>(true),
                    LTWLookup = GetComponentLookup<LocalToWorld>(true),
                    TranslationLookup = GetComponentLookup<LocalTransform>(true),
                    Queries = GetComponentLookup<PathQueryComponent>(),
                    Agents = GetComponentLookup<AgentComponent>(true),
                    PathSegments = GetBufferLookup<PathSegmentElement>(),
                    TriangleIds = GetBufferLookup<TriangleElement>(),
                    PathFinder = resources,
                }.Execute();
                // .Run(/* buffer, 1, Dependency */);
        }


        // TODO: Turn this into an AgentAspect.Lookup
        struct FindPathJob : IJob //ParallelForDefer
        {
            [ReadOnly]
            public NativeArray<Entity> Entities;
            [NativeDisableContainerSafetyRestriction]
            public ComponentLookup<PathQueryComponent> Queries;
            [NativeDisableContainerSafetyRestriction]
            public ComponentLookup<AgentComponent> Agents;
            [NativeDisableContainerSafetyRestriction]
            public BufferLookup<PathSegmentElement> PathSegments;
            [NativeDisableContainerSafetyRestriction]
            public BufferLookup<TriangleElement> TriangleIds;
            public PathFinderSystemStateComponent PathFinder;

            [NativeSetThreadIndex]
            int _threadId;

            [ReadOnly]
            public ComponentLookup<NavmeshAgentComponent> NavmeshAgents;
            [ReadOnly]
            public ComponentLookup<NavmeshComponent> Navmeshes;
            [ReadOnly]
            public ComponentLookup<LocalToWorld> LTWLookup;
            [ReadOnly]
            public ComponentLookup<LocalTransform> TranslationLookup;

            public unsafe void Execute(/* int index */)
            {
                for (int index = 0; index < Entities.Length; index++) {
                    // Assert.IsTrue(_threadId > 0 && _threadId <= PathFinder.Instances.Length);
                    var entity = Entities[index];
                    var query = Queries[entity];
                    var segments = PathSegments[entity];
                    segments.Clear();
                    var ids = TriangleIds[entity];
                    ids.Clear();
                    var instanceIndex = _threadId - 1;
                    var instance = PathFinder.Instances[0];

                    var navmeshEntity = NavmeshAgents[entity].Navmesh;
                    var ltw = math.inverse(LTWLookup[navmeshEntity].Value);
                    var pos = TranslationLookup[entity];
                    query.State = instance.FindPath(math.transform(ltw, pos.Position).xz, math.transform(ltw, query.To).xz, Agents[entity], Navmeshes[navmeshEntity].Navmesh, segments, ids, out _);
                    if (query.State == PathQueryState.PathFound)
                        ++query.Version;
                    Queries[entity] = query;
                }
            }
        }

        static bool SortedSequencesContainAnIdenticalElement(DynamicBuffer<int> seq0, DynamicBuffer<int> seq1)
        {
            if (seq0.Length == 0 || seq1.Length == 0)
                return false;

            var i0 = 0;
            var i1 = 0;

            while (true)
            {
                var v0 = seq0[i0];
                var v1 = seq1[i1];

                if (v0 == v1)
                    return true;

                if (v0 < v1)
                {
                    if (i0 < seq0.Length - 1)
                        ++i0;
                    else
                        return false;
                }
                else if (i1 < seq1.Length - 1)
                    ++i1;
                else
                    return false;
            }
        }
    }
}
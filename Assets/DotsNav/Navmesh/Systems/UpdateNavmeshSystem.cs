using System;
using DotsNav.Data;
using DotsNav.Hybrid;
using DotsNav.Navmesh.Data;
using DotsNav.Systems;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Entities.SystemAPI;

namespace DotsNav.Navmesh.Systems
{
    [/* BurstDiscard,  */UpdateInGroup(typeof(DotsNavSystemGroup))]
    [RequireMatchingQueriesForUpdate] // todo doesnt work?
    public unsafe partial struct UpdateNavmeshSystem : ISystem
    {
        int numMajorInMinorInsertions;
        EntityQuery _navmeshQuery;
        EntityQuery _destroyQuery;
        EntityQuery _insertQuery;
        EntityQuery _insertBulkQuery;
        EntityQuery _blobQuery;
        EntityQuery _blobBulkQuery;

        EntityQuery _insertTerrainQuery;
        EntityQuery _insertTerrainBulkQuery;
        EntityQuery _destroyTerrainQuery;
        
        ComponentLookup<NavmeshComponent> _navmeshLookup;
        ComponentLookup<LocalToWorld> _localToWorldLookup;
        BufferLookup<DestroyedTriangleElement> _destroyedTriangleBufferLookup;

        [BurstDiscard]
        public void OnCreate(ref SystemState state)
        {
            numMajorInMinorInsertions = 0;
            _navmeshQuery = 
                new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<NavmeshComponent>()
                    .Build(ref state);

            _destroyQuery =
                new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<CleanUpObstacleComponent>()
                    .WithNone<NavmeshObstacleComponent>()
                    .Build(ref state);

            _insertQuery =
                new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<PlaneComponent>()
                    .WithAll<VertexElement>()
                    .WithAll<NavmeshObstacleComponent>()
                    .WithAll<LocalToWorld>()
                    .WithNone<VertexAmountElement>()
                    .WithNone<CleanUpObstacleComponent>()
                    .Build(ref state);

            _insertBulkQuery =
                new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<PlaneComponent>()
                    .WithAll<VertexElement>()
                    .WithAll<VertexAmountElement>()
                    .WithAll<NavmeshObstacleComponent>()
                    .WithAll<LocalToWorld>()
                    .WithNone<CleanUpObstacleComponent>()
                    .Build(ref state);

            _blobQuery =
                new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<PlaneComponent>()
                    .WithAll<VertexBlobComponent>()
                    .WithAll<NavmeshObstacleComponent>()
                    .WithAll<LocalToWorld>()
                    .WithNone<CleanUpObstacleComponent>()
                    .Build(ref state);

            _blobBulkQuery =
                new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<PlaneComponent>()
                    .WithAll<ObstacleBlobComponent>()
                    .WithAll<NavmeshObstacleComponent>()
                    .WithAll<LocalToWorld>()
                    .WithNone<CleanUpObstacleComponent>()
                    .Build(ref state);
                
            _destroyTerrainQuery =
                new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<CleanUpTerrainComponent>()
                    .WithNone<NavmeshTerrainComponent>()
                    .Build(ref state);

            _insertTerrainQuery =
                new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<PlaneComponent>()
                    .WithAll<VertexElement>()
                    .WithAll<NavmeshTerrainComponent>()
                    .WithAll<LocalToWorld>()
                    .WithNone<VertexAmountElement>()
                    .WithNone<CleanUpTerrainComponent>()
                    .Build(ref state);

            _insertTerrainBulkQuery =
                new EntityQueryBuilder(Allocator.Temp)
                    .WithAll<PlaneComponent>()
                    .WithAll<VertexElement>()
                    .WithAll<VertexAmountElement>()
                    .WithAll<NavmeshTerrainComponent>()
                    .WithAll<LocalToWorld>()
                    .WithNone<CleanUpTerrainComponent>()
                    .Build(ref state);
            
            _navmeshLookup = state.GetComponentLookup<NavmeshComponent>(true);
            _localToWorldLookup = state.GetComponentLookup<LocalToWorld>(true);
            _destroyedTriangleBufferLookup = state.GetBufferLookup<DestroyedTriangleElement>();
        }

        [BurstDiscard]
        public void OnDestroy(ref SystemState state)
        {
        }

        [BurstDiscard]
        public void OnUpdate(ref SystemState state)
        {
            state.EntityManager.GetAllUniqueSharedComponents(out NativeList<PlaneComponent> planes, Allocator.TempJob);
            state.EntityManager.GetAllUniqueSharedComponents(out NativeList<CleanUpObstacleComponent> removals, Allocator.TempJob);
            var dependencies = new NativeList<JobHandle>(Allocator.Temp);
            var planeEntities = _navmeshQuery.ToEntityArray(Allocator.Temp);
            _navmeshLookup.Update(ref state);
            _localToWorldLookup.Update(ref state);
            _destroyedTriangleBufferLookup.Update(ref state);
            
            foreach (var planeEntity in planeEntities)
            {
                var destroyIsEmpty = true;
                var destroyTerrainIsEmpty = true;
                var exists = false;
                foreach (var removal in removals)
                {
                    if (removal.Plane == planeEntity)
                    {
                        exists = true;
                        break;
                    }
                }

                if (exists)
                {
                    _destroyQuery.SetSharedComponentFilter(new CleanUpObstacleComponent { Plane = planeEntity });
                    destroyIsEmpty = _destroyQuery.IsEmpty;

                    _destroyTerrainQuery.SetSharedComponentFilter(new CleanUpTerrainComponent { Plane = planeEntity });
                    destroyTerrainIsEmpty = _destroyTerrainQuery.IsEmpty;
                }

                var insertIsEmpty = true;
                var insertBulkIsEmpty = true;
                var blobIsEmpty = true;
                var blobBulkIsEmpty = true;

                var insertTerrainIsEmpty  = true;
                var insertTerrainBulkIsEmpty  = true;
                
                exists = false;
                foreach (var plane1 in planes)
                {
                    if (plane1.Entity == planeEntity)
                    {
                        exists = true;
                        break;
                    }
                }

                if (exists)
                {
                    var plane = new PlaneComponent { Entity = planeEntity };

                    _insertQuery.SetSharedComponentFilter(plane);
                    insertIsEmpty = _insertQuery.IsEmpty;
                    _insertBulkQuery.SetSharedComponentFilter(plane);
                    insertBulkIsEmpty = _insertBulkQuery.IsEmpty;
                    _blobQuery.SetSharedComponentFilter(plane);
                    blobIsEmpty = _blobQuery.IsEmpty;
                    _blobBulkQuery.SetSharedComponentFilter(plane);
                    blobBulkIsEmpty = _blobBulkQuery.IsEmpty;

                    _insertTerrainQuery.SetSharedComponentFilter(plane);
                    insertTerrainIsEmpty = _insertTerrainQuery.IsEmpty;
                    _insertTerrainBulkQuery.SetSharedComponentFilter(plane);
                    insertTerrainBulkIsEmpty = _insertTerrainBulkQuery.IsEmpty;
                }

                if (destroyIsEmpty && insertIsEmpty && insertBulkIsEmpty && blobIsEmpty && blobBulkIsEmpty && destroyTerrainIsEmpty && insertTerrainIsEmpty && insertTerrainBulkIsEmpty)
                    continue;

                var data = new NativeReference<JobData>(Allocator.TempJob);

                /* var dependency =  */new PreJob
                {
                    Plane = planeEntity,
                    Data = data,
                    NavmeshLookup = _navmeshLookup,
                    LocalToWorldLookup = _localToWorldLookup
                }.Run(); //(state.Dependency);

                var ecb = HasSingleton<RunnerSingleton>() 
                    ? GetSingletonRW<EndDotsNavEntityCommandBufferSystem.Singleton>().ValueRW.CreateCommandBuffer(state.WorldUnmanaged) 
                    : GetSingletonRW<EndFixedStepSimulationEntityCommandBufferSystem.Singleton>().ValueRW.CreateCommandBuffer(state.WorldUnmanaged);
                
                if (!destroyIsEmpty)
                { 
                    /* dependency =  */new DestroyJob
                    {
                        Data = data,
                        Ecb = ecb
                    }.Run(_destroyQuery); // .Schedule(_destroyQuery, dependency);

                    /* dependency =  */new RemoveRefinementsJob { Data = data }.Run(); // .Schedule(dependency);
                }
                
                if (!insertIsEmpty)
                {
                    // /* dependency =  */new InsertJob
                    // {
                        var Data = data;
                        var Ecb = ecb;
                    // }.Run(_insertQuery); // .Schedule(_insertQuery, dependency);

                    foreach (var (obstacle, vertices, localToWorld, entity) in SystemAPI.Query<RefRW<NavmeshObstacleComponent>, DynamicBuffer<VertexElement>, RefRO<LocalToWorld>>().WithEntityAccess()) {
                        // void Execute(Entity entity, in NavmeshObstacleComponent obstacle, in DynamicBuffer<VertexElement> vertices, in LocalToWorld localToWorld)
                        // {
                            var navmesh = Data.Value.Navmesh;
                            var ltw = math.mul(Data.Value.PlaneLtwInv, localToWorld.ValueRO.Value);
                            Ecb.AddSharedComponent(entity, new CleanUpObstacleComponent { Plane = Data.Value.Plane });

                            if (Data.Value.Empty)
                            {
                                navmesh->InsertMajor(new Span<PlanePoint>(vertices.GetUnsafeReadOnlyPtr(), vertices.Length), entity, ltw);
                            }
                            else
                            {
                                navmesh->C.Clear();
                                navmesh->InsertMajor(new Span<PlanePoint>(vertices.GetUnsafeReadOnlyPtr(), vertices.Length), entity, ltw);
                                navmesh->SearchDisturbances();
                            }
                        // }
                    }
                }

                if (!insertBulkIsEmpty)  {
                    /* dependency =  */new InsertBulkJob { Data = data }.Run(_insertBulkQuery); // .Schedule(_insertBulkQuery, dependency);
                }

                if (!blobIsEmpty)
                {
                    /* dependency =  */new BlobJob
                    {
                        Data = data,
                        Ecb = ecb
                    }.Run(_blobQuery); // .Schedule(_blobQuery, dependency);
                }
                
                if (!blobBulkIsEmpty) {
                    /* dependency =  */new BlobBulkJob { Data = data }.Run(_blobBulkQuery); // .Schedule(_blobBulkQuery, dependency);
                }

                // /* dependency =  */new PostJob
                // {
                //     Data = data,
                //     DestroyedTriangleBufferLookup = _destroyedTriangleBufferLookup,
                // }.Run(); // .Schedule(dependency);


                // PostJob, DELETE:
                // public void Execute()
                {
                    var navmesh = data.Value.Navmesh;
                    
                    if (data.Value.Empty) {
                        navmesh->GlobalRefine();
                    } else {
                        navmesh->LocalRefinement();
                    }

                    UnityEngine.Debug.Log($"InsetMajorInMinor; numMajorInMinorInsertions: {numMajorInMinorInsertions}, AddedOrModifiedMajorEdges.Length: {navmesh->AddedOrModifiedMajorEdges.Length}, OKUUUURRRRRRRRRRRRRRRRRRRRRRRRRRRRR");
                    foreach (IntPtr edge in navmesh->AddedOrModifiedMajorEdges) {
                        navmesh->InsertMajorInMinor((Edge*)edge);
                        numMajorInMinorInsertions++;
                    }
                    navmesh->AddedOrModifiedMajorEdges.Clear();


                    var destroyedTriangles = _destroyedTriangleBufferLookup[data.Value.Plane];
                    destroyedTriangles.Clear();
                    var tris = navmesh->DestroyedTriangles.GetEnumerator();
                    while (tris.MoveNext()) {
                        destroyedTriangles.Add(tris.Current);
                    }
                    destroyedTriangles.Reinterpret<int>().AsNativeArray().Sort();
                }


                if (!destroyTerrainIsEmpty)
                { 
                    // /* dependency =  */new DestroyTerrainJob
                    // {
                    //     Data = data,
                    //     Ecb = ecb
                    // }.Run(_destroyTerrainQuery); // .Schedule(_destroyTerrainQuery, dependency);

                    foreach (var (_, entity) in SystemAPI.Query<CleanUpTerrainComponent>().WithNone<NavmeshTerrainComponent>().WithEntityAccess()) {
                        data.Value.Navmesh->RemoveConstraintMinor(entity);
                        ecb.RemoveComponent<CleanUpTerrainComponent>(entity);
                    }
                }


                if (!insertTerrainIsEmpty)
                {
                    // /* dependency =  */new InsertTerrainJob
                    // {
                        var Data = data;
                        var Ecb = ecb;
                    // }.Run(_insertQuery); // .Schedule(_insertQuery, dependency);

                    foreach (var (terrain, vertices, localToWorld, entity) in SystemAPI.Query<RefRW<NavmeshTerrainComponent>, DynamicBuffer<VertexElement>, RefRO<LocalToWorld>>().WithEntityAccess()) {
                        // void Execute(Entity entity, in NavmeshTerrainComponent terrain, in DynamicBuffer<VertexElement> vertices, in LocalToWorld localToWorld)
                        // {
                            var navmesh = Data.Value.Navmesh;
                            var ltw = math.mul(Data.Value.PlaneLtwInv, localToWorld.ValueRO.Value);
                            Ecb.AddSharedComponent(entity, new CleanUpTerrainComponent { Plane = Data.Value.Plane }); // TODO: Use EntityQuery version of this?

                            if (Data.Value.Empty)
                            {
                                navmesh->InsertMinor(new Span<PlanePoint>(vertices.GetUnsafeReadOnlyPtr(), vertices.Length), entity, ltw);
                            }
                            else
                            {
                                // navmesh->C.Clear(); // Minor graph doesn't use C
                                navmesh->InsertMinorCost(new Span<PlanePoint>(vertices.GetUnsafeReadOnlyPtr(), vertices.Length), entity, 3, ltw);
                                // navmesh->SearchDisturbances(); // Minor graph doesn't have Disturbances
                            }
                        // }
                    }
                }

                if (!insertTerrainBulkIsEmpty) {
                    /* dependency =  */new InsertTerrainBulkJob { Data = data }.Run(_insertTerrainBulkQuery); // .Schedule(_insertBulkQuery, dependency);
                }
                
                //dependencies.Add(dependency);
            }
            
            // state.Dependency = JobHandle.CombineDependencies(dependencies);
        }

        // [BurstDiscard]
        struct PreJob : IJob
        {
            public Entity Plane;
            public NativeReference<JobData> Data;
            [ReadOnly] public ComponentLookup<NavmeshComponent> NavmeshLookup;
            [ReadOnly] public ComponentLookup<LocalToWorld> LocalToWorldLookup;
            [BurstDiscard]
            public void Execute()
            {
                var navmesh = NavmeshLookup[Plane].Navmesh;
                var planeLtwInv = math.inverse(LocalToWorldLookup[Plane].Value);
                navmesh->DestroyedTriangles.Clear();
                navmesh->V.Clear();

                Data.Value = new JobData
                {
                    Plane = Plane,
                    Navmesh = navmesh,
                    Empty = navmesh->IsEmpty,
                    PlaneLtwInv = planeLtwInv,
                };
            }
        }

        // [BurstDiscard]
        partial struct DestroyJob : IJobEntity
        {
            public NativeReference<JobData> Data;
            public EntityCommandBuffer Ecb;
            [BurstDiscard]
            void Execute(Entity entity)
            {
                Data.Value.Navmesh->RemoveConstraintMajor(entity);
                Ecb.RemoveComponent<CleanUpObstacleComponent>(entity);
            }
        }

        // [BurstDiscard]
        struct RemoveRefinementsJob : IJob
        {
            public NativeReference<JobData> Data;
            [BurstDiscard]
            public void Execute()
            {
                Data.Value.Navmesh->RemoveRefinements();
            }
        }
        
        // [BurstDiscard]
        partial struct InsertJob : IJobEntity
        {
            public NativeReference<JobData> Data;
            public EntityCommandBuffer Ecb;
            [BurstDiscard]
            void Execute(Entity entity, in NavmeshObstacleComponent obstacle, in DynamicBuffer<VertexElement> vertices, in LocalToWorld localToWorld)
            {
                var navmesh = Data.Value.Navmesh;
                var ltw = math.mul(Data.Value.PlaneLtwInv, localToWorld.Value);
                Ecb.AddSharedComponent(entity, new CleanUpObstacleComponent { Plane = Data.Value.Plane });

                if (Data.Value.Empty) {
                    navmesh->InsertMajor(new Span<PlanePoint>(vertices.GetUnsafeReadOnlyPtr(), vertices.Length), entity, ltw);
                } else {
                    navmesh->C.Clear();
                    navmesh->InsertMajor(new Span<PlanePoint>(vertices.GetUnsafeReadOnlyPtr(), vertices.Length), entity, ltw);
                    navmesh->SearchDisturbances();
                }
            }
        }
        
        // [BurstDiscard]
        partial struct InsertBulkJob : IJobEntity
        {
            public NativeReference<JobData> Data;
            [BurstDiscard]
            void Execute(in DynamicBuffer<VertexElement> vertices, in DynamicBuffer<VertexAmountElement> amounts, in LocalToWorld localToWorld)
            {
                var navmesh = Data.Value.Navmesh;
                var ptr = (PlanePoint*)vertices.GetUnsafeReadOnlyPtr();
                var ltw = math.mul(Data.Value.PlaneLtwInv, localToWorld.Value);
                var start = 0;
                
                if (Data.Value.Empty) {
                    foreach (var amount in amounts) {
                        navmesh->InsertMajor(new Span<PlanePoint>(ptr + start, amount), Entity.Null, ltw);
                        start += amount;
                    }
                } else {
                    foreach (var amount in amounts) {
                        navmesh->C.Clear();
                        navmesh->InsertMajor(new Span<PlanePoint>(ptr + start, amount), Entity.Null, ltw);
                        navmesh->SearchDisturbances();
                        start += amount;
                    }
                }
            }
        }
        
        // [BurstDiscard]
        partial struct BlobJob : IJobEntity
        {
            public NativeReference<JobData> Data;
            public EntityCommandBuffer Ecb;
            [BurstDiscard]
            void Execute(in Entity entity, in VertexBlobComponent blob, in LocalToWorld localToWorld)
            {
                var navmesh = Data.Value.Navmesh;
                var ltw = math.mul(Data.Value.PlaneLtwInv, localToWorld.Value);
                Ecb.AddSharedComponent(entity, new CleanUpObstacleComponent{Plane = Data.Value.Plane});
                ref var vertices = ref blob.BlobRef.Value.Vertices;

                if (Data.Value.Empty) {
                    navmesh->InsertMajor(new Span<PlanePoint>(vertices.GetUnsafePtr(), vertices.Length), entity, ltw);
                } else {
                    navmesh->C.Clear();
                    navmesh->InsertMajor(new Span<PlanePoint>(vertices.GetUnsafePtr(), vertices.Length), entity, ltw);
                    navmesh->SearchDisturbances();
                }
            }
        }
        
        // [BurstDiscard]
        partial struct BlobBulkJob : IJobEntity
        {
            public NativeReference<JobData> Data;
            [BurstDiscard]
            void Execute(in ObstacleBlobComponent blob, in LocalToWorld localToWorld)
            {
                var navmesh = Data.Value.Navmesh;
                var vertices = (PlanePoint*)blob.BlobRef.Value.Vertices.GetUnsafePtr();
                ref var a = ref blob.BlobRef.Value.Amounts;
                var ltw = math.mul(Data.Value.PlaneLtwInv, localToWorld.Value);
                var start = 0;
                
                if (Data.Value.Empty) {
                    for (var i = 0; i < a.Length; i++) {
                        var amount = a[i];
                        navmesh->InsertMajor(new Span<PlanePoint>(vertices + start, amount), Entity.Null, ltw);
                        start += amount;
                    }
                } else {
                    for (var i = 0; i < a.Length; i++) {
                        var amount = a[i];
                        navmesh->C.Clear();
                        navmesh->InsertMajor(new Span<PlanePoint>(vertices + start, amount), Entity.Null, ltw);
                        navmesh->SearchDisturbances();
                        start += amount;
                    }
                }
            }
        }

        // [BurstDiscard]
        struct PostJob : IJob // TODO: Split this into Post and Destroyed, because Destroyed needs to happen after all Terrain
        {
            public NativeReference<JobData> Data;
            [NativeDisableContainerSafetyRestriction]
            public BufferLookup<DestroyedTriangleElement> DestroyedTriangleBufferLookup;
            [BurstDiscard]
            public void Execute()
            {
                var navmesh = Data.Value.Navmesh;
                
                if (Data.Value.Empty) {
                    navmesh->GlobalRefine();
                } else {
                    navmesh->LocalRefinement();
                }

                UnityEngine.Debug.Log("OKURRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR");
                foreach (IntPtr edge in navmesh->AddedOrModifiedMajorEdges) {
                    navmesh->InsertMajorInMinor((Edge*)edge);
                }
                navmesh->AddedOrModifiedMajorEdges.Clear();


                var destroyedTriangles = DestroyedTriangleBufferLookup[Data.Value.Plane];
                destroyedTriangles.Clear();
                var tris = navmesh->DestroyedTriangles.GetEnumerator();
                while (tris.MoveNext()) {
                    destroyedTriangles.Add(tris.Current);
                }
                destroyedTriangles.Reinterpret<int>().AsNativeArray().Sort();
            }
        }

        // [BurstDiscard]
        partial struct DestroyTerrainJob : IJobEntity
        {
            public NativeReference<JobData> Data;
            public EntityCommandBuffer Ecb;
            [BurstDiscard]
            void Execute(Entity entity)
            {
                Data.Value.Navmesh->RemoveConstraintMinor(entity);
                Ecb.RemoveComponent<CleanUpTerrainComponent>(entity);
            }
        }

        // [BurstDiscard]
        partial struct InsertTerrainJob : IJobEntity
        {
            public NativeReference<JobData> Data;
            public EntityCommandBuffer Ecb;

            // Replace
        }

        // [BurstDiscard]
        partial struct InsertTerrainBulkJob : IJobEntity
        {
            public NativeReference<JobData> Data;
            [BurstDiscard]
            void Execute(in DynamicBuffer<VertexElement> vertices, in DynamicBuffer<VertexAmountElement> amounts, in LocalToWorld localToWorld)
            {
                var navmesh = Data.Value.Navmesh;
                var ptr = (PlanePoint*)vertices.GetUnsafeReadOnlyPtr();
                var ltw = math.mul(Data.Value.PlaneLtwInv, localToWorld.Value);
                var start = 0;
                
                if (Data.Value.Empty) {
                    foreach (var amount in amounts) {
                        navmesh->InsertMinor(new Span<PlanePoint>(ptr + start, amount), Entity.Null, ltw);
                        start += amount;
                    }
                } else {
                    foreach (var amount in amounts) {
                        navmesh->C.Clear();
                        navmesh->InsertMinor(new Span<PlanePoint>(ptr + start, amount), Entity.Null, ltw);
                        navmesh->SearchDisturbances();
                        start += amount;
                    }
                }
            }
        }



        struct JobData
        {
            public Entity Plane;
            public Navmesh* Navmesh;
            public bool Empty;
            public float4x4 PlaneLtwInv;
        }

        struct CleanUpObstacleComponent : ICleanupSharedComponentData
        {
            public Entity Plane;
        }

        struct CleanUpTerrainComponent : ICleanupSharedComponentData
        {
            public Entity Plane;
        }
    }
}
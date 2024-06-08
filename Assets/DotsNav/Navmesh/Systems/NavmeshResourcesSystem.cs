using DotsNav.Navmesh.Data;
using DotsNav.Systems;
using Unity.Collections;
using Unity.Entities;

namespace DotsNav.Navmesh.Systems
{
    [UpdateInGroup(typeof(DotsNavSystemGroup), OrderFirst = true)]
    unsafe partial class NavmeshResourcesSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            var ecbSource = EcbUtility.Get(World);
            var buffer = ecbSource.CreateCommandBuffer().AsParallelWriter();

            // Entities
            //     .WithBurst()
            //     .WithNone<SystemStateComponent>()
            //     .ForEach((Entity entity, int entityInQueryIndex, ref NavmeshComponent data) =>
            //     {
            //         data.Navmesh = (Navmesh*) Mem.Malloc<Navmesh>(Allocator.Persistent);
            //         *data.Navmesh = new Navmesh(data);
            //         buffer.AddComponent(entityInQueryIndex, entity, new SystemStateComponent{Navmesh = data.Navmesh});
            //     })
            //     .ScheduleParallel();

            // Entities
            //     .WithBurst()
            //     .WithNone<NavmeshComponent>()
            //     .ForEach((Entity entity, int entityInQueryIndex, SystemStateComponent state) =>
            //     {
            //         state.Navmesh->Dispose();
            //         buffer.RemoveComponent<SystemStateComponent>(entityInQueryIndex, entity);
            //     })
            //     .ScheduleParallel();
            // ecbSource.AddJobHandleForProducer(Dependency);

            
            int entityInQueryIndex = 0;
            foreach (var (data, entity) in SystemAPI.Query<RefRW<NavmeshComponent>>().WithEntityAccess().WithNone<SystemStateComponent>()) {
                data.ValueRW.Navmesh = (Navmesh*) Mem.Malloc<Navmesh>(Allocator.Persistent);
                *data.ValueRW.Navmesh = new Navmesh(data.ValueRW);
                buffer.AddComponent(entityInQueryIndex, entity, new SystemStateComponent{Navmesh = data.ValueRW.Navmesh});
                entityInQueryIndex++;
            }

            entityInQueryIndex = 0;
            foreach (var (state, entity) in SystemAPI.Query<RefRW<SystemStateComponent>>().WithEntityAccess().WithNone<NavmeshComponent>()) {
                state.ValueRW.Navmesh->Dispose();
                buffer.RemoveComponent<SystemStateComponent>(entityInQueryIndex, entity);
            }

        }

        struct SystemStateComponent : ICleanupComponentData
        {
            public Navmesh* Navmesh;
        }

        protected override void OnDestroy()
        {
            Entities
                .WithBurst()
                .ForEach((NavmeshComponent resources)
                    => resources.Navmesh->Dispose())
                .Run();
        }
    }
}
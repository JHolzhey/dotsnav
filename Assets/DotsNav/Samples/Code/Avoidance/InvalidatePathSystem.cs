using DotsNav.Data;
using DotsNav.PathFinding;
using DotsNav.PathFinding.Data;
using DotsNav.PathFinding.Systems;
using DotsNav.Systems;
using Unity.Entities;

[UpdateInGroup(typeof(DotsNavSystemGroup))]
[UpdateAfter(typeof(AgentDirectionSystem))]
partial class InvalidatePathSystem : SystemBase
{
    protected override void OnUpdate()
    {
        Entities
            .WithBurst()
            .ForEach((DirectionComponent direction, AgentComponent agent, ref PathQueryComponent query) =>
            {
                if (direction.DistanceFromPathSquared > agent * agent) // TODO: Do we even need this code? Maybe for individual soldiers?
                    query.State = PathQueryState.Invalidated;
            })
            .ScheduleParallel();
    }
}
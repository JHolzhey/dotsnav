using DotsNav.Data;
using DotsNav.Hybrid;
using Unity.Entities;

namespace DotsNav.Systems
{
    [UpdateInGroup(typeof(DotsNavSystemGroup), OrderFirst = true)]
    partial class AgentHybridReadSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            Entities
                .WithoutBurst()
                .ForEach((DotsNavAgent monoAgent, ref AgentComponent radius) =>
                {
                    radius.Radius.min = monoAgent.MinRadius;
                    radius.Radius.max = monoAgent.MaxRadius;
                })
                .Run();
        }
    }
}
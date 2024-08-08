using DotsNav.PathFinding.Data;
using DotsNav.PathFinding.Hybrid;
using DotsNav.Systems;
using Unity.Entities;
using Unity.Transforms;

namespace DotsNav.PathFinding.Systems
{
    [UpdateInGroup(typeof(DotsNavSystemGroup), OrderFirst = true)]
    partial class PathFinderHybridReadSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            Entities
                .WithoutBurst()
                .ForEach((DotsNavPathFinder pathFinder, ref PathFinderComponent d) =>
                {
                    d.RecalculateFlags = pathFinder.GetRecalculateFlags();
                })
                .Run();

            Entities
                .WithoutBurst()
                .ForEach((DotsNavPathFindingAgent hybrid, ref PathQueryComponent query, ref LocalTransform translation, ref AgentDrawComponent drawData) =>
                {
                    query.State = hybrid.State;
                    query.To = hybrid.Goal;
                    var pos = hybrid.transform.position;
                    pos.y = 0;
                    translation.Position = pos;

                    drawData.Draw = hybrid.DrawPath;
                    drawData.Delimit = hybrid.DrawCorners;
                    drawData.Color = hybrid.DrawColor;
                })
                .Run();
         }
    }
}
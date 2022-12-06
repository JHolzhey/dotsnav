﻿using DotsNav.Data;
using DotsNav.LocalAvoidance.Data;
using DotsNav.LocalAvoidance.Hybrid;
using DotsNav.Systems;
using Unity.Entities;
using Unity.Transforms;

namespace DotsNav.LocalAvoidance.Systems
{
    [UpdateInGroup(typeof(DotsNavSystemGroup), OrderFirst = true)]
    partial class LocalAvoidanceHybridReadSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            Entities
                .WithoutBurst()
                .ForEach((DotsNavLocalAvoidanceAgent monoAgent, ref LocalTransform translation, ref RVOSettingsComponent agentComponent, ref MaxSpeedComponent maxSpeed) =>
                {
                    translation.Position = monoAgent.transform.position;
                    maxSpeed.Value = monoAgent.MaxSpeed;
                    agentComponent.MaxNeighbours = monoAgent.MaxNeighbours;
                    agentComponent.NeighbourDist = monoAgent.NeighbourDist;
                    agentComponent.TimeHorizon = monoAgent.TimeHorizon;
                    agentComponent.TimeHorizonObst = monoAgent.TimeHorizon;
                })
                .Run();
        }
    }
}
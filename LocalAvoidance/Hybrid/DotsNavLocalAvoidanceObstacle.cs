﻿using DotsNav.Hybrid;
using Unity.Entities;
using UnityEngine;
using UnityEngine.Serialization;

namespace DotsNav.LocalAvoidance.Hybrid
{
    [UpdateAfter(typeof(LocalAvoidanceConversionSystem))]
    class LocalAvoidanceObstacleConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate()
        {
            Entities.ForEach((DotsNavLocalAvoidanceObstacle localAvoidance) =>
            {
                var entity = GetPrimaryEntity(localAvoidance);
                DstEntityManager.AddComponentData(entity, new ObstacleTreeElementComponent{Tree = localAvoidance.LocalAvoidance.Entity});
            });
        }
    }

    [RequireComponent(typeof(DotsNavObstacle))]
    public class DotsNavLocalAvoidanceObstacle : MonoBehaviour
    {
        public DotsNavLocalAvoidance LocalAvoidance;
    }
}
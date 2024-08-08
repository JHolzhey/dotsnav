using DotsNav.Data;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using UnityEngine.Android;

namespace DotsNav.Hybrid
{
    public class DotsNavAgent : ToEntity
    {
        public DotsNavPlane Plane;
        public float Radius {
            get => MinRadius;
            set => MinRadius = value;
        }
        public float MinRadius = .5f;
        public float MaxRadius = .5f; // Make Min and Max equal for singular soldiers

        protected override void Convert(EntityManager entityManager, Entity entity)
        {
            Assert.IsTrue(MinRadius > 0 && MaxRadius > 0, "Radius must be larger than 0");
            entityManager.AddComponentData(entity, new AgentComponent(new FloatRange(MinRadius, MaxRadius)));
            entityManager.AddComponentObject(entity, this);

            entityManager.AddComponent<LocalTransform>(entity);
        }
    }
}
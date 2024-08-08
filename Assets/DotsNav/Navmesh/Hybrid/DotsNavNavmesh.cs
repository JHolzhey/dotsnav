using DotsNav.Core.Hybrid;
using DotsNav.Drawing;
using DotsNav.Hybrid;
using DotsNav.Navmesh.Data;
using Unity.Entities;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

namespace DotsNav.Navmesh.Hybrid
{
    /// <summary>
    /// Creates a navmesh on startup and can then be used to insert and destroy obstacles. Destroying this object triggers
    /// the destruction of the navmesh releasing its resources.
    /// </summary>
    [RequireComponent(typeof(DotsNavPlane))]
    public class DotsNavNavmesh : MonoBehaviour, IPlaneComponent
    {
        /// <summary>
        /// Determines the size of initial allocations. Changing this value after initialization has no effect
        /// </summary>
        [Min(100)]
        public int ExpectedVerts = 1000;

        /// <summary>
        /// Vertices inserted with this range of an existing vertex are merged instead. Changing this value after initialization has no effect
        /// </summary>
        public float MergePointDistance = 1e-3f;

        /// <summary>
        /// Margin for considering points collinear when removing intersections. Changing this value after initialization has no effect
        /// </summary>
        public float CollinearMargin = 1e-6f;

        [Header("Debug")]
        public DrawMode DrawMode = DrawMode.Constrained;

        World _world;
        Entity _entity;

        /// <summary>
        /// The amount of vertices in the current triangulation
        /// </summary>
        public int Vertices { get; internal set; }

        public bool IsInitialized => Vertices > 7;

        void IPlaneComponent.InsertObstacle(EntityManager em, Entity plane, Entity obstacle, ConstraintType constraintType)
        {
            if (constraintType == ConstraintType.Obstacle) {
                 em.AddComponentData(obstacle, new NavmeshMajorConstraintComponent());
            } else if (constraintType == ConstraintType.Terrain) {
                em.AddComponentData(obstacle, new NavmeshMinorConstraintComponent());
            }
        }

        /// <summary>
        /// Returns the native navmesh which exposes the triangulation. This structure is invalidated each update and
        /// the latest version should be obtained each cycle
        /// </summary>
        public unsafe Navmesh GetNativeNavmesh() => *_world.EntityManager.GetComponentData<NavmeshComponent>(_entity).Navmesh;

        public void Convert(EntityManager entityManager, Entity entity)
        {
            _world = entityManager.World;
            _entity = entity;
            
            var plane = GetComponent<DotsNavPlane>();

            UnsafeList<NavmeshMaterialType> materialTypes = new UnsafeList<NavmeshMaterialType>(10, Allocator.Persistent) { // TODO: Expose to Editor
                new NavmeshMaterialType("Default", Color.gray, new (2f)),
                new NavmeshMaterialType("PavedRoad", Color.blue, new (1f)),
                new NavmeshMaterialType("Clay", Color.red, new (1.1f)),
                new NavmeshMaterialType("Grass", Color.green, new (1.1f)),
                new NavmeshMaterialType("Rock", Color.black, new (2f)),
                new NavmeshMaterialType("Ladder", Color.yellow, new (1.5f, 0.9f)),
                new NavmeshMaterialType("Test", Color.cyan, new (1.5f)),
                new NavmeshMaterialType("Test2", Color.magenta, new (1.5f)),
                new NavmeshMaterialType("Test3", Color.white, new (1.5f)),
            };

            Terrain terrain = Terrain.activeTerrain;
            float3 postScaleFactor = 0.95f * (float3)plane.Size.ToXxY(10f) / (float3)terrain.terrainData.size;
            TerrainMesh terrainMesh = new TerrainMesh(terrain.GetHeightMapData(Allocator.Temp), 0.005f, postScaleFactor);
            
            entityManager.AddComponentData(entity, new NavmeshComponent
            (
                plane.Size,
                ExpectedVerts,
                materialTypes,
                terrainMesh,
                MergePointDistance,
                CollinearMargin
            ));
            entityManager.AddBuffer<DestroyedTriangleElement>(entity);
            entityManager.AddComponentData(entity, new NavmeshDrawComponent
            {
                DrawMode = DrawMode,
                ConstrainedColor = plane.ConstrainedColor,
                UnconstrainedColor = plane.UnconstrainedColor
            });

            entityManager.AddComponentObject(entity, this);
        }
    }
}
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
                em.AddComponentData(obstacle, new NavmeshObstacleComponent());
            } else if (constraintType == ConstraintType.Terrain) {
                em.AddComponentData(obstacle, new NavmeshTerrainComponent());
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

            UnsafeList<NavmeshMaterialType> surfaceTypes = new UnsafeList<NavmeshMaterialType>(10, Allocator.Persistent) { // TODO: Expose to Editor
                new NavmeshMaterialType("Default", 2f, Color.gray),
                new NavmeshMaterialType("PavedRoad", 1f, Color.blue),
                new NavmeshMaterialType("Clay", 1.1f, Color.red),
                new NavmeshMaterialType("Grass", 1.1f, Color.green),
                new NavmeshMaterialType("Rock", 2f, Color.black),
                new NavmeshMaterialType("Ladder", 1.5f, Color.yellow, 0.1f),
                new NavmeshMaterialType("Test", 1.5f, Color.cyan),
                new NavmeshMaterialType("Test2", 1.5f, Color.magenta),
                new NavmeshMaterialType("Test3", 1.5f, Color.white),
            };

            Terrain terrain = Terrain.activeTerrain;
            float3 postScaleFactor = 0.95f * (float3)plane.Size.ToXxY(10f) / (float3)terrain.terrainData.size;
            TerrainMesh terrainMesh = new TerrainMesh(terrain.GetHeightMapData(Allocator.Temp), 0.005f, postScaleFactor);
            
            entityManager.AddComponentData(entity, new NavmeshComponent
            (
                plane.Size,
                ExpectedVerts,
                surfaceTypes,
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
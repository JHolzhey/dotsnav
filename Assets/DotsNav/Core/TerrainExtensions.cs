using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Unity.Collections.LowLevel.Unsafe;

public static class TerrainExtensions
{
    public static int[] layerToMaterialIndices;
    public static int numTerrainLayers;

    public static int SampleLayerIndex(this Terrain terrain, float3 position) {
        int2 splatMapCoords = terrain.GetSplatMapCoords(position);
        float[,,] splatMapData = terrain.terrainData.GetAlphamaps(splatMapCoords.x, splatMapCoords.y, 1, 1);
        int maxBlendLayerIndex = default;
        float maxBlendLayerStrength = -1;
        for (int i_Layer = 0; i_Layer < numTerrainLayers; i_Layer++) {
            if (splatMapData[0,0,i_Layer] > maxBlendLayerStrength) {
                maxBlendLayerIndex = i_Layer;
                maxBlendLayerStrength = splatMapData[0,0,i_Layer];
            }
        }
        return layerToMaterialIndices[maxBlendLayerIndex];
    }

    public static float3 SampleNormal(this Terrain terrain, float3 position) {
        float2 normalizedCoords = terrain.GetNormalizedCoords(position);
        return terrain.terrainData.GetInterpolatedNormal(normalizedCoords.x, normalizedCoords.y);
    }

    private static int2 GetSplatMapCoords(this Terrain terrain, float3 position) {
        float2 alphamapSize = new float2(terrain.terrainData.alphamapWidth, terrain.terrainData.alphamapHeight);
        float2 normalizedCoords = terrain.GetNormalizedCoords(position);
        float2 splatMapCoords = normalizedCoords * alphamapSize;
        return new int2((int)splatMapCoords.x, (int)splatMapCoords.y);
    }

    public static float2 GetNormalizedCoords(this Terrain terrain, float3 position) {
        TerrainData terrainData = terrain.terrainData;
        float3 terrainBottomLeft = terrain.GetPosition();
        float3 normalizedCoords = (position - terrainBottomLeft) / terrainData.size;
        
        return new float2(normalizedCoords.x, normalizedCoords.z);
    }

    public static Heightmap GetHeightMapData(this Terrain terrain, Allocator allocator) {
        TerrainData terrainData = terrain.terrainData;
        int heightmapResolution = terrainData.heightmapResolution;
        float[,] heights = terrainData.GetHeights(0, 0, heightmapResolution, heightmapResolution);

        Native2DArray<float> heightmapData = new Native2DArray<float>(heightmapResolution, heightmapResolution, allocator);

        for (int x = 0; x < heightmapResolution; x++) {
            for (int y = 0; y < heightmapResolution; y++) {
                heightmapData[x, y] = heights[y, x];
            }
        }
        return new Heightmap(heightmapData.LengthX, heightmapData.LengthY, heightmapData.flatArray, terrain.terrainData.heightmapScale, terrain.GetPosition());
    }

    /* public static Heightmap GetNormalData(this Terrain terrain, Allocator allocator) {
        TerrainData terrainData = terrain.terrainData;
        int heightmapResolution = terrainData.heightmapResolution;
        float[,] heights = terrainData.GetHeights(0, 0, heightmapResolution, heightmapResolution);

        Native2DArray<float> heightmapData = new Native2DArray<float>(heightmapResolution+1, heightmapResolution+1, allocator);

        for (int x = 0; x < heightmapResolution; x++) {
            
            for (int y = 0; y < heightmapResolution; y++) {
                heightmapData[x, y] = heights[y, x];
            }
        }
        float3 heightmapScale = terrain.terrainData.heightmapScale;
        float3 offset = (float3)terrain.GetPosition() + heightmapScale.XOZ()/2f;
        return new Heightmap(heightmapData.LengthX, heightmapData.LengthY, heightmapData.flatArray, heightmapScale, offset);
    } */

    public static void SimplifyTerrainMesh(this Terrain terrain, float maxError, float3 scaleFactor, out UnsafeList<float3> points, out UnsafeList<int3> triangles) {
        Triangulator triangulator = new Triangulator(terrain.GetHeightMapData(Allocator.Temp));
        triangulator.Run(maxError, int.MaxValue, int.MaxValue);

        points = triangulator.Points(scaleFactor);
        triangles = triangulator.Triangles();
    }


        // float3 center = new float3(heightmapResolution/2, -100, heightmapResolution/2 + 40) * heightmapScale + terrainPositionOffset;
        // verts.Add(new MyVector3(center.x, center.y, center.z));
        // norms.Add(new MyVector3(1f,0f,0f));
        // int a = verts.Count;
        // for (int x = 0; x < heightmapResolution; x++) {
        //     for (int y = 0; y < heightmapResolution; y++) {
        //         // Add each new vertex in the plane
        //         float3 newVertexPosition = new float3(x, heightMap[y, x], y + 40) * heightmapScale + terrainPositionOffset;
        //         verts.Add(new MyVector3(newVertexPosition.x, newVertexPosition.y, newVertexPosition.z));
        //         norms.Add(new MyVector3(1f,0f,0f));
        //         CommonLib.CreatePrimitive(PrimitiveType.Cube, newVertexPosition, new float3(0.2f), Color.red);
        //         // Skip if a new square on the plane hasn't been formed
        //         if (x == 0 || y == 0) continue;
        //         // Adds the index of the three vertices in order to make up each of the two tris
        //         tris.Add(a + heightmapResolution * x + y); //Top right
        //         tris.Add(a + heightmapResolution * x + y - 1); //Bottom right
        //         tris.Add(a + heightmapResolution * (x - 1) + y - 1); //Bottom left - First triangle
        //         tris.Add(a + heightmapResolution * (x - 1) + y - 1); //Bottom left 
        //         tris.Add(a + heightmapResolution * (x - 1) + y); //Top left
        //         tris.Add(a + heightmapResolution * x + y); //Top right - Second triangle
        //     }
        // }

        // for (int x = 0; x < heightmapResolution; x++) {
        //     for (int y = 0; y < heightmapResolution; y++) {
        //         if (x == 1) {
        //             float3 newVertexPosition = new float3(x, heightMap[y, x], y + 40) * heightmapScale + terrainPositionOffset;
        //             verts.Add(new MyVector3(newVertexPosition.x, newVertexPosition.y, newVertexPosition.z));
        //             tris.Add(0);
        //             tris.Add(1 + heightmapResolution * (x - 1) + y); //Top left
        //             tris.Add(1 + heightmapResolution * (x - 1) + y - 1); //Bottom left
        //         }
        //         if (x == heightmapResolution - 1) {
        //             tris.Add(0);
        //             tris.Add(1 + heightmapResolution * x + y - 1); //Bottom right
        //             tris.Add(1 + heightmapResolution * x + y); //Top right
        //         }
        //         if (y == 1) {
        //             tris.Add(0);
        //             tris.Add(1 + heightmapResolution * (x - 1) + y - 1); //Bottom left
        //             tris.Add(1 + heightmapResolution * x + y - 1); //Bottom right
        //         }
        //         if (y == heightmapResolution - 1) {
        //             tris.Add(0);
        //             tris.Add(1 + heightmapResolution * x + y); //Top right
        //             tris.Add(1 + heightmapResolution * (x - 1) + y); //Top left
        //         }
        //     }
        // }

        // CommonLib.CreatePrimitive(PrimitiveType.Cube, new float3(verts[0].x, verts[0].y, verts[0].z), new float3(0.3f), Color.blue);
        // CommonLib.CreatePrimitive(PrimitiveType.Cube, new float3(verts[heightmapResolution - 1].x, verts[heightmapResolution - 1].y, verts[heightmapResolution - 1].z), new float3(0.3f), Color.yellow);
        // CommonLib.CreatePrimitive(PrimitiveType.Cube, new float3(verts[heightmapResolution * heightmapResolution - 1].x, verts[heightmapResolution * heightmapResolution - 1].y, verts[heightmapResolution * heightmapResolution - 1].z), new float3(0.3f), Color.green);
        // CommonLib.CreatePrimitive(PrimitiveType.Cube, new float3(verts[heightmapResolution * heightmapResolution - heightmapResolution ].x, verts[heightmapResolution * heightmapResolution - heightmapResolution ].y, verts[heightmapResolution * heightmapResolution - heightmapResolution ].z), new float3(0.3f), Color.magenta);
}
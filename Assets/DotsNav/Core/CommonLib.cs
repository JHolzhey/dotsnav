using System;
using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Rendering;
using static System.Runtime.CompilerServices.MethodImplOptions;
using MI = System.Runtime.CompilerServices.MethodImplAttribute;

public static class CommonLib
{
    // public static Entity CreatePolygonPrefab(Material material, Mesh mesh) {
    //     EntityManager entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;

    //     RenderMeshDescription desc = new RenderMeshDescription(shadowCastingMode: ShadowCastingMode.Off, receiveShadows: false);
    //     RenderMeshArray renderMeshArray = new RenderMeshArray(new Material[] { material }, new Mesh[] { mesh });

    //     Entity prefab = entityManager.CreateEntity(); // Create empty base entity

    //     // Call AddComponents to populate base entity with the components required by Entities Graphics
    //     RenderMeshUtility.AddComponents(prefab, entityManager, desc, renderMeshArray, MaterialMeshInfo.FromRenderMeshArrayIndices(0, 0));
    //     entityManager.AddComponentData(prefab, LocalTransform.FromPosition(float3.zero));

    //     return prefab;
    // }

    public static void Assert(bool assertion) {}
    
    // public static GameObject ObjectBetween2Points(float3 point1, float3 point2, PrimitiveType primitiveType) {
    //     return ObjectBetween2Points(point1, point2, GameObject.CreatePrimitive(primitiveType));
    // }

    public static GameObject ObjectBetween2Points(float3 point1, float3 point2, GameObject obj) {
        Debug.Assert(!MathLib.IsNaN(point1) && !MathLib.IsNaN(point2), "Object Between - NaN input");
        float3 vector = point2 - point1;
        float3 vectorUnit = math.normalizesafe(vector, math.up());
        quaternion rotation = MathLib.CalcRotationFromNormal(vectorUnit);
        
        obj.transform.rotation = rotation;
        obj.transform.position = MathLib.Midpoint(point1, point2);
        obj.Resize(new float3(obj.transform.localScale.x, math.length(vector), obj.transform.localScale.z));
        // obj.transform.localScale = new float3(obj.transform.localScale.x, math.length(vector), obj.transform.localScale.z);
        return obj;
    }

    public static GameObject CreatePrimitive(PrimitiveType primitiveType, float3 position, float3 localScale, Color color, quaternion localRotation = new quaternion(), float destroyTime = math.INFINITY) {
        GameObject primitive = GameObject.CreatePrimitive(primitiveType);
        primitive.transform.position = position;
        primitive.transform.localScale = localScale;
        primitive.transform.localRotation = localRotation;
        primitive.GetComponent<Renderer>().material.color = color;
        if (destroyTime != math.INFINITY) {
            UnityEngine.Object.Destroy(primitive, destroyTime);
        }
        return primitive;
    }
    // public static GameObject CreateDebugUI(float number, float3 startPosition, float destroyTime = math.INFINITY) {
    //     return GameObject.Find("WorldSpaceCanvas").GetComponent<WorldSpaceUIController>().DisplayDebugIcon(number, startPosition, destroyTime);
    // }

    // public static void CreateExactCapsule(float3 start, float3 end, float radius, Color color, float destroyTime = math.INFINITY) {
    //     CreatePrimitive(PrimitiveType.Sphere, start, new float3(radius*2f), color, default, destroyTime);
    //     CreatePrimitive(PrimitiveType.Sphere, end, new float3(radius*2f), color, default, destroyTime);
    //     ObjectBetween2Points(start, end, CreatePrimitive(PrimitiveType.Cylinder, end, new float3(radius*2f), color, default, destroyTime));
    // }

    // public static void DebugMatrixVector(float4x4 mat, float scale = 0.025f, float destroyTime = math.INFINITY) {
    //     DebugVector(mat.Translation(), mat.Up(), Color.red, scale, destroyTime);
    //     DebugVector(mat.Translation(), mat.Right(), Color.blue, scale, destroyTime);
    //     DebugVector(mat.Translation(), mat.Forward(), Color.green, scale, destroyTime);
    // }
    public static GameObject DebugVector(float3 start, float3 vector, Color color, float scale = 0.025f, float destroyTime = math.INFINITY) {
        Debug.Assert(!MathLib.IsNaN(start), "Vector - start NaN input");
        Debug.Assert(!MathLib.IsNaN(vector), "Vector - vector NaN input");
        if (math.lengthsq(vector) == 0f) {
            return CreatePrimitive(PrimitiveType.Cube, start, new float3(scale*2f), color, default, destroyTime);
        }
        GameObject vectorPrimitive = CreatePrimitive(PrimitiveType.Cylinder, float3.zero, new float3(scale), color, default, destroyTime);
        float3 end = start + vector;
        ObjectBetween2Points(start, start + vector, vectorPrimitive);

        GameObject endPrimitive = CreatePrimitive(PrimitiveType.Cube, end, new float3(scale*2f), color, MathLib.CalcRotationFromNormal(vector), destroyTime);
        return vectorPrimitive;
    }
    // public static void DebugPolygon(NativeArray<float3> vertexPoints, Color color, float planeOffset = 0, float scale = 0.025f, bool isSwitchWise = false, float destroyTime = math.INFINITY) {
    //     DebugPolygon(new Polygon(vertexPoints), color, planeOffset, scale, isSwitchWise, destroyTime);
    // }
    // public static void DebugPolygon(Polygon polygon, Color color, float planeOffset = 0, float scale = 0.025f, bool isSwitchWise = false, float destroyTime = math.INFINITY) {
    //     Debug.Assert(!MathLib.IsNaN(polygon.Plane.normal) && !float.IsNaN(polygon.Plane.distance) && !float.IsNaN(planeOffset), "Polygon - NaN input");
    //     for (int i = 0; i < polygon.NumVertices; i++) {
    //         float3 vector = polygon.GetEdgeVectorWise(i, isSwitchWise);
    //         Debug.Assert(!MathLib.IsNaN(vector), "Polygon - edge vector NaN");
    //         if (math.length(vector) > 0f) {
    //             vector -= math.normalizesafe(vector, float3.zero)*scale*8f;
    //         } else {
    //             Debug.Log("DebugPolygon - Zero length vector");
    //         }
    //         DebugVector(polygon.GetVertexPointWise(i, isSwitchWise)+polygon.Plane.normal*planeOffset, vector, color, scale, destroyTime);
    //     }
    // }
    // public static void DebugPlane(float3 planeNormal, float planeDistance, Color color, float width = 3f, float planeOffset = 0, float thickness = 0.025f, float destroyTime = math.INFINITY) {
    //     DebugPlane(planeNormal*planeDistance, planeNormal, color, width, planeOffset, thickness, destroyTime);
    // }
    // public static void DebugPlane(float3 planeOrigin, float3 planeNormal, Color color, float width = 3f, float planeOffset = 0, float thickness = 0.025f, float destroyTime = math.INFINITY) {
    //     Debug.Assert(!MathLib.IsNaN(planeOrigin) && !MathLib.IsNaN(planeNormal) && !float.IsNaN(planeOffset), "Plane - NaN input");
    //     quaternion rotation = MathLib.CalcRotationFromNormal(planeNormal);
    //     color.a = 0.5f;
    //     CreatePrimitive(PrimitiveType.Cube, planeOrigin + planeNormal*planeOffset, new float3(width, thickness, width), color, rotation, destroyTime);
    // }

    // public static void DebugLogArray<T>(NativeArray<T> array, string arrayName, string prefix = "") where T : unmanaged {
    //     string debugString = prefix + " - " + arrayName;
    //     for (int i = 0; i < array.Length; i++) {
    //         debugString += "  ["+ i +"]: " + array[i] + ",  ";
    //     }
    //     Debug.Log(debugString);
    // }

    // // Unused and move this to MathLib
    // // (lengthVector, instanceList, increment, cloneInstance, startCFrame, parent, size, isRespaceBtw)
    // public static float[] SpaceOut(float3 lengthVector, float spacingIncrement, bool isRespaceBtw) {
    //     float length = math.length(lengthVector);
    //     int numSpaces = (int)math.floor(length / spacingIncrement);


    //     // local length = lengthVector.Magnitude
    //     // if length <= 0 then print("Shud be working") lengthVector = Vector3.new(0,1,0) end
    //     // local amount = math.floor(length / increment)
    //     // local divide = amount
        
    //     // if amount <= 0 then divide = 1 end
    //     // increment = isRespaceBtw and length / divide or increment

    //     return new float[3];
    // }

    public static void Resize(this GameObject obj, float3 size) {
        Mesh mesh = obj.GetComponent<MeshFilter>().sharedMesh;
        Bounds meshBounds = mesh.bounds;
        float3 localScale = size / meshBounds.size;

        obj.transform.localScale = localScale;
    }

    // public static void RangeColor(Color color, int rangeCount) { // TODO: Probably delete this
    //     color
    //     Mesh mesh = obj.GetComponent<MeshFilter>().sharedMesh;
    //     Bounds meshBounds = mesh.bounds;
    //     float3 localScale = size / meshBounds.size;

    //     obj.transform.localScale = localScale;
    // }

    public static Color[] CycleColors = { Color.blue, Color.cyan, Color.green, Color.yellow, Color.red };
}


public static class MiscExtensions
{
    public static float3 ViewDirection(this Camera camera) {
        return camera.transform.forward;
    }

    const int s_ByteSize = 1;
    const int s_UIntSize = 4;
    const int s_ULongSize = 8;
    static MiscExtensions() {
        Debug.Assert(s_ByteSize == UnsafeUtility.SizeOf<byte>()); // TODO: These are gauranteed so delete
        Debug.Assert(s_UIntSize == UnsafeUtility.SizeOf<uint>());
        Debug.Assert(s_ULongSize == UnsafeUtility.SizeOf<ulong>());
    }
    
    // Private Extensions:
    [MI(AggressiveInlining)] unsafe static byte ToByte<TEnum>(ref this TEnum t) where TEnum : struct, Enum {
        void* enumPtr = UnsafeUtility.AddressOf(ref t);
        return *(byte*)enumPtr;
    }
    [MI(AggressiveInlining)] unsafe static uint ToUInt<TEnum>(ref this TEnum t) where TEnum : struct, Enum {
        void* enumPtr = UnsafeUtility.AddressOf(ref t);
        return *(uint*)enumPtr;
    }
    [MI(AggressiveInlining)] unsafe static ulong ToULong<TEnum>(ref this TEnum t) where TEnum : struct, Enum {
        void* enumPtr = UnsafeUtility.AddressOf(ref t);
        return *(ulong*)enumPtr;
    }

    // Public Extensions:
    [MI(AggressiveInlining)] public static bool HasAnyFlagsB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ByteSize, "Enum is not a byte");
        return (lhs.ToByte() & rhs.ToByte()) > 0;
    }
    [MI(AggressiveInlining)] public static bool HasAnyFlagsI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_UIntSize, "Enum is not an int");
        return (lhs.ToUInt() & rhs.ToUInt()) > 0;
    }
    [MI(AggressiveInlining)] public static bool HasAnyFlagsL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ULongSize, "Enum is not a long");
        return (lhs.ToULong() & rhs.ToULong()) > 0;
    }

    [MI(AggressiveInlining)] public static bool HasNoFlagsB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ByteSize, "Enum is not a byte");
        return (lhs.ToByte() & rhs.ToByte()) == 0;
    }
    [MI(AggressiveInlining)] public static bool HasNoFlagsI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_UIntSize, "Enum is not an int");
        return (lhs.ToUInt() & rhs.ToUInt()) == 0;
    }
    [MI(AggressiveInlining)] public static bool HasNoFlagsL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ULongSize, "Enum is not a long");
        return (lhs.ToULong() & rhs.ToULong()) == 0;
    }

    [MI(AggressiveInlining)] public static bool HasAllFlagsB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ByteSize, "Enum is not a byte");
        byte flags = rhs.ToByte();
        return (lhs.ToByte() & flags) == flags;
    }
    [MI(AggressiveInlining)] public static bool HasAllFlagsI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_UIntSize, "Enum is not an int");
        uint flags = rhs.ToUInt();
        return (lhs.ToUInt() & flags) == flags;
    }
    [MI(AggressiveInlining)] public static bool HasAllFlagsL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ULongSize, "Enum is not a long");
        ulong flags = rhs.ToULong();
        return (lhs.ToULong() & flags) == flags;
    }

    [MI(AggressiveInlining)] public static bool HasOnlyFlagsB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ByteSize, "Enum is not a byte");
        byte flags = rhs.ToByte();
        return (lhs.ToByte() | flags) == flags;
    }
    [MI(AggressiveInlining)] public static bool HasOnlyFlagsI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_UIntSize, "Enum is not an int");
        uint flags = rhs.ToUInt();
        return (lhs.ToUInt() | flags) == flags;
    }
    [MI(AggressiveInlining)] public static bool HasOnlyFlagsL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ULongSize, "Enum is not a long");
        ulong flags = rhs.ToULong();
        return (lhs.ToULong() | flags) == flags;
    }


    [MI(AggressiveInlining)] public static int MaskB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ByteSize, "Enum is not a byte");
        return lhs.ToByte() & rhs.ToByte();
    }
    [MI(AggressiveInlining)] public static bool MaskI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_UIntSize, "Enum is not an int");
        return (lhs.ToUInt() & rhs.ToUInt()) > 0;
    }
    [MI(AggressiveInlining)] public static bool MaskL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ULongSize, "Enum is not a long");
        return (lhs.ToULong() & rhs.ToULong()) > 0;
    }




    // public static bool HasAnyFlags<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum { // HasFlag
    //     int size = UnsafeUtility.SizeOf<TEnum>();
    //     if (size == s_UIntSize) {
    //         return (ToUInt(lhs) & ToUInt(rhs)) > 0;
    //     } else if (size == s_ULongSize) {
    //         return (ToULong(lhs) & ToULong(rhs)) > 0;
    //     } else if (size == s_ByteSize) {
    //         return (ToByte(lhs) & ToByte(rhs)) > 0;
    //     }
    //     throw new Exception($"No matching conversion function found for an Enum of size: {size}");
    // }
    // public static bool HasNoFlags<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum { // HasFlag
    //     int size = UnsafeUtility.SizeOf<TEnum>();
    //     if (size == s_UIntSize) {
    //         return (ToUInt(lhs) & ToUInt(rhs)) == 0;
    //     } else if (size == s_ULongSize) {
    //         return (ToULong(lhs) & ToULong(rhs)) == 0;
    //     } else if (size == s_ByteSize) {
    //         return (ToByte(lhs) & ToByte(rhs)) == 0;
    //     }
    //     throw new Exception($"No matching conversion function found for an Enum of size: {size}");
    // }
    // public static bool HasAllFlags<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
    //     int size = UnsafeUtility.SizeOf<TEnum>();
    //     if (size == s_UIntSize) {
    //         uint flags = ToUInt(rhs);
    //         return (ToUInt(lhs) & flags) == flags;
    //     } else if (size == s_ULongSize) {
    //         ulong flags = ToULong(rhs);
    //         return (ToULong(lhs) & flags) == flags;
    //     } else if (size == s_ByteSize) {
    //         byte flags = ToByte(rhs);
    //         return (ToByte(lhs) & flags) == flags;
    //     }
    //     throw new Exception($"No matching conversion function found for an Enum of size: {size}");
    // }
    // public static bool HasOnlyFlags<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
    //     int size = UnsafeUtility.SizeOf<TEnum>();
    //     if (size == s_UIntSize) {
    //         uint flags = ToUInt(rhs);
    //         return (ToUInt(lhs) | flags) == flags;
    //     } else if (size == s_ULongSize) {
    //         ulong flags = ToULong(rhs);
    //         return (ToULong(lhs) | flags) == flags;
    //     } else if (size == s_ByteSize) {
    //         byte flags = ToByte(rhs);
    //         return (ToByte(lhs) | flags) == flags;
    //     }
    //     throw new Exception($"No matching conversion function found for an Enum of size: {size}");
    // }
}
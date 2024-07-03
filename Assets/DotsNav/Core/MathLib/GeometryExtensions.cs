using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using System.Linq;


namespace Unity.Mathematics
{
public static class GeometryExtensions
{
    const float ValidDistanceFromOrigin = 10000f;

    public static bool IsPhysicallyValid(this float3 value) {
        Debug.Assert(math.all(!math.isnan(value)), $"float3 is NaN: {value}");
        Debug.Assert(math.all(value < ValidDistanceFromOrigin) && math.all(value > -ValidDistanceFromOrigin), $"float3 is too large: {value}");
        return math.all(!math.isnan(value)) && math.all(value < ValidDistanceFromOrigin) && math.all(value > -ValidDistanceFromOrigin);
    }
    public static bool IsPhysicallyValid(this float2 value) {
        Debug.Assert(math.all(!math.isnan(value)), $"float3 is NaN: {value}");
        Debug.Assert(math.all(value < ValidDistanceFromOrigin) && math.all(value > -ValidDistanceFromOrigin), $"float3 is too large: {value}");
        return math.all(!math.isnan(value)) && math.all(value < ValidDistanceFromOrigin) && math.all(value > -ValidDistanceFromOrigin);
    }
    public static bool IsPhysicallyValid(this float value) {
        Debug.Assert(!math.isnan(value), $"float is NaN: {value}");
        Debug.Assert(value < ValidDistanceFromOrigin && value > -ValidDistanceFromOrigin, $"float is too large: {value}");
        return !math.isnan(value) && value < ValidDistanceFromOrigin && value > -ValidDistanceFromOrigin;
    }

    public static float4x4 CalcInvLocalMatrix<TTransform>(this TTransform transform) where TTransform : IGeometry {
        return math.inverse(transform.CalcLocalMatrix());
    }
    public static void CalcLocalMatrices<TTransform>(this TTransform transform, out float4x4 localMatrix, out float4x4 invLocalMatrix) where TTransform : IGeometry {
        localMatrix = transform.CalcLocalMatrix();
        invLocalMatrix = math.inverse(localMatrix);
    }
}
}
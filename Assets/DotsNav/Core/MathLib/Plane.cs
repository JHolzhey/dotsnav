using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;


namespace Unity.Mathematics
{
public struct Plane : IComponentData, IGeometry
{
    public float3 normal; // Could make these private setter properties
    public float distance;
    public readonly float3 Center => float3.zero;
    public readonly float4x4 CalcLocalMatrix() => MathLib.CalcLocalMatrix(normal, float3.zero);
    public bool IsNormalUp { get {
        Debug.Assert(math.dot(normal, math.up()) == normal.y, $"{math.dot(normal, math.up())}, {normal.y}, This should be obivous TODO: Replace with =>");
        return normal.y > 0; }
    }

    public readonly bool IsVertical => MathLib.IsVectorXZPlane(normal);

    public readonly float SlopeAngle => MathLib.PlaneSlopeAngle(normal);

    public Plane(float3 normal, float3 arbitraryPointOnPlane) : this(normal, MathLib.CalcPlaneDistance(normal, arbitraryPointOnPlane)) {}
    public Plane(float3 normal, float distance) {
        this.normal = normal;
        this.distance = distance;
    }

    public void Update(float3 normal, float distance) {
        this.normal = normal;
        this.distance = distance;
    }

    public bool IsRayCollide(float3 origin, float3 direction, float length, out float distanceAlongRay, out float3 nearestPointToPlane, float planeOffset = 0f) {
        return MathLib.IsRayPlaneIntersecting(origin, direction, length, normal, distance + planeOffset, out distanceAlongRay, out nearestPointToPlane);
    }

    public bool IsLineCollide(float3 origin, float3 direction, out float distanceAlongRay, out float3 nearestPointToPlane, float planeOffset = 0f) {
        return MathLib.IsLinePlaneIntersecting(origin, direction, normal, distance + planeOffset, out distanceAlongRay, out nearestPointToPlane);
    }

    public bool IsVectorTowardsFront(float3 vector) => MathLib.IsVectorTowardsNormal(vector, normal);

    public bool IsPointInFront(float3 point) => MathLib.IsPointInFrontOfPlane(point, normal, distance);

    public float ToPointDistanceSigned(float3 point) => MathLib.PointToPlaneDistanceSigned(point, normal, distance);
    public float ToPointDistance(float3 point) => MathLib.PointToPlaneDistance(point, normal, distance);
    public float3 ProjPoint(float3 point) => MathLib.ProjPointToPlane(point, normal, distance);

    public float SampleElevation(float3 positionXZ) => MathLib.SamplePlaneElevation(positionXZ, normal, distance);
    public float SampleElevation(float2 position) => MathLib.SamplePlaneElevation(position.XOY(), normal, distance);

    public bool IsValid() => normal.IsPhysicallyValid() && distance.IsPhysicallyValid();
}
}
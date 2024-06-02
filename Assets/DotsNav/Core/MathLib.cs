using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Diagnostics;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Transforms;
using Unity.Mathematics;
using UnityEngine;
using Debug = UnityEngine.Debug;
using static System.Runtime.CompilerServices.MethodImplOptions;
using MI = System.Runtime.CompilerServices.MethodImplAttribute;
using Unity.Entities.UniversalDelegates;



// public static void SetupParent<T>(ref T commands, Entity parent, Entity child, in LocalToWorld parentLocalToWorld, in LocalTransform childLocalTransform, DynamicBuffer<Child> childs)
//     where T : IEntityCommands
// {
//     // Setup the child
//     commands.Entity = child;
//     commands.AddComponent(new ComponentTypeSet(ComponentType.ReadWrite<Parent>(), ComponentType.ReadWrite<PreviousParent>()));
//     commands.SetComponent(new Parent { Value = parent });
//     commands.SetComponent(new PreviousParent { Value = parent });
//     commands.SetComponent(new LocalToWorld { Value = math.mul(parentLocalToWorld.Value, childLocalTransform.ToMatrix()) });

//     // Setup the parent
//     commands.Entity = parent;
//     if (childs.IsCreated) {
//         commands.AppendToBuffer(new Child { Value = child });
//     } else {
//         commands.AddBuffer<Child>().Add(new Child { Value = child });
//     }
// }


namespace Unity.Mathematics
{
public struct AsByte {
    byte compressedAsByte;
    
    public AsByte(float value, float maxFloatValue) { // maxFloatValue might be 2 * math.PI, to compress an angle
        Debug.Assert(value <= maxFloatValue, "value cannot be greater than max, otherwise compression will be wrong");
        this.compressedAsByte = (byte)(value * (byte.MaxValue/maxFloatValue) + 0.5f);
    }
    public float ToFloat(float maxFloatValue) => (float)compressedAsByte * (maxFloatValue/byte.MaxValue);
    

    // public AsByteQuake(float value, float maxFloatValue) { // maxFloatValue might be 2 * math.PI, to compress an angle
    //     Debug.Assert(value <= maxFloatValue, "value cannot be greater than max, otherwise compression will be wrong");
    //     this.compressedAsByte = (byte)(((int)value * 256/360) & 255);
    // }
    // public float ToFloatQuake(float maxFloatValue) => compressedAsByte * (maxFloatValue/256); // Quake method


    // public AsByte(float value, FloatRange floatRange) {
    //     float compressedFloat = MathLib.Remap(floatRange.min, floatRange.max, 0, 255.999f, value);
    //     compressedAsByte = (byte)compressedFloat;
    // }
    // public AsByte(int value, IntRange intRange) {}

    public static implicit operator byte(AsByte asByte) => asByte.compressedAsByte;
}

[StructLayout(LayoutKind.Explicit)]
public struct IntFloatUnion {
    [FieldOffset(0)] public int intValue;
    [FieldOffset(0)] public float floatValue;
    public IntFloatUnion(int value) { this.floatValue = 0; this.intValue = value; }
    public IntFloatUnion(float value) { this.intValue = 0; this.floatValue = value; }
    
    public static implicit operator IntFloatUnion(float @float) => new IntFloatUnion(@float);
    public static implicit operator float(IntFloatUnion floatU) => floatU.floatValue;

    public static implicit operator IntFloatUnion(int @int) => new IntFloatUnion(@int);
    public static implicit operator int(IntFloatUnion intU) => intU.intValue;
}
public struct FloatWithFriend<TFriend> where TFriend : unmanaged {
    public readonly float Float;
    public readonly TFriend Friend;
    public FloatWithFriend(float Float, TFriend Friend) { this.Float = Float; this.Friend = Friend; }
}
[Serializable] public struct FloatRange {
    public float min; public float max;
    public FloatRange(float min, float max) { this.min = min; this.max = max; }
    public readonly float Mid => (max - min)/2f;
}
[Serializable] public struct IntRange {
    public int min; public int max;
    public IntRange(int min, int max) { this.min = min; this.max = max; }
    public readonly int Mid => (max - min)/2;
}
[Serializable] public struct IntRect {
    public int xMin; public int yMin; public int xMax; public int yMax;
    public IntRect(int xMin, int yMin, int xMax, int yMax) {
        this.xMin = xMin; this.yMin = yMin; this.xMax = xMax; this.yMax = yMax;
        Debug.Assert(xMin <= xMax && yMin <= yMax, "Invalid IntRect");
    }
    public IntRect(int2 bottomLeft, int2 topRight) : this(bottomLeft.x, bottomLeft.y, topRight.x, topRight.y) {}
    public readonly int Area() => (xMax - xMin + 1) * (yMax - yMin + 1);
}
public static class MathExtensions {
    public static float3 NextCirclePointXZ(this ref Random random, float3 center, float radius) {
        float randomRadius = random.NextFloat(0, radius);
        float2 randomDirection = random.NextFloat2Direction();
        return center + new float3(randomDirection.x, 0, randomDirection.y)*randomRadius;
    }
    public static float3 XOZ(this float3 float3, float y = 0f) { return new float3(float3.x, y, float3.z); }
    public static float3 XOY(this float2 float2, float y = 0f) { return new float3(float2.x, y, float2.y); }
    public static float3 XYO(this float2 float2, float z = 0f) { return new float3(float2.x, float2.y, z); }
    public static float3 OXY(this float2 float2, float x = 0f) { return new float3(x, float2.x, float2.y); }
    public static Color ToColor(this float3 color, float a = 1f) { return new Color(color.x, color.y, color.z, a); }
    public static Color ToColor(this float4 color) { return new Color(color.x, color.y, color.z, color.w); }
    public static float4 ToFloat4(this Color color) { return new float4(color.r, color.g, color.b, color.a); }
}
public static class MathLib
{
    public enum VectorComponent { x, y, z, w }
    public readonly static int XIndex = (int)VectorComponent.x;
    public readonly static int YIndex = (int)VectorComponent.y;
    public readonly static int ZIndex = (int)VectorComponent.z;
    public readonly static float3 XMask = new float3(1,0,0);
    public readonly static float3 YMask = new float3(0,1,0);
    public readonly static float3 ZMask = new float3(0,0,1);
    public readonly static float3 XZMask = new float3(1,0,1);
    public readonly static float3 XYMask = new float3(1,1,0);
    public readonly static float3 YZMask = new float3(0,1,1);

    public const float Cos45 = 0.70710678118f; //math.cos(math.PI/4f);
    public const float Dot45 = Cos45;
    public const float Rad45 = math.PI/4f;
    public const float Rad90 = math.PI/2f;
    public const float Rad180 = math.PI;
    public const float Rad270 = math.PI * 3f / 2f;
    public const float Rad360 = math.PI*2f;

    // public static float Mod(float a, float n) { return a - math.floor(a/n) * n; } // Use math.fmod instead
    // TODO: This probalby doesn't work:
    public static float VectorUpPercentage(float3 vector) { Debug.Assert(IsNormalized(vector, 0.001f)); return math.asin(vector.y)/Rad90; }

    // Normalizes any number to an arbitrary range by assuming the range wraps around when going below min or above max 
    public static float NormalizeToRange(float value, float start, float end)  {
        float widthDivisor = end - start;
        float offsetValue  = value - start; // value relative to 0
        return offsetValue - (math.floor(offsetValue / widthDivisor) * widthDivisor) + start; // + start to reset back to start of original range
    }

    public static float invlerp(float start, float end, float value) { return (value - start) / (end - start); } // unclamped, so can extrapolate

    // Remaps a value in range fromStart-fromEnd to value in range toStart-toEnd
    public static float Remap(float fromStart, float fromEnd, float toStart, float toEnd, float value) { // unclamped, so can extrapolate
        float t = invlerp(fromStart, fromEnd, value);
        return math.lerp(toStart, toEnd, t);
    }

    ///! returns true if not first
    public static bool LogicalIf(bool first, bool second) => !(first && !second);
    
//     public unsafe static bool DebugCheckNull(void* data) {
// #if UNITY_ASSERTIONS
//         if (data == null) {
//             Debug.Assert(false, "Attempting to call function when data is null. Prevented a segfault");
//             return false;
//         }
// #endif
//         return true;
//     }


    [MI(AggressiveInlining)] public static int signNo0(int f) => f >= 0 ? 1 : -1;
    [MI(AggressiveInlining)] public static int signNo0(float f) => f >= 0f ? 1 : -1;
    [MI(AggressiveInlining)] public static float fsignNo0(float f) => f >= 0f ? 1f : -1f;
    [MI(AggressiveInlining)] public static float fsignBool(bool b) => b ? 1f : -1f;

    // Signed Modulo: since C# % is not a true modulo, and actually a remainder
    public static int smod(int index, int divisor) => ((index % divisor) + divisor) % divisor;
    public static float smod2(float value, float divisor) => ((value % divisor) + divisor) % divisor; // TODO: Compare performance
    public static float smod(float value, float divisor) => value - math.floor(value / divisor) * divisor;
    public static float AnglesSeparationDeg(float angle0, float angle1) => smod(angle0 - angle1 + 180, 360) - 180;
    public static float AnglesSeparation(float angle0, float angle1) => smod(angle0 - angle1 + Rad180, Rad360) - Rad180;
    public static float AngesDistanceDeg(float angle0, float angle1) => math.abs(AnglesSeparationDeg(angle0, angle1));
    public static float AnglesDistance(float angle0, float angle1) => math.abs(AnglesSeparation(angle0, angle1));
    public static float NormalizeAngle0_2PI(float angle) => smod(angle, Rad360);
    public static float Sq(float number) => number*number;
    public static float Sq(float3 vector) => Sq(vector.x) + Sq(vector.y) + Sq(vector.z);

    public static bool IsInRangeInclusive(float number, float min, float max) => (min <= number) && (number <= max);
    public static float3 ProjNormVectorXZPlane(float3 vector) { vector.y = 0; return math.normalize(vector); }
    public static float3 ProjNormVectorYZPlane(float3 vector) { vector.x = 0; return math.normalize(vector); }
    public static float3 ProjNormVectorXYPlane(float3 vector) { vector.z = 0; return math.normalize(vector); }
    
    public static bool IsVectorXZPlane(float3 vector, float epsilon = 0.001f) => math.abs(vector.y) < epsilon;
    public static bool IsVectorYZPlane(float3 vector, float epsilon = 0.001f) => math.abs(vector.x) < epsilon;
    public static bool IsVectorXYPlane(float3 vector, float epsilon = 0.001f) => math.abs(vector.z) < epsilon;
    public static bool IsVectorTowardsNormal(float3 vector, float3 normal) => math.dot(vector, normal) < 0f;
    public static bool IsNormalized(float3 vector, float epsilon = 0.001f) => IsEpsEqual(math.length(vector), 1f, epsilon);

    public static bool IsEpsEqualApprox(float2 vector1, float2 vector2, float epsilon = float.Epsilon) => math.all(math.abs(vector2 - vector1) < epsilon);
    public static bool IsEpsEqualApprox(float3 vector1, float3 vector2, float epsilon = float.Epsilon) => math.all(math.abs(vector2 - vector1) < epsilon);

    public static bool IsEpsEqual(float float1, float float2, float epsilon = float.Epsilon) => math.abs(float1 - float2) <= epsilon;
    public static bool IsEpsEqual(float2 vector1, float2 vector2, float epsilon = float.Epsilon) => math.distancesq(vector1, vector2) <= epsilon*epsilon;
    public static bool IsEpsEqual(float3 vector1, float3 vector2, float epsilon = float.Epsilon) => math.distancesq(vector1, vector2) <= epsilon*epsilon;

    public static bool IsEqual(float2 vector1, float2 vector2) => vector1.Equals(vector2);
    public static bool IsEqual(float3 vector1, float3 vector2) => vector1.Equals(vector2);

    public static bool IsZero(float3 vector) => IsEqual(vector, float3.zero);
    public static bool IsNaN(float3 float3) => math.any(math.isnan(float3));

    public static float3 Midpoint(float3 point1, float3 point2) => Average(point1, point2); // Same as Average but for points
    public static float3 Midpoint(float3 point1, float3 point2, float3 point3) => Average(point1, point2, point3);
    public static float3 Average(float3 vector0, float3 vector1) => (vector0 + vector1) * 0.5f; // Same as MidPoint but for vectors
    public static float3 Average(float3 vector0, float3 vector1, float3 vector2) => (vector0 + vector1 + vector2) * 1f/3f;
    public static float3 AverageNormalize(float3 vector0, float3 vector1) => math.normalize(Average(vector0, vector1)); // Same as MidPoint but for vector average
    public static bool IsParallel(float3 vector0, float3 vector1, float epsilon = float.Epsilon) {
        Debug.Assert(!IsZero(vector0) && !IsZero(vector1), $"One of the input vectors is zero: {vector0}, {vector1}");
        return IsParallelUnit(math.normalize(vector0), math.normalize(vector1), epsilon);
    }
    public static bool IsParallelUnit(float3 vectorUnit0, float3 vectorUnit1, float epsilon = float.Epsilon) {
        Debug.Assert(IsNormalized(vectorUnit0) && IsNormalized(vectorUnit1), $"Given unit vectors must be normalized; {vectorUnit0}, {vectorUnit1}");
        return IsEpsEqual(math.abs(math.dot(vectorUnit0, vectorUnit1)), 1f, epsilon);
    }
    public static bool IsPerpendicular(float3 vector0, float3 vector1, float epsilon = float.Epsilon) {
        Debug.Assert(!IsZero(vector0) && !IsZero(vector1), $"One of the input vectors is zero: {vector0}, {vector1}");
        return IsPerpendicularUnit(math.normalize(vector0), math.normalize(vector1), epsilon);
    }
    public static bool IsPerpendicularUnit(float3 vectorUnit0, float3 vectorUnit1, float epsilon = float.Epsilon) {
        Debug.Assert(IsNormalized(vectorUnit0) && IsNormalized(vectorUnit1), $"Given unit vectors must be normalized; {vectorUnit0}, {vectorUnit1}");
        return IsEpsEqual(math.abs(math.dot(vectorUnit0, vectorUnit1)), 0f, epsilon);
    }

    public unsafe static T* NullCoalesce<T>(T* a, T* b) where T : unmanaged => a != null ? a : b; 

    public static int ReverseIndex(int index, int listSize) => listSize - 1 - index; 
    public static int NextIndex(int index, int listSize) => (index + 1) % listSize; 
    public static int PrevIndex(int index, int listSize) => ClampIndexLoop(index - 1, listSize); // Not actually mod operator, so need to ClampIndexLoop
    // Clamp list indices - Will even work if index is larger/smaller than listSize, so can loop multiple times
    public static int ClampIndexLoop(int index, int listSize) => smod(index, listSize); 

    // TODO: Reword naming?
    public static int ComparerIsFirstMax(float value0, float value1) => value0.CompareTo(value1); 
    public static int ComparerIsFirstMin(float value0, float value1) => -value0.CompareTo(value1); 
    public static int ComparerIsFirstMax(int value0, int value1) => value0.CompareTo(value1); 
    public static int ComparerIsFirstMin(int value0, int value1) => -value0.CompareTo(value1); 

    public static float3 Reflect(float3 vector, float3 normal, float restitution = 0) { // restitution: 0 is most bouncy, 1 is least bouncy
        return vector - (2.0f - restitution) * math.project(vector, normal);
    }

    public static bool IsSpheresIntersecting(float3 sphereCenter0, float sphereRadius0, float3 sphereCenter1, float sphereRadius1) {
        return IsPointSphereIntersecting(sphereCenter0, sphereCenter1, sphereRadius0 + sphereRadius1);
    }
    public static bool IsPointSphereIntersecting(float3 point, float3 sphereCenter, float sphereRadius) {
        float3 pointToSphere = sphereCenter - point; // * vec3Mask // Mask is to make it only 2D
        float distanceToSphereSqrd = math.dot(pointToSphere, pointToSphere); // Method of squaring float3's
        if (distanceToSphereSqrd <= sphereRadius*sphereRadius) {
            return true;
        }
        return false;
    }

    public static bool IsRaySphereIntersecting(float3 rayStart, float3 rayDirection, float rayLength, float3 sphereCenter, float sphereRadius, out float distanceAlongRay) {
        return IsRaySphereIntersecting(rayStart, rayDirection, rayLength, sphereCenter, sphereRadius, out distanceAlongRay, out float3 _);
    }
    public static bool IsRaySphereIntersecting(float3 rayStart, float3 rayDirection, float rayLength, float3 sphereCenter, float sphereRadius, out float3 nearestPointOnRay) {
        return IsRaySphereIntersecting(rayStart, rayDirection, rayLength, sphereCenter, sphereRadius, out float _, out nearestPointOnRay);
    }
    public static bool IsRaySphereIntersecting(float3 rayStart, float3 rayDirection, float rayLength, float3 sphereCenter, float sphereRadius, out float distanceAlongRay, out float3 nearestPointOnRay) {
        nearestPointOnRay = NearestPointOnRayToPoint(sphereCenter, rayStart, rayDirection, rayLength, out distanceAlongRay, out bool _);
        return IsPointSphereIntersecting(nearestPointOnRay, sphereCenter, sphereRadius);
    }
    // TODO: Plane capsule should be super easy
    // Untested:
    public static bool IsPointCapsuleIntersecting(float3 point, float3 capsuleStart, float3 capsuleEnd, float capsuleLength, float capsuleRadius, out float3 nearestPointOnCapsuleSeg) {
        return IsRaySphereIntersecting(capsuleStart, math.normalize(capsuleEnd - capsuleStart), capsuleLength, point, capsuleRadius, out nearestPointOnCapsuleSeg);
    }
    public static bool IsPointCylinderIntersecting(float3 point, float3 cylinderStart, float3 cylinderEnd, float cylinderLength, float cylinderRadius, out float3 nearestPointOnCylinderSeg) {
        float3 cylinderDirection = math.normalize(cylinderEnd - cylinderStart);
        nearestPointOnCylinderSeg = NearestPointOnRayToPoint(point, cylinderStart, cylinderDirection, cylinderLength, out float distanceAlongRay, out bool wasClamped);
        if (!wasClamped) {
            return IsPointSphereIntersecting(point, nearestPointOnCylinderSeg, cylinderRadius);
        }
        return false;
    }
    // Untested:
    public static bool IsCapsuleSphereIntersecting(float3 capsuleStart, float3 capsuleDirection, float capsuleLength, float capsuleRadius, float3 sphereCenter, float sphereRadius, out float distanceAlongRay, out float3 nearestPointOnRay) {
        return IsRaySphereIntersecting(capsuleStart, capsuleDirection, capsuleLength, sphereCenter, capsuleRadius + sphereRadius, out distanceAlongRay, out nearestPointOnRay);
    }
    // TODO: IsCapsuleSphereXZ

    public static bool IsSpherePlaneIntersecting(float3 sphereCenter, float sphereRadius, float3 planeNormal, float3 pointOnPlane, out float penetration) {
        float3 pointOnPlaneToSphere = sphereCenter - pointOnPlane;

        penetration = -(math.dot(pointOnPlaneToSphere, planeNormal) - sphereRadius); // TODO: Could use PointToPlaneDistance here
        return penetration > 0;
    }

    // TODO: Untested
    // 2 planes. Taken from spacepuppy
    public static bool IsPlanesIntersecting(float3 plane0Normal, float plane0Distance, float3 plane1Normal, float plane1Distance, out float3 lineArbitraryOrigin, out float3 lineDirection) {
        // Debug.Assert(IsNormalized(vectorUnit0) && IsNormalized(vectorUnit1), $"Given unit vectors must be normalized; {vectorUnit0}, {vectorUnit1}");
        
        lineDirection = math.cross(plane0Normal, plane1Normal); // math.normalize( // Not normalizing because spacepuppy doesn't
        float3 plane1Tangent = math.cross(plane1Normal, lineDirection);
        float dot0NormTo1Tang = math.dot(plane0Normal, plane1Tangent);

        if (!IsEpsEqual(dot0NormTo1Tang, 0f, 0.0001f)) { // if dot above is very close to 0, planes are parallel, return false
            float3 p0Top1 = (plane0Normal * plane0Distance) - (plane1Normal * plane1Distance);
            float t = math.dot(plane0Normal, p0Top1) / dot0NormTo1Tang;
            lineArbitraryOrigin = (plane1Normal * plane1Distance) + t * plane1Tangent;
            return true;
        }
        lineArbitraryOrigin = new float3(float.NaN);
        return false;
    }
    // 3 planes. Taken from spacepuppy
    public static bool IsPlanesIntersecting(float3 plane0Normal, float plane0Distance, float3 plane1Normal, float plane1Distance, float3 plane2Normal, float plane2Distance, out float3 intersectionPoint) {
        // Debug.Assert(IsNormalized(vectorUnit0) && IsNormalized(vectorUnit1), $"Given unit vectors must be normalized; {vectorUnit0}, {vectorUnit1}");
        if (IsPlanesIntersecting(plane0Normal, plane0Distance, plane1Normal, plane1Distance, out float3 lineArbitraryOrigin, out float3 lineDirection)) {
            if (IsLinePlaneIntersecting(lineArbitraryOrigin, lineDirection, plane2Normal, plane2Distance, out float _, out intersectionPoint)) {
                return true;
            }
        }
        intersectionPoint = new float3(float.NaN);
        return false;
    }
    public static bool IsRayPlaneIntersecting(float3 rayOrigin, float3 rayDirection, float rayLength, float3 planeNormal, float planeDistance, out float distanceAlongRay) {
        return IsRayPlaneIntersecting(rayOrigin, rayDirection, rayLength, planeNormal, planeDistance, out distanceAlongRay, out float3 _);
    }
    public static bool IsRayPlaneIntersecting(float3 rayOrigin, float3 rayDirection, float rayLength, float3 planeNormal, float planeDistance, out float3 nearestPointToPlane) {
        return IsRayPlaneIntersecting(rayOrigin, rayDirection, rayLength, planeNormal, planeDistance, out float _, out nearestPointToPlane);
    }

    public static bool IsRayPlaneIntersecting(float3 rayOrigin, float3 rayDirection, float rayLength, float3 planeNormal, float planeDistance, out float distanceAlongRay, out float3 nearestPointToPlane) {
        if (IsLinePlaneIntersecting(rayOrigin, rayDirection, planeNormal, planeDistance, out float distanceAlongLine, out nearestPointToPlane)) {
            distanceAlongRay = math.clamp(distanceAlongLine, 0, rayLength);
            nearestPointToPlane = rayOrigin + (rayDirection * distanceAlongRay);
            return distanceAlongLine == distanceAlongRay; // i.e. return wasNotClamped // This wouldn't work if clamp somehow changes the value // It works though
        }
        distanceAlongRay = distanceAlongLine;
        return false;
    }

    public static bool IsLinePlaneIntersecting(float3 lineOrigin, float3 lineDirection, float3 planeNormal, float planeDistance, out float distanceAlongLine, out float3 pointOnPlane) {
        float constants = math.dot(lineOrigin, planeNormal);
        float coefficients = math.dot(lineDirection, planeNormal);

        if (IsEpsEqual(math.abs(coefficients), 0f, 0.001f)) { // Line and Plane are parallel, no intersection
            distanceAlongLine = 0f;
            pointOnPlane = lineOrigin;
            return false;
        }
        distanceAlongLine = (planeDistance - constants) / coefficients;
        pointOnPlane = lineOrigin + (lineDirection * distanceAlongLine);
        return true;
    }

        // z = (x - x1) / (x2 - x1) * (z2 - z1) + z1
    public static bool IsPointOnLineGivenX(float3 lineOrigin, float3 lineDirection, float x, out float distanceAlongLine, out float3 pointOnLine) =>
        IsLinePlaneIntersecting(lineOrigin, lineDirection, XMask, x, out distanceAlongLine, out pointOnLine);
    public static bool IsPointOnLineGivenY(float3 lineOrigin, float3 lineDirection, float y, out float distanceAlongLine, out float3 pointOnLine) =>
        IsLinePlaneIntersecting(lineOrigin, lineDirection, YMask, y, out distanceAlongLine, out pointOnLine);
    public static bool IsPointOnLineGivenZ(float3 lineOrigin, float3 lineDirection, float z, out float distanceAlongLine, out float3 pointOnLine) =>
        IsLinePlaneIntersecting(lineOrigin, lineDirection, ZMask, z, out distanceAlongLine, out pointOnLine);

    public static bool IsPointOnSegGivenX(float3 segStart, float3 segEnd, float x, out float3 pointOnSeg) => IsPointOnSegGivenHelper(segStart, segEnd, x, XIndex, out pointOnSeg);
    public static bool IsPointOnSegGivenY(float3 segStart, float3 segEnd, float y, out float3 pointOnSeg) => IsPointOnSegGivenHelper(segStart, segEnd, y, YIndex, out pointOnSeg);
    public static bool IsPointOnSegGivenZ(float3 segStart, float3 segEnd, float z, out float3 pointOnSeg) => IsPointOnSegGivenHelper(segStart, segEnd, z, ZIndex, out pointOnSeg);
    static bool IsPointOnSegGivenHelper(float3 segStart, float3 segEnd, float dimensionValue, int componentIndex, out float3 pointOnSeg) {
        float t = (dimensionValue - segStart[componentIndex]) / segEnd[componentIndex] - segStart[componentIndex];
        if (float.IsNaN(t)) {
            pointOnSeg = float3.zero;
            return false;
        }
        pointOnSeg = segStart + t * (segEnd - segStart);
        return true;
    }

    // TODO: Untested:
    public static bool IsPointWithinSegProjFast(float3 point, float3 segStart, float3 segEnd) {
        float3 segDirection = math.normalize(segEnd - segStart);
        return IsPointInFrontOfPlane(point, segDirection, segStart) && IsPointInFrontOfPlane(point, -segDirection, segEnd);
    }
    public static bool IsPointWithinRayProjFast(float3 point, float3 rayStart, float3 rayDirection, float rayLength) {
        float3 vev = point - rayStart;
        float dotLength = math.dot(vev, rayDirection);
        return dotLength >= 0 && dotLength <= rayLength;
    }

    // TODO: Unfinished - Implementation of this within Habrador Geometry
    public static bool IsPointWithinSegProj(float3 point, float3 segStart, float3 segEnd, out float3 nearestPointOnSeg) {
        return IsPointWithinRayProj(point, segStart, math.normalize(segEnd - segStart), math.length(segEnd - segStart), out nearestPointOnSeg);
    }
    public static bool IsPointWithinRayProj(float3 point, float3 rayStart, float3 rayDirection, float rayLength, out float3 nearestPointOnRay) {
        nearestPointOnRay = NearestPointOnRayToPoint(point, rayStart, rayDirection, rayLength, out float _, out bool wasClamped);
        return wasClamped;
    }

    // TODO: Test that both do the same thing:
    public static float3 PointToRayVector2(float3 point, float3 rayStart, float3 rayDirection, float rayLength) {
        float3 nearestPointOnRay = NearestPointOnRayToPoint(point, rayStart, rayDirection, rayLength, out float _, out bool _);
        return nearestPointOnRay - point;
    }
    public static float3 PointToRayVector(float3 point, float3 rayStart, float3 rayDirection, float rayLength) {
        float dotOnSeg = math.clamp(math.dot(point - rayStart, rayDirection), 0, rayLength);
        float3 nearestSegPoint = rayStart + (rayDirection * dotOnSeg);
        return nearestSegPoint - point;
    }
    
    public static float PointToRayDistance(float3 point, float3 rayStart, float3 rayDirection, float rayLength) {        
        return math.length(PointToRayVector(point, rayStart, rayDirection, rayLength));
    }
    public static float PointToRayDistanceXZ(float3 point, float3 rayStart, float3 rayDirection, float rayLength) {        
        float3 fromSegVector = PointToRayVector(point, rayStart, rayDirection, rayLength);
        fromSegVector.y = 0f;
        return math.length(fromSegVector);
    }

    // Simply an alias for the below in case I forget (the below follows MathLib naming conventions but this may be easier to remember)
    public static bool IsPointOnPlane(float3 point, float3 planeNormal, float3 arbitraryPointOnPlane, float epsilon = float.Epsilon) {
        return IsPointPlaneIntersecting(point, planeNormal, arbitraryPointOnPlane, epsilon);
    }
    // Untested
    public static bool IsPointPlaneIntersecting(float3 point, float3 planeNormal, float3 arbitraryPointOnPlane, float epsilon = float.Epsilon) {
        return PointToPlaneDistance(point, planeNormal, arbitraryPointOnPlane) < epsilon;
    }

    public static bool IsPointInFrontPlane(float3 point, float3 planeNormal, float planeDistance) {
        return 0 > PointToPlaneDistanceSigned(point, planeNormal, planeDistance);
    }
    // Untested
    public static bool IsPointInFrontOfPlane(float3 point, float3 planeNormal, float3 arbitraryPointOnPlane) {
        return 0 > PointToPlaneDistanceSigned(point, planeNormal, arbitraryPointOnPlane);
    }
    public static float PointToPlaneDistanceSigned(float3 point, float3 planeNormal, float planeDistance) {
        return math.dot(point, planeNormal) - planeDistance;
    }
    // Positive is in front of normal, negative behind
    public static float PointToPlaneDistanceSigned(float3 point, float3 planeNormal, float3 arbitraryPointOnPlane) {
        return math.dot(point - arbitraryPointOnPlane, planeNormal); // point - pointOnPlane is correct 
    }
    public static float PointToPlaneDistance(float3 point, float3 planeNormal, float planeDistance) {
        return math.abs(PointToPlaneDistanceSigned(point, planeNormal, planeDistance));
    }
    public static float PointToPlaneDistance(float3 point, float3 planeNormal, float3 arbitraryPointOnPlane) {
        return math.abs(PointToPlaneDistanceSigned(point, planeNormal, arbitraryPointOnPlane));
    }
    public static float3 ProjPointToPlane(float3 point, float3 planeNormal, float planeDistance) {
        return point - PointToPlaneDistanceSigned(point, planeNormal, planeDistance) * planeNormal;
    }
    public static float3 ProjPointToPlane(float3 point, float3 planeNormal, float3 arbitraryPointOnPlane) {
        return point - PointToPlaneDistanceSigned(point, planeNormal, arbitraryPointOnPlane) * planeNormal; // TODO: May have to make point + after changing PointToPlaneDistanceSigned
    }

    // public static float3 PointToPlaneSpace(float3 point, float3 planeNormal, float3 arbitraryPointOnPlane) {
    //     return point - PointToPlaneDistanceSigned(point, planeNormal, arbitraryPointOnPlane) * planeNormal;
    // }

    // TODO: Implement:
    // public static float PointTo2DLineDistance(float2 point, float2 planeNormal, float3 arbitraryPointOnPlane) {
    //     return math.abs(PointToPlaneDistanceSigned(point, planeNormal, arbitraryPointOnPlane));
    // }

    public static float3 ResolveSpherePlanePenetration(float3 currentSphereCenter, float3 sphereVelocityDirection, float3 planeNormal, float penetration) { // penetration includes radius
        float dotDirectionNormal = math.dot(-sphereVelocityDirection, planeNormal);
        float dotCoeff = 1/dotDirectionNormal;
        float distanceBackwards = penetration * dotCoeff;
        return currentSphereCenter - sphereVelocityDirection * distanceBackwards; // returns resolved position so sphere is barely touching plane
    }

    // TODO: IsPointPolygonIntersecting?
    // Out of polygon function, but construct edges here. pointOptionallyOnPlane a bad name?
    // pointOptionallyOnPlane does not have to be on the plane for this to return true, hence 'projected to plane'
    public static bool IsPointInConvexPolygon(float3 pointOptionallyOnPlane, in NativeArray<float3> vertexPoints, float3 planeNormal, float amountOutsideEdges = 0) { // Clockwise polygon
        for (int i = 0; i < vertexPoints.Length; i++) {
            float3 vertexPos1 = vertexPoints[i];
            float3 vertexPos2 = vertexPoints[NextIndex(i, vertexPoints.Length)];

            float3 edgeVector = vertexPos2 - vertexPos1;
            float3 edgeRight = math.normalize(math.cross(edgeVector, planeNormal));
            

            float3 pointToVertex1 = pointOptionallyOnPlane - vertexPos1;
            float distanceFromEdge = -math.dot(edgeRight, pointToVertex1);
            if (distanceFromEdge > 0) // Negative if behind the edge // TODO: Use amountOutsideEdges here
                return false;
        }
        return true;
    }

    public static float3 PointInConvexPolygonNearestToPoint(float3 pointOptionallyOnPlane, in NativeArray<float3> vertexPoints, float3 planeNormal) { // Clockwise polygon
        for (int i = 0; i < vertexPoints.Length; i++) {
            float3 vertexPos1 = vertexPoints[i];
            float3 vertexPos2 = vertexPoints[NextIndex(i, vertexPoints.Length)];

            float3 edgeVector = vertexPos2 - vertexPos1;
            float3 edgeRight = math.normalize(math.cross(edgeVector, planeNormal));
            

            float3 pointToVertex1 = pointOptionallyOnPlane - vertexPos1;
            float distanceFromEdge = -math.dot(edgeRight, pointToVertex1);
            // TODO: IsPointWithinSegProj is kinda slow here, better to use IsPointWithinSegProjFast? Then can calculate
            // If point is outisde of edge, and within the edge seg projected, then return the nearest point on edge seg
            if (distanceFromEdge < 0 && IsPointWithinSegProj(pointOptionallyOnPlane, vertexPos1, vertexPos2, out float3 nearestPointOnSeg)) // Negative if behind the edge
                return nearestPointOnSeg;
        }
        return ProjPointToPlane(pointOptionallyOnPlane, planeNormal, vertexPoints[0]);
    }

    public static float3 ReflectPointAcrossPlane(float3 point, float3 planeNormal, float planeDistance) {
        float distance = math.dot(planeNormal, point) - planeDistance; // TODO: Use PointToPlaneDistance()
        return point - planeNormal * distance * 2f;
    }

    public static float GetUnitVectorXZComponent(float3 vector) {
        Debug.Assert(IsNormalized(vector));
        return math.sqrt(1f - vector.y*vector.y);
    }
    /* public static float UnitVectorAngleOfElevation(float3 vector) {
        return math.atan(vector.y / GetUnitVectorXZComponent(vector));
    } */

    // SlopeAngle == math.abs(AngleOfElevation)
    public static float PlaneSlopeAngle(float3 planeNormal) => Rad90 - math.abs(VectorAngleOfElevation(planeNormal));
    public static float PlaneAngleOfElevation(float3 planeNormal) => Rad90 - VectorAngleOfElevation(planeNormal); // TODO: Untested
    // TODO: Consider renaming to VectorSlopeAngle
    public static float VectorSlopeAngle(float3 vector) => math.abs(VectorAngleOfElevation(vector)); // TODO: Is this really necessary?
    public static float VectorAngleOfElevation(float3 vector) => AngleBetweenVectorAndXZPlane(vector); // This is an alias for AngleBetweenVectorAndXZPlane()
    public static float AngleBetweenVectorAndXZPlane(float3 vector) { // TODO: This could be greatly optimized since its just math.acos(vector.y)
        return AngleBetweenVectorAndPlane(vector, math.up());
    }
    public static float AngleBetweenVectorAndPlane(float3 vector, float3 planeNormal) { // TODO: What angle range does this return?
        return Rad90 - AngleBetweenTails(vector, planeNormal);
    }
    public static float AngleBetweenPlanes(float3 planeNormal0, float3 planeNormal1) {
        return AngleBetweenTails(planeNormal0, planeNormal1);
    }

    public static float AngleBetweenTails(float3 p0, float3 p1_atAngle, float3 p2) { // Returns between 0-180
        return AngleBetweenTails(p1_atAngle - p0, -(p2 - p1_atAngle)); // Negative because tails
    }
    public static float AngleBetweenTails(float3 tailVector0, float3 tailVector1) { // Tail-to-tail - Returns between 0-180 // TODO: UnitTails?
        Debug.Assert(!IsNaN(tailVector0) && !IsNaN(tailVector1), "AngleBetweenTails - At least one tail vector is NaN");
        // Debug.Assert(IsNormalized(tailVector0) && IsNormalized(tailVector1), "AngleBetweenTails - length(tailVector0): " + math.length(tailVector0) + ", length(tailVector1): " + math.length(tailVector1));
        float dotNormalized = math.clamp( math.dot(tailVector0, tailVector1) / (math.length(tailVector0) * math.length(tailVector1)) , -1f, 1f ); // TODO: Why clamp?
        Debug.Assert(-1.01f <= dotNormalized && dotNormalized <= 1.01f);
        float angleRad = math.acos(dotNormalized);
        // Debug.Assert(!float.IsNaN(angleRad), "AngleBetweenTails - tailVector0: " + tailVector0 + ", tailVector1: " + tailVector1 + ", angleRad: " + angleRad + ", dot: " + dot);
        // dot(a_normalized, b_normalized) = cos(alpha) -> acos(dot(a_normalized, b_normalized)) = alpha
        return angleRad;
    }

    // Polygon does not need to be planar
    public static bool IsPolygonConvex(in NativeArray<float3> vertexPoints, float3 planeNormal, out bool isInAngle, float minAngle = 0, float maxAngle = Rad180) { // Any wise
        Debug.Assert(!IsNaN(planeNormal), "IsPolygonConvex - planeNormal is NaN");
        Debug.Assert(IsInRangeInclusive(minAngle, 0, Rad180) && IsInRangeInclusive(maxAngle, 0, Rad180) && minAngle <= maxAngle);
        isInAngle = true;
        int i_Prev = vertexPoints.Length-1;
        float angleSum = 0;
        for (int i = 0; i < vertexPoints.Length; i++) {
            float3 prevEdgeVector = vertexPoints[i] - vertexPoints[i_Prev]; // i - i_Prev
            float3 edgeVector = vertexPoints[NextIndex(i, vertexPoints.Length)] - vertexPoints[i]; // i_Next - i

            CalcPerpParaFromNormal(prevEdgeVector, planeNormal, out float3 prevPerpendicular, out float3 prevParallel); // Using perpendicular so non-planar polygons will work
            CalcPerpParaFromNormal(edgeVector, planeNormal, out float3 perpendicular, out float3 parallel);
            // Debug.Log("prevPerp: " + prevPerp + ",   prevPerpendicular" + prevPerpendicular);
            // Debug.Log("perp: " + perp + ",   perpendicular" + perpendicular);

            float angle = AngleBetweenTails(prevPerpendicular, -perpendicular);
            if (angle < minAngle || angle > maxAngle) { Debug.Log($"IsPolygonConvex - Angle wrong: {math.degrees(angle)}"); isInAngle = false; }

            float3 crossForSign = math.cross(prevEdgeVector, -edgeVector);
            float sign = math.sign(math.dot(planeNormal, crossForSign));
            if (sign < 0) { Debug.Log($"IsPolygonConvex - Sign wrong: {sign}"); return false; }

            angleSum += angle;
            i_Prev = i;
        }
        float shouldbeAngle = (vertexPoints.Length - 2) * Rad180; // Angle sum prevents star polygons
        if (!IsEpsEqual(angleSum, shouldbeAngle, 0.0001f)) { Debug.Log($"IsPolygonConvex - Angle sum wrong: {math.abs(angleSum - shouldbeAngle)}"); return false; }
        return true;
    }

    // TODO: LocalXZ?
    public static bool IsPolygonSelfIntersecting(in NativeArray<float3> vertexPoints, float4x4 invLocalMatrix) { // Any wise
        NativeArray<float2> localVertexPoints = new NativeArray<float2>(vertexPoints.Length, Allocator.Temp);
        for (int i = 0; i < vertexPoints.Length; i++) {
            localVertexPoints[i] = PointWorldToLocal(vertexPoints[i], invLocalMatrix).xz;
        }

        // int i_Prev = vertexPoints.Length-1;
        for (int i = 0; i < vertexPoints.Length; i++) {
            int i_Next = NextIndex(i, vertexPoints.Length);
            float2 vertexPoint_i = localVertexPoints[i];
            float2 nextVertexPoint_i = localVertexPoints[i_Next];

            for (int j = i; j < vertexPoints.Length; j++) {
                int j_Next = NextIndex(j, vertexPoints.Length);
                if (j != i && j != i_Next && j_Next != i && j_Next != i_Next) {
                    float2 vertexPoint_j = localVertexPoints[j];
                    float2 nextVertexPoint_j = localVertexPoints[j_Next];

                    if (IsSegsIntersecting( vertexPoint_i, nextVertexPoint_i, vertexPoint_j, nextVertexPoint_j, out float2 intersectionPoint )) {
                        float3 worldIntersectionPoint = PointLocalToWorld(intersectionPoint.XOY(), math.inverse(invLocalMatrix));
                        CommonLib.CreatePrimitive(PrimitiveType.Capsule, worldIntersectionPoint, new float3(0.1f), Color.red);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // public static NativeArray<float3> InsetPolygonXZ(NativeArray<float3> vertexPoints, float3 planeNormal, float insetOutset, out bool isWorked) { // Any wise
    //     NativeArray<float3> vertexPointsCopy = new NativeArray<float3>(vertexPoints.Length, vertexPoints.AllocatorLabel());
    //     vertexPoints.CopyTo(vertexPointsCopy);
    
    //     isWorked = true;
    //     int i_Prev = vertexPoints.Length-1;
    //     for (int i = 0; i < vertexPoints.Length; i++) {
    //         float3 prevEdgeVector = vertexPoints[i] - vertexPoints[i_Prev]; // i - i_Prev
    //         float3 edgeVector = vertexPoints[NextIndex(i, vertexPoints.Length)] - vertexPoints[i]; // i_Next - i
    //         float3 prevEdgeRight = math.normalize(math.cross(prevEdgeVector, planeNormal));
    //         float3 edgeRight = math.normalize(math.cross(edgeVector, planeNormal));

    //         float3 edgePMid = vertexPoints[i] + edgeRight * insetOutset;
    //         float3 prevEdgePMid = vertexPoints[i] + prevEdgeRight * insetOutset;
    //         if (!IsSegsIntersectingXZ(edgePMid - edgeVector*1f, edgePMid + edgeVector*1f,
    //                 prevEdgePMid - prevEdgeVector*1f, prevEdgePMid + prevEdgeVector*1f, out float3 intersectionPoint)) {
    //             isWorked = false;
    //         }
    //         vertexPointsCopy[i] = intersectionPoint;
    //         i_Prev = i;
    //     }
    //     return vertexPointsCopy;
    // }

    // TODO: Pretty sure this is not working:
    public static float3 CalcPolygonIntegratedCentroidNotIdeal(in NativeArray<float3> vertexPoints) { // Signed polygon area
        float polygonArea = CalcPolygonArea(vertexPoints); // can combine with below
	
        float factor = 1 / (6 * polygonArea); // 6 is a constant
        
        float sumX_Y = 0, sumX_Z = 0, sumY = 0, sumZ = 0;
        
        int i_Prev = vertexPoints.Length-1; // Allows 'looping' around list
        for (int i = 0; i < vertexPoints.Length; i++) {
            float3 prevVertexPoint = vertexPoints[i_Prev];
            float3 vertexPoint = vertexPoints[i];
            
            // float forAll = math.length(math.cross(prevVertexPoint, vertexPoint)); //(prevVertexPoint.x*vertexPoint.z - vertexPoint.x*prevVertexPoint.z);
            sumX_Y += (prevVertexPoint.x + vertexPoint.x) * (prevVertexPoint.x*vertexPoint.y - vertexPoint.x*prevVertexPoint.y);
            sumX_Z += (prevVertexPoint.x + vertexPoint.x) * (prevVertexPoint.x*vertexPoint.z - vertexPoint.x*prevVertexPoint.z);
            sumY += (prevVertexPoint.y + vertexPoint.y) * (prevVertexPoint.x*vertexPoint.y - vertexPoint.x*prevVertexPoint.y);
            sumZ += (prevVertexPoint.z + vertexPoint.z) * (prevVertexPoint.x*vertexPoint.z - vertexPoint.x*prevVertexPoint.z);
            
            i_Prev = i; // iPrev is previous vertex to i
        }
        
        float centroidX = factor * sumX_Z; // Original formula did not make this negative
        float centroidY = factor * sumY;
        float centroidZ = factor * sumZ;
        
        return new float3(centroidX, centroidY, centroidZ);
    }

    // Don't think this is working
    public static float3 CalcPolygonIntegratedCentroidAttempt(in NativeArray<float3> vertexPoints) { // Signed polygon area
        float polygonArea = CalcPolygonArea(vertexPoints); // can combine with below
	
        float factor = 1 / (6 * polygonArea); // 6 is a constant
        
        float sumX = 0, sumY = 0, sumZ = 0;
        
        int i_Prev = vertexPoints.Length-1; // Allows 'looping' around list
        for (int i = 0; i < vertexPoints.Length; i++) {
            float3 prevVertexPoint = vertexPoints[i_Prev];
            float3 vertexPoint = vertexPoints[i];
            
            float forAll = math.length(math.cross(prevVertexPoint, vertexPoint)); //(prevVertexPoint.x*vertexPoint.z - vertexPoint.x*prevVertexPoint.z);
            sumX += (prevVertexPoint.x + vertexPoint.x) * forAll;
            sumY += (prevVertexPoint.y + vertexPoint.y) * forAll;
            sumZ += (prevVertexPoint.z + vertexPoint.z) * forAll;
            
            i_Prev = i; // iPrev is previous vertex to i
        }
        
        float centroidX = factor * sumX; // Original formula did not make this negative
        float centroidY = factor * sumY;
        float centroidZ = factor * sumZ;
        
        return new float3(centroidX, centroidY, centroidZ);
    }

    /* public static float CalcPolygonArea(in NativeArray<float3> vertexPoints) {
        float3 crossSum = float3.zero;
        float3 firstVertexPoint = vertexPoints[0];
        for (int i = 1; i < vertexPoints.Length; i++) {
            float3 vertexPoint = vertexPoints[i];
            float3 nextVertexPoint = vertexPoints[NextIndex(i, vertexPoints.Length)];
            float3 cross = math.cross(vertexPoint - firstVertexPoint, nextVertexPoint - firstVertexPoint);
            crossSum += 0.5f*cross;
        }
        return math.length(crossSum);
    } */

    // https://math.stackexchange.com/questions/3207981/how-do-you-calculate-the-area-of-a-2d-polygon-in-3d
    public static float CalcPolygonArea(in NativeArray<float3> vertexPoints) { // Signed polygon area
        float3 crossSum = float3.zero;

        float3 prevVertexPoint = vertexPoints[vertexPoints.Length-1];

        for (int i = 0; i < vertexPoints.Length-1; i++) {
            float3 vertexPoint = vertexPoints[i];
            float3 nextVertexPoint = vertexPoints[i+1];

            crossSum += math.cross(prevVertexPoint - vertexPoint, prevVertexPoint - nextVertexPoint);
        }
        return 0.5f * math.length(crossSum);
    }

    public static class PolygonUtility {
        // https://stackoverflow.com/a/1165943
        public static float SignedArea(NativeArray<float2> points) // TODO: Horrible, rewrite maybe using math.det
        {
            if (points.Length <= 1) {
                return 0;
            }
            // Sum over the edges, (x2 − x1)(y2 + y1).
            // If the result is positive the curve is clockwise, if it's negative the curve is counter-clockwise.
            // (The result is twice the enclosed area, with a +/- convention.)
            float sum = 0;

            for (var i = 0; i < points.Length - 1; i++){
                var p1 = points[i];
                var p2 = points[i + 1];

                sum += (p2.x - p1.x) * (p2.y + p1.y);
            }

            {
                var p1 = points[^1];
                var p2 = points[0];

                sum += (p2.x - p1.x) * (p2.y + p1.y);
            }

            return sum;
        }

        public static bool IsClockwise(NativeArray<float2> points) {
            // Positive value is clockwise, negative value is not
            return SignedArea(points) > 0;
        }

        public static bool IsCounterClockwise(NativeArray<float2> points) {
            // Positive value is clockwise, negative value is not
            return SignedArea(points) < 0;
        }
    }

    public static float CalcTriangleAreaXZ(float3 point1, float3 point2, float3 point3) {
        float determinant = point1.x*(point2.z - point3.z) + point2.x*(point3.z - point1.z) + point3.x*(point1.z - point2.z);
        float area = 0.5f * determinant;
        return math.abs(area);
    }
    public static float CalcTriangleArea(float3 point1, float3 point2, float3 point3) {
        return 0.5f * math.length(math.cross(point1 - point2, point2 - point3));
    }

    // TODO: Untested
    public static float3 CalcTriangleCircumCenter(float3 a, float3 b, float3 c) {
        float3 a2b = math.pow(math.length(a), 2.0f) * b;
        float3 b2a = math.pow(math.length(b), 2.0f) * a;
        float3 aCrossb = math.cross(a, b);
        float3 numer = math.cross(a2b - b2a, aCrossb);
        float denom = 2.0f * math.pow(math.length(aCrossb), 2.0f);
        return numer / denom + c;
    }

    public static bool IsPolygonConvexOld(in NativeArray<float3> vertexPositions, out int concaveVertexIndex) { // Clockwise polygon
        int n = vertexPositions.Length;
        if (n >= 3) {
            for (int i = 0; i < vertexPositions.Length; i++) {
                concaveVertexIndex = (i + 1) % n;
                float3 point0 = vertexPositions[i];
                float3 point1_angle = vertexPositions[(i + 1) % n];
                float3 point2 = vertexPositions[(i + 2) % n];

                float3 vector = point1_angle - point0;
                float3 normal = math.cross(math.up(), vector);

                float dot = math.dot(math.normalize(point2 - point1_angle), normal);
                // Debug.Log("dot: " + dot);
                if (dot < 0) {
                    return false;
                }
            }
        }
        concaveVertexIndex = -1;
        return true;
    }

    // TODO: This should be split (potentially into 3 parts), one for square, one for rect
    // Takes in direction and right vectors because start and end may be the same point (maybe have overloaded method that does it automatically) 
    public static NativeArray<float3> CreatePlanarRectPolygon(float3 start, float3 end, float3 direction, float3 right, float directionWidth, float rightWidth, Allocator allocator) {
        NativeArray<float3> vertexPoints = new NativeArray<float3>(4, allocator);

        vertexPoints[0] = start - direction*directionWidth + right*rightWidth;
        vertexPoints[1] = start - direction*directionWidth - right*rightWidth;
        vertexPoints[2] = end + direction*directionWidth - right*rightWidth;
        vertexPoints[3] = end + direction*directionWidth + right*rightWidth;

        return vertexPoints;
    }

    // If you have and accurate planeNormal already, there is another variant of ArePointsCoplanar that takes it
    public static bool ArePointsCoplanar(in NativeArray<float3> points, float epsilon = float.Epsilon) {
        return ArePointsCoplanar(in points, out float3 _, out float _, epsilon);
    }

    public static bool ArePointsCoplanar(in NativeArray<float3> points, out float3 planeNormal, out float planeDistance, float epsilon = float.Epsilon) {
        CalcTrianglePlaneCCW(points[0], points[1], points[2], out planeNormal, out planeDistance);
        return ArePointsCoplanar(in points, planeNormal, epsilon);
    }
    public static bool ArePointsCoplanar(in NativeArray<float3> points, float3 planeNormal, float epsilon = float.Epsilon) {
        if (points.Length >= 3) {
            // CalcPlaneFrom3PointsCCW(vertexPoints[0], vertexPoints[1], vertexPoints[2], out planeNormal, out planeDistance);
            for (int i = 1; i < points.Length; i++) {
                if (!IsPointOnPlane(points[i], planeNormal, points[0], epsilon)) {
                    return false;
                }
            }
        } else {
            planeNormal = default;
            Debug.Assert(false, "vertexPositions must have at least 3 elements");
        }
        return true;
    }

    // Sorts CCW
    public struct SortAroundLocalXZComparer : IComparer<float3> { // Sorts using XZ
        public float4x4 invLocalMatrix;
        public int Compare(float3 point0, float3 point1) {
            SquaredPolar(point0, out float angle0, out float distanceSq0);
            SquaredPolar(point1, out float angle1, out float distanceSq1);
            int angleSign = (int)math.sign(angle0 - angle1);
            return angleSign != 0 ? angleSign : (int)math.sign(distanceSq0 - distanceSq1); // points.sort((a,b) => a[2] - b[2] || a[3] - b[3]);
        }
        void SquaredPolar(float3 point, out float angle, out float distanceSq) {
            float3 localPoint = PointWorldToLocal(point, invLocalMatrix);
            angle = math.atan2(localPoint.z, localPoint.x); // -centroid.y  -centroid.x
            distanceSq = Sq(localPoint.x) + Sq(localPoint.z); // Square of distance
        }
    }
    // Sorts CCW
    public static NativeArray<float3> SortPointsCopyAroundLocalXZ(NativeArray<float3> points, float4x4 invLocalMatrix, Allocator allocator) {
        NativeArray<float3> pointsCopy = new NativeArray<float3>(points.Length, allocator);
        points.CopyTo(pointsCopy);
        return SortPointsAroundLocalXZ(ref pointsCopy, invLocalMatrix);
    }
    public static ref NativeArray<float3> SortPointsAroundLocalXZ(ref NativeArray<float3> points, float4x4 invLocalMatrix) {
        // Sort by polar angle and distance, centered at centroid.
        points.Sort(new SortAroundLocalXZComparer{ invLocalMatrix = invLocalMatrix });
        return ref points;
    }

    // Potential future implementation where take code from floats.Sort() and swap elements in both arrays at the same time
    /* public struct SortFloatWithFriend<TFriend> : IComparer<float> where TFriend : unmanaged {
        NativeArray<TFriend> friends; // = new NativeArray<TFriend>(points.Length, allocator);
        public SortFloatWithFriend(NativeArray<TFriend> friends) {}
        public int Compare(float sortFriend0, float sortFriend1) {}
    } */
    // public interface ISortFloatWithFriend<TFriend> where TFriend : unmanaged {
    //     float Float { get; }
    //     TFriend Friend { get; }
    // }
    public struct SortFloatWithFriendComparer<TFriend> : IComparer<FloatWithFriend<TFriend>> where TFriend : unmanaged {
        public int Compare(FloatWithFriend<TFriend> sortFriend0, FloatWithFriend<TFriend> sortFriend1) {
            return sortFriend0.Float.CompareTo(sortFriend1.Float);
        }
    }
    public static void SortFloatWithFriend<TFriend>(this NativeArray<FloatWithFriend<TFriend>> floatsAndFriends) where TFriend : unmanaged {
        // floatsAndFriends.Sort(new SortFloatWithFriendComparer<TFriend>());
        SortFloatWithFriend(floatsAndFriends, new SortFloatWithFriendComparer<TFriend>());
    }
    public static void SortFloatWithFriend<TFriend, TComparer>(this NativeArray<FloatWithFriend<TFriend>> floatsAndFriends, TComparer comparer)
        where TFriend : unmanaged  where TComparer : unmanaged, IComparer<FloatWithFriend<TFriend>> {
        floatsAndFriends.Sort(comparer);
    }
    // public static ref NativeArray<FloatWithFriend<TFriend>> SortFloatWithFriend<TFriend>(this NativeArray<FloatWithFriend<TFriend>> floatsAndFriends) where TFriend : unmanaged {
    //     floatsAndFriends.Sort(new SortFloatWithFriendComparer<TFriend>());
    //     return ref floatsAndFriends;
    // }

    public static float3 CalcCentroid(in NativeArray<float3> points) {
        float3 pointSum = float3.zero;
        for (int i = 0; i < points.Length; i++) {
            pointSum += points[i];
        }
        return pointSum / points.Length;
    }

    // Credit?
    public static void CalcTriBarycentricCoords(float3 p, float3 a, float3 b, float3 c, out float u, out float v, out float w) {
        float3 v0 = b - a;
        float3 v1 = c - a;
        float3 v2 = p - a;
        float d00 = math.dot(v0, v0);
        float d01 = math.dot(v0, v1);
        float d11 = math.dot(v1, v1);
        float d20 = math.dot(v2, v0);
        float d21 = math.dot(v2, v1);
        float denom = d00 * d11 - d01 * d01;
        v = (d11 * d20 - d01 * d21) / denom;
        w = (d00 * d21 - d01 * d20) / denom;
        u = 1 - v - w;
    }
    // I developed this algorithm essentially from scratch (only given the CalcTriBary above):
    public static NativeArray<float> CalcPolygonBarycentricCoords(in NativeArray<float3> vertexPoints, float3 point) {
        NativeArray<float> weights = new NativeArray<float>(vertexPoints.Length, Allocator.Temp);
        CalcPolygonBarycentricCoords(in vertexPoints, point, ref weights);
        return weights;
    }
    public static void CalcPolygonBarycentricCoords(in NativeArray<float3> vertexPoints, float3 point, ref NativeArray<float> weights) { // TODO: Test breaking for loop
        int numPoints = vertexPoints.Length;
        Debug.Assert(numPoints == weights.Length, "Weights array must have the same size as vertex points array");

        float3 polygonCentorid = CalcCentroid(vertexPoints);
        float3 normal = CalcPolygonNormalCCW(vertexPoints);


        int iPrev = numPoints - 1;
        // for i,part in pairs(parts) do
        for (int i = 0; i < vertexPoints.Length; i++) { // Loop through each triangle created by i_prev, i, and polygonCentroid (point will only be inside on of them)
            // if CommonLib.PointWithinPolygon(reversePart, { vertexPoints[iPrev], vertexPoints[i], PolygonCentoridBasic }) then
            NativeArray<float3> triVertices = new NativeArray<float3>(3, Allocator.Temp){ [0]=vertexPoints[iPrev], [1]=vertexPoints[i], [2]=polygonCentorid };
            // Debug.Assert(IsEpsilonEqual(normal, CalcNormalFromPolygonCCW(triVertices), 0.001f), "normal: " + normal + ",  planeNormal: " + CalcNormalFromPolygonCCW(triVertices));
            if (IsPointInConvexPolygon(point, triVertices, normal)) { // TODO: Make IsPointInTri and use the Tri's normal. Why not using Tri's normal?
                //float3 ThirdImaginaryPoint = 3*PolygonCentoridBasic - vertexPoints[iPrev] - vertexPoints[i];
                
                // Centroid of the polygon without i and iPrev:
                float3 thirdImaginaryPoint = (polygonCentorid*numPoints - vertexPoints[iPrev] - vertexPoints[i])/(numPoints - 2);
                
                    // float TriangleThirdArea = CalcTriangleAreaXZ(vertexPoints[iPrev], vertexPoints[i], point);
                    // Debug.Log(CalcTriangleArea(vertexPoints[iPrev], vertexPoints[i], point) + "  :  " + TriangleThirdArea);
                    // float TriangleiArea = CalcTriangleAreaXZ(vertexPoints[iPrev], thirdImaginaryPoint, point);
                    // Debug.Log(CalcTriangleArea(vertexPoints[iPrev], thirdImaginaryPoint, point) + "  :  " + TriangleiArea);
                    // float TriangleiPrevArea = CalcTriangleAreaXZ(vertexPoints[i], thirdImaginaryPoint, point);
                    // Debug.Log(CalcTriangleArea(vertexPoints[i], thirdImaginaryPoint, point) + "  :  " + TriangleiPrevArea);
                
                // print(TriangleiArea, " : ", TriangleiPrevArea, " : ", TriangleThirdArea);
                
                CalcTriBarycentricCoords(point, vertexPoints[iPrev], vertexPoints[i], thirdImaginaryPoint, out float u, out float v, out float w);
                
                weights[iPrev] = u; // (TriangleiPrevArea / polygonArea);
                weights[i] = v; // (TriangleiArea / polygonArea);
                
                // for j,part in pairs(parts) do
                for (int j = 0; j < vertexPoints.Length; j++) {
                    if (j != i && j != iPrev) {
                        weights[j] = w / (numPoints - 2); // (TriangleThirdArea / polygonArea); // Distribute remaining weight equally to all other points
                    }
                }
                // break; //TODO: ????
                //print(u, " : ", v, " : ", w);
            }

            //reversePartWeighting[indexTarget] = areaDifference / polygonArea
            //print(reversePartWeighting[indexTarget])

            iPrev = i;
        }
	}

    public static void ScaleSegLength(float3 edgeVertPoint1, float3 edgeVertPoint2, float scaleAmount, out float3 scaledEdgeVertPoint1, out float3 scaledEdgeVertPoint2) {
        float3 edgeVectorUnit = math.normalize(edgeVertPoint2 - edgeVertPoint1);
        scaledEdgeVertPoint1 = edgeVertPoint1 - edgeVectorUnit*scaleAmount;
        scaledEdgeVertPoint2 = edgeVertPoint2 + edgeVectorUnit*scaleAmount;
    }

    public static bool ArePolygonEdgesWithinLength(in NativeArray<float3> vertexPoints, out NativeList<int> badEdgeIndices, float minLength = 0.1f, float maxLength = 100f) {
        badEdgeIndices = new NativeList<int>(Allocator.Temp);
        for (int i = 0; i < vertexPoints.Length; i++) {
            float3 edgeVector = vertexPoints[NextIndex(i, vertexPoints.Length)] - vertexPoints[i];
            float edgeLength = math.length(edgeVector);
            if (IsNaN(edgeVector) || edgeLength < minLength || edgeLength > maxLength) {
                badEdgeIndices.Add(i);
            }
        }
        return badEdgeIndices.IsEmpty;
    }

    public static float DistanceAlongLineNearestToPoint(float3 point, float3 lineOrigin, float3 lineDirection) {
        float3 lineOriginToPoint = point - lineOrigin;
        return math.dot(lineOriginToPoint, lineDirection);
    }
    public static float3 NearestPointOnLineToPoint(float3 point, float3 lineOrigin, float3 lineDirection, out float distanceAlongLine) {
        distanceAlongLine = DistanceAlongLineNearestToPoint(point, lineOrigin, lineDirection);
        return lineOrigin + (lineDirection * distanceAlongLine);
    }
    public static float3 NearestPointOnSegToPoint(float3 point, float3 segStart, float3 segEnd, out float distanceAlongSeg, out bool wasClamped) {
        return NearestPointOnRayToPoint(point, segStart, math.normalize(segEnd - segStart), math.length(segEnd - segStart), out distanceAlongSeg, out wasClamped);
    }
    public static float3 NearestPointOnRayToPoint(float3 point, float3 rayStart, float3 rayDirection, float rayLength, out float distanceAlongRay, out bool wasClamped) {
        float distanceAlongRayLine = DistanceAlongLineNearestToPoint(point, rayStart, rayDirection);
        distanceAlongRay = math.clamp(distanceAlongRayLine, 0, rayLength);
        wasClamped = distanceAlongRay != distanceAlongRayLine;
        return rayStart + (rayDirection * distanceAlongRay);

        // float3 rayStartToPoint = point - rayStart;
        // float dotDistanceAlongRay = math.dot(rayStartToPoint, rayDirection);
        // distanceAlongRay = math.clamp(dotDistanceAlongRay, 0, rayLength);
        // wasClamped = dotDistanceAlongRay != distanceAlongRay;
        // return rayStart + (rayDirection * distanceAlongRay);
    }

    // https://math.stackexchange.com/q/3436386
    public static float3 NearestPointOnLine1ToLine2(float3 line1Point, float3 line1Direction, float3 line2Point, float3 line2Direction, out float distanceAlongLine1)
    {
        float3 posDiff = line1Point - line2Point;
        float3 crossNormal = math.normalize(math.cross(line1Direction, line2Direction));
        float3 rejection = posDiff - math.project(posDiff, line2Direction) - math.project(posDiff, crossNormal);
        distanceAlongLine1 = -math.length(rejection) / math.dot(line1Direction, math.normalize(rejection));
        return line1Point + line1Direction * distanceAlongLine1;
    }
    // Abstract below with the above
    public static float3 NearestPointOnRayToLine(float3 rayStart, float3 rayDirection, float rayLength, float3 linePoint, float3 lineDirection, out float distanceAlongRay) {
        NearestPointOnLine1ToLine2(rayStart, rayDirection, linePoint, lineDirection, out distanceAlongRay);
        return rayStart + rayDirection * math.clamp(distanceAlongRay, 0, rayLength);
    }

    public static float ShortestDistanceBtwLines(float3 line1Direction, float3 line2Direction, float3 line1Point, float3 line2Point) {
        return ShortestDistanceBtwLines(line1Direction, line2Direction, line2Point - line1Point);
    }

    public static float ShortestDistanceBtwLines(float3 line1Direction, float3 line2Direction, float3 arbitraryPoint1ToPoint2Vector) {
        float3 crossNormal = math.normalize(math.cross(line1Direction, line2Direction));
        return math.dot(crossNormal, -arbitraryPoint1ToPoint2Vector);
    }

    public static bool IsCapsulesIntersecting(float3 start1, float3 end1, float length1, float radius1, float3 start2, float3 end2, float length2, float radius2, out float distanceAlongRay, out float3 nearestPoint) {
        return IsRayAACapsuleIntersecting(start1, end1, length1, start2, end2, length2, radius1 + radius2, out distanceAlongRay, out nearestPoint);
    }

    // Untested but this might work for any capsule orientation: Also add intersection point?
    public static bool IsRayAACapsuleIntersecting(float3 rayStart, float3 rayEnd, float rayLength, float3 capsuleStart, float3 capsuleEnd, float capsuleLength, float capsuleRadius, out float distanceAlongRay, out float3 nearestPoint)
    {
        RayToAABB(rayStart, rayEnd, out float3 rayMinPosition, out float3 rayMaxPosition);
        CapsuleToAABB(capsuleStart, capsuleEnd, capsuleRadius, out float3 capsuleMinPosition, out float3 capsuleMaxPosition);
        nearestPoint = float3.zero; distanceAlongRay = -1;

        if (IsAABBsIntersecting(rayMinPosition, rayMaxPosition, capsuleMinPosition, capsuleMaxPosition)) { // This all probably won't work in some cases
            float3 rayVector = rayEnd - rayStart;
            float3 rayDirection = math.normalize(rayVector);

            float3 capsuleVector = capsuleEnd - capsuleStart;
            float3 capsuleVectorDirection = math.normalize(capsuleVector);

            // float3 nearestPointOnRay = NearestPointOnRayToLine(rayStart, rayDirection, rayLength, capsuleStart, capsuleVectorDirection);

            float3 nearestPointOnCapsuleSeg = NearestPointOnRayToLine(capsuleStart, capsuleVectorDirection, capsuleLength, rayStart, rayDirection, out distanceAlongRay);
            nearestPoint = nearestPointOnCapsuleSeg;
            // Capsule seg is from start to end positions
            distanceAlongRay = math.length(nearestPointOnCapsuleSeg - rayStart); // Without this, selecting a soldier sometimes selects one in behind

            bool isIntersecting = IsRaySphereIntersecting(rayStart, rayDirection, rayLength, nearestPointOnCapsuleSeg, capsuleRadius, out float3 _);
            // IsPointCapsuleIntersecting

            Color color = Color.red;
            if (isIntersecting) { color = Color.green; }
            // CommonLib.CreatePrimitive(PrimitiveType.Sphere, nearestPointOnCapsuleSeg, new float3(0.1f), color, new Quaternion(), 1.0f);
            // CommonLib.CreatePrimitive(PrimitiveType.Sphere, _f, new float3(0.1f), Color.yellow, new Quaternion(), 1.0f);

            return isIntersecting; // TODO: If true, now test the other way around to be certain
        }
        return false;
    }

    public static bool IsAABBsIntersecting(float3 minPosBox1, float3 maxPosBox1, float3 minPosBox2, float3 maxPosBox2) {
        /* CommonLib.CreatePrimitive(PrimitiveType.Cube, minPosBox1, new float3(0.05f), Color.blue, new Quaternion(), 5.0f);
        CommonLib.CreatePrimitive(PrimitiveType.Cube, maxPosBox1, new float3(0.05f), Color.black, new Quaternion(), 5.0f);

        CommonLib.CreatePrimitive(PrimitiveType.Cube, minPosBox2, new float3(0.05f), Color.green, new Quaternion(), 5.0f);
        CommonLib.CreatePrimitive(PrimitiveType.Cube, maxPosBox2, new float3(0.05f), Color.magenta, new Quaternion(), 5.0f); */

        return minPosBox1.y <= maxPosBox2.y && minPosBox2.y <= maxPosBox1.y
            && minPosBox1.x <= maxPosBox2.x && minPosBox2.x <= maxPosBox1.x
            && minPosBox1.z <= maxPosBox2.z && minPosBox2.z <= maxPosBox1.z;
    }

    public static bool IsPointInAABB(float3 point, float3 minPosBox, float3 maxPosBox) { // Untested and unused
        return point.y <= maxPosBox.y && minPosBox.y <= point.y
            && point.x <= maxPosBox.x && minPosBox.x <= point.x
            && point.z <= maxPosBox.z && minPosBox.z <= point.z;
    }

    public static void CapsuleToAABB(float3 capsuleStart, float3 capsuleEnd, float capsuleRadius, out float3 minPosition, out float3 maxPosition) {
        if (capsuleRadius == 0f) {
            RayToAABB(capsuleStart, capsuleEnd, out minPosition, out maxPosition);
        } else {
            float3 minBtwSpheres = math.min(capsuleStart, capsuleEnd);
            float3 maxBtwSpheres = math.max(capsuleStart, capsuleEnd);
            
            float3 toCapsuleMaxPos = new float3(capsuleRadius);
            minPosition = minBtwSpheres - toCapsuleMaxPos;
            maxPosition = maxBtwSpheres + toCapsuleMaxPos;
        }
    }

    public static void SphereToAABB(float3 center, float radius, out float3 minPosition, out float3 maxPosition) {
        float3 toMaxVector = new float3(radius);
        minPosition = center - toMaxVector;
        maxPosition = center + toMaxVector;
    }

    public static void RayToAABB(float3 rayStart, float3 rayEnd, out float3 minPosition, out float3 maxPosition) {
        minPosition = math.min(rayStart, rayEnd);
        maxPosition = math.max(rayStart, rayEnd);
    }

    // public static AABB AABBFromMinMax(float3 minPosition, float3 maxPosition) { // TODO: Doesn't work
    //     AABB aabb = new AABB();
    //     aabb.Center = (minPosition + maxPosition)/2;
    //     aabb.Extents = maxPosition - aabb.Center;
    //     return aabb;
    // }

    public static float3 CellCoordsToWorld(int2 cellCoords, float3 bottomLeftWorld, float cellSize) {
        return bottomLeftWorld + (new float3(cellCoords.x+0.5f, 0, cellCoords.y+0.5f) * cellSize);
    }
    public static float3 CellCoordsBottomLeftToWorld(int2 cellCoords, float3 bottomLeftWorld, float cellSize) {
        return bottomLeftWorld + (new float3(cellCoords.x, 0, cellCoords.y) * cellSize);
    }
    public static int2 WorldToCellCoordsXZ(float3 position, float3 bottomLeftWorld, float cellSize) { return WorldToCellCoords(position.xz, bottomLeftWorld.xz, cellSize); }
    public static int2 WorldToCellCoords(float2 position, float2 bottomLeftWorld, float cellSize) {
        float2 relativeToBottomLeftPos = (position - bottomLeftWorld)/ cellSize;
        return new int2((int)relativeToBottomLeftPos.x, (int)relativeToBottomLeftPos.y);
    }

    // If given normal = math.up() and localOrigin = float3.zero, these CalcLocalMatrices will be equivalent to the default Unity origin matrix normal is the local up vector
    public static float4x4 CalcLocalMatrixTRS(float3 normal, float3 localOrigin, float angle, float scale = 1f) {
        return float4x4.TRS(localOrigin, math.mul(quaternion.AxisAngle(normal, angle), CalcRotationFromNormal(normal)), scale);
    }
    // tangent is the local ?? vector (i.e. forward, left, or right?)
    public static float4x4 CalcLocalMatrix(float3 normal, float3 tangentApprox, float3 localOrigin) { // TODO: Rename tangentGuess?
        Debug.Assert(!IsParallel(normal, tangentApprox), "tangent cannot not be parallel to normal");
        float3 bitangent = math.normalize(math.cross(tangentApprox, normal));
        return new float4x4(quaternion.LookRotation(bitangent, normal), localOrigin);
    }
    public static float4x4 CalcLocalMatrix(float3 normal, float3 localOrigin) {
        return new float4x4(CalcRotationFromNormal(normal), localOrigin);
    }
    public static float4x4 CalcInvLocalMatrixTRS(float3 normal, float3 localOrigin, float angle, float scale = 1f) => math.inverse(CalcLocalMatrixTRS(normal, localOrigin, angle, scale));
    public static float4x4 CalcInvLocalMatrix(float3 normal, float3 tangent, float3 localOrigin) => math.inverse(CalcLocalMatrix(normal, tangent, localOrigin));
    public static float4x4 CalcInvLocalMatrix(float3 normal, float3 localOrigin) => math.inverse(CalcLocalMatrix(normal, localOrigin));

    public static float3 CalcTangentToNormal(float3 normal) => CalcTangentToNormal(normal, math.up());
    public static float3 CalcTangentToNormal(float3 normal, float3 upOrBitangentGuess) { // bitangentGuess will usually be math.up()
        Debug.Assert(!math.all(upOrBitangentGuess == math.forward()), "This function will fail if given math.forward() since it uses it as second guess below");
        float3 tangent = math.cross(normal, upOrBitangentGuess); // Pick world up vector to try to make a tangent to the normal
        if (math.lengthsq(tangent) <= 0.0001f) { // If normal is same direction as up, then use world forward vector to make a tangent instead
            tangent = math.cross(normal, math.forward());
        }
        return math.normalize(tangent);
    }
    // TODO: Does this need to be normalized?
    public static quaternion CalcRotationFromNormal(float3 normal) {
        float3 tangent = CalcTangentToNormal(normal);
        float3 bitangent = math.normalize(math.cross(tangent, normal)); // Example: bitangent would be upVector for a wall // Doesn't need checks
        return quaternion.LookRotation(bitangent, normal);
    }
    public static float ConvertToUnityAngle(float angle) { return Rad270 - angle; }
    public static float CalcHeadingAngle(float3 start, float3 end) { return CalcHeadingAngle(end - start); }
    public static float CalcHeadingAngle(float3 startToEndVector) { return math.atan2(startToEndVector.z, startToEndVector.x); }
    public static quaternion CalcHeadingRotation(float3 start, float3 end) {
        return quaternion.RotateY(CalcHeadingAngle(start, end));
    }

    public static float3 PointWorldToLocal(float3 worldPoint, float4x4 invLocalMatrix) {
        float4 localPoint = math.mul(invLocalMatrix, new float4(worldPoint, 1f));
        return new float3(localPoint.xyz);
    }
    public static float3 PointLocalToWorld(float3 localPoint, float4x4 localMatrix) {
        float4 worldPoint = math.mul(localMatrix, new float4(localPoint, 1f));
        return new float3(worldPoint.xyz);
    }

    public static float3 VectorWorldToLocal(float3 worldVector, float4x4 invLocalMatrix) {
        float4 localVector = math.mul(invLocalMatrix, new float4(worldVector, 0f));
        return new float3(localVector.xyz);
    }
    public static float3 VectorLocalToWorld(float3 localVector, float4x4 localMatrix) {
        float4 worldVector = math.mul(localMatrix, new float4(localVector, 0f));
        return new float3(worldVector.xyz);
    }

    // Perp and para relative to vector // https://math.stackexchange.com/questions/1008529/finding-the-perpendicular-components-of-a-vector
    public static void CalcPerpParaFromNormal(float3 vector, float3 normal, out float3 perpendicular, out float3 parallel) {
        parallel = math.project(vector, normal);
        perpendicular = vector - parallel;
    }

    public static void CalcVertHorizComponents(float3 start, float3 end, out float vertical, out float horizontal) {
        CalcVertHorizComponents(end - start, out vertical, out horizontal);
    }
    public static void CalcVertHorizComponents(float3 vector, out float vertical, out float horizontal) {
        vertical = vector.y;
        horizontal = math.length(new float3(vector.x, 0, vector.z));
    }

    public static float3 GetMaxXYZPoint(in NativeArray<float3> points, out int pointMaxXYZIndex) {
        float3 pointMaxXYZ = new float3(float.MinValue);
        pointMaxXYZIndex = -1;
        for (int i = 0; i < points.Length; i++) {
            bool isBestPoint = false;
            if (points[i].x > pointMaxXYZ.x) {
                isBestPoint = true;
            } else if (points[i].x == pointMaxXYZ.x) {
                if (points[i].y > pointMaxXYZ.y) {
                    isBestPoint = true;
                } else if (points[i].y == pointMaxXYZ.y) {
                    if (points[i].z > pointMaxXYZ.z) {
                        isBestPoint = true;
                    } else if (points[i].z == pointMaxXYZ.z) {
                        Debug.Log("GetMaxXYZPoint - Duplicate point, potentially bad");
                    }
                    // if z tie, then they are same point
                }
            }
            if (isBestPoint) {
                pointMaxXYZ = points[i];
                pointMaxXYZIndex = i;
            }
            // if (math.all(vertexPoints[i] >= pointMaxXYZ)) { // Doesn't work because is component-wise
            //     pointMaxXYZ = vertexPoints[i];
            //     pointMaxXYZIndex = i;
            // }
        }
        Debug.Assert(pointMaxXYZIndex != -1, "TODO, what could cause this?");
        return pointMaxXYZ;
    }
    // Assumes polygon is planar
    public static float3 CalcPolygonNormalCCW(in NativeArray<float3> vertexPoints) => CalcPolygonNormalCCW(vertexPoints, out int _);
    public static float3 CalcPolygonNormalCCW(in NativeArray<float3> vertexPoints, out int pointMaxXYZIndex) { // Normal points towards screen if CCW relative to screen
        
        int n = vertexPoints.Length;
        float3 pointMaxXYZ = GetMaxXYZPoint(vertexPoints, out pointMaxXYZIndex);
        
            /* float3 goodNormal = math.normalize(math.cross(vertexPoints[NextIndex(i_PointMaxXYZ, n)] - pointMaxXYZ, pointMaxXYZ - vertexPoints[PrevIndex(i_PointMaxXYZ, n)]));
            float3 badNormal = CalcNormalFrom3PointsCCW(vertexPoints[0], vertexPoints[1], vertexPoints[2]);
            if (!IsEpsilonEqual(goodNormal, badNormal, 0.001f)) {
                Debug.Log("NOT EQUAL!! ==========================");
            } */
        Debug.Assert(pointMaxXYZIndex != NextIndex(pointMaxXYZIndex, n) && pointMaxXYZIndex != PrevIndex(pointMaxXYZIndex, n));
        return math.normalize(math.cross(vertexPoints[NextIndex(pointMaxXYZIndex, n)] - pointMaxXYZ, pointMaxXYZ - vertexPoints[PrevIndex(pointMaxXYZIndex, n)]));
        // if (IsFloat3NaN(normal)) {
        //     float3 vec1 = vertexPoints[NextIndex(pointMaxXYZIndex, n)] - pointMaxXYZ;
        //     float3 vec2 = pointMaxXYZ - vertexPoints[PrevIndex(pointMaxXYZIndex, n)];
        //     Debug.Log("vertexPoints.Length: " + vertexPoints.Length);
        //     Debug.Log("math.length(vec2): " + math.length(vec2) + ",  math.length(vec1): " + math.length(vec1));
        //     // Debug.Log("AreVectorsParallel: " + AreVectorsParallel(vec1, vec2));
        //     // Debug.Log("pointMaxXYZ: " + pointMaxXYZ);
        //     // Debug.Log("NextIndex(pointMaxXYZIndex, n)]: " + vertexPoints[NextIndex(pointMaxXYZIndex, n)]);
        //     // Debug.Log("PrevIndex(pointMaxXYZIndex, n)]: " + vertexPoints[PrevIndex(pointMaxXYZIndex, n)]);
        // }
        // return normal;
    }
    public static float3 CalcPolygonConvexNormalCCW(in NativeArray<float3> vertexPoints) { // Normal points towards screen if CCW relative to screen
        float3 normal = CalcTriangleNormalCCW(vertexPoints[0], vertexPoints[1], vertexPoints[2]);
        if (float.IsNaN(normal.x)) { normal = CalcPolygonNormalCCW(vertexPoints); } // 0,1,2 points could be in a straight line, must check for that and fix it
        return normal;
    }
    public static float3 CalcBisectionTails(float3 vector0, float3 vector1, float3 normalUsedIfParallel) {
        return CalcBisectionUnitTails(math.normalize(vector0), math.normalize(vector1), normalUsedIfParallel);
    }
    public static float3 CalcBisectionUnitTails(float3 vectorUnit0, float3 vectorUnit1, float3 normalUsedIfParallel) {
        if (IsParallelUnit(vectorUnit0, vectorUnit1, 0.001f)) {
            return math.normalize(math.cross(normalUsedIfParallel, vectorUnit0));
        }
        return math.normalize(vectorUnit0 + vectorUnit1);
    }
    // Will return NaN if points are collinear
    public static float3 CalcTriangleNormalCCW(float3 point0, float3 point1, float3 point2) { // Normal points towards screen if CCW relative to screen
        return CalcTriangleNormalCCW(point2 - point1, point1 - point0);
    }
    public static float3 CalcTriangleNormalCCW(float3 edgeVector0, float3 edgeVector1) { // Normal points towards screen if CCW relative to screen
        return math.normalize(math.cross(edgeVector0, edgeVector1));
    }
    public static void CalcPolygonPlaneCCW(in NativeArray<float3> vertexPoints, out float3 planeNormal, out float planeDistance) {
        planeNormal = CalcPolygonNormalCCW(vertexPoints, out int pointMaxXYZIndex); // ax + by + cz = d -> Normal = (a,b,c), Distance = d
        planeDistance = CalcPlaneDistance(planeNormal, vertexPoints[pointMaxXYZIndex]); // math.dot(planeNormal, vertexPoints[pointMaxXYZIndex]);
    }
    public static void CalcTrianglePlaneCCW(float3 point0, float3 point1, float3 point2, out float3 planeNormal, out float planeDistance) {
        planeNormal = CalcTriangleNormalCCW(point0, point1, point2); // ax + by + cz = d -> Normal = (a,b,c), Distance = d
        planeDistance = CalcPlaneDistance(planeNormal, point0); // math.dot(planeNormal, point0);
    }

    public static float CalcPlaneDistance(float3 planeNormal, float3 arbitraryPointOnPlane) {
        return math.dot(planeNormal, arbitraryPointOnPlane);
    }

    public static float3 CalcArbitraryPointOnPlane(float3 planeNormal, float planeDistance) {
        return planeNormal * planeDistance;
    }

    // For non-planar polygons
    public static void CalcPointsPlaneAverage(in NativeArray<float3> points, out float3 planeNormalAverage, out float planeDistanceAverage) {
        float3 planeNormalSum = float3.zero;
        float planeDistanceSum = 0f;

        int i_Prev = points.Length-1;
        for (int i = 0; i < points.Length; i++) {
            int i_Next = NextIndex(i, points.Length);
            CalcTrianglePlaneCCW(points[i_Prev], points[i], points[i_Next], out float3 planeNormal, out float planeDistance);
            planeNormalSum += planeNormal;
            planeDistanceSum += planeDistance;
            
            i_Prev = i;
        }
        planeNormalAverage = math.normalize(planeNormalSum/points.Length);
        planeDistanceAverage = planeDistanceSum/points.Length;
    }

    // public static quaternion CalcRotationFromDirection(float3 normal) {
    //     float3 tangent = CalcTangentToNormal(normal);
    //     float3 bitangent = math.cross(tangent, normal); // Example: bitangent would be upVector for a wall
    //     return quaternion.LookRotation(normal, bitangent);
    // }

    public static bool IsRaysIntersectingXZ(float3 start0, float3 vector0, float3 start1, float3 vector1, out float3 intersectionPoint) {
        bool isIntersecting = IsSegsIntersecting(start0.xz, (start0 + vector0).xz, start1.xz, (start1 + vector1).xz, out float2 intersection2);
        intersectionPoint = new float3(intersection2.x, 0, intersection2.y);
        return isIntersecting;
    }
    public static bool IsRaysIntersecting(float2 start0, float2 vector0, float2 start1, float2 vector1, out float2 intersectionPoint) {
        return IsSegsIntersecting(start0, start0 + vector0, start1, start1 + vector1, out intersectionPoint);
    }
    public static bool IsSegsIntersectingXZ(float3 p00, float3 p01, float3 p10, float3 p11, out float3 intersectionPoint) {
        bool isIntersecting = IsSegsIntersecting(p00.xz, p01.xz, p10.xz, p11.xz, out float2 intersection2); intersectionPoint = new float3(intersection2.x, 0, intersection2.y);
        return isIntersecting;
    }
    public static bool IsSegsIntersectingXY(float3 p00, float3 p01, float3 p10, float3 p11, out float3 intersectionPoint) {
        bool isIntersecting = IsSegsIntersecting(p00.xy, p01.xy, p10.xy, p11.xy, out float2 intersection2); intersectionPoint = intersection2.XYO();
        return isIntersecting;
    }
    // Modified version of: https://stackoverflow.com/a/1968345/8132000
    public static bool IsSegsIntersecting(float2 p00, float2 p01, float2 p10, float2 p11, out float2 intersectionPoint)
    {
        float2 s1 = p01 - p00;
        float2 s2 = p11 - p10;

        float det = -s2.x * s1.y + s1.x * s2.y;
        float u = (-s1.y * (p00.x - p10.x) + s1.x * (p00.y - p10.y)) / det;
        float t = (s2.x * (p00.y - p10.y) - s2.y * (p00.x - p10.x)) / det;

        if (u >= 0 && u <= 1 && t >= 0 && t <= 1) { // Found collision
            intersectionPoint = new float2(p00.x + (t * s1.x), p00.y + (t * s1.y));
            return true;
        }
        intersectionPoint = float2.zero;
        return false; // No collision
    }

    public static bool IsLocalRaysIntersectingXZ(float4x4 invLocalMatrix, float3 startPoint0, float3 vector0, float3 startPoint1, float3 vector1, out float2 intersectionPoint) {
        float3 currLoopLocalP0 = PointWorldToLocal(startPoint0, invLocalMatrix);            // Debug.Log("currLoopLocalP0: " + currLoopLocalP0);
        float3 currLoopLocalP1 = PointWorldToLocal(startPoint0 + vector0, invLocalMatrix);  // Debug.Log("currLoopLocalP1: " + currLoopLocalP1);
        float3 nextLoopLocalP0 = PointWorldToLocal(startPoint1, invLocalMatrix);            // Debug.Log("nextLoopLocalP0: " + nextLoopLocalP0);
        float3 nextLoopLocalP1 = PointWorldToLocal(startPoint1 + vector1, invLocalMatrix);  // Debug.Log("nextLoopLocalP1: " + nextLoopLocalP1);

        return IsSegsIntersecting(currLoopLocalP0.xz, currLoopLocalP1.xz, nextLoopLocalP0.xz, nextLoopLocalP1.xz, out intersectionPoint);
    }

    public static float SamplePlaneElevation(float3 positionXZ, float3 planeNormal, float planeDistance) {
        Debug.Assert(planeNormal.y != 0, "plane cannot be vertical");
	    return (planeDistance - (positionXZ.x * planeNormal.x) - (positionXZ.z * planeNormal.z)) / planeNormal.y;
    }

    public static int Index2DTo1D(int2 coords, int numRows) { return Index2DTo1D(coords.x, coords.y, numRows); }
    public static int Index2DTo1D(int coordX, int coordY, int numRows) {
        return coordX + (coordY * numRows); // (row * numRows) + column
    }

    public static int2 Index1DTo2D(int index, int numRows) {
        int x = index % numRows;
        int y = index / numRows;
        return new int2(x, y);
    }

    public static float CalcTriangleHeight(float triArea, float triBaseLength) { // A = (1/2)*b*h   ->   h = 2*(A/b)
        return 2f * (triArea / triBaseLength);
    }

    // https://stackoverflow.com/questions/1406029/how-to-calculate-the-volume-of-a-3d-mesh-object-the-surface-of-which-is-made-up
    public static float MeshVolume(Mesh mesh) { // TODO: Untested
        float sum = 0;
        /* for (int i = 0; i < mesh.triangles.Length; i++) {
            Triangle t = mesh.triangles[i];
            sum += SignedTriangleVolume(t.P1, t.P2, t.P3);
        } */
        return math.abs(sum);
    }
    static float SignedTriangleVolume(float3 p0, float3 p1, float3 p2) { // TODO: Try doing 1/6 at the end inside MeshVolume() instead of here
        return math.dot(p0, math.cross(p1, p2)) / 6f;
    }

    public static void SpaceOut(float fullLength, float endsSpacing, float targetBtwSpacing, bool isRespaceBtwBtw, out int numSpots, out float actualBtwSpacing, out float startOffset) {

        // float numWindowsFloat = wallLength / midSpacing; // Unfinished to make ends spacing change too

        float midLength = fullLength - endsSpacing*2;
        numSpots = (int)math.ceil(midLength / targetBtwSpacing); // Choice between ceil or round+1 // Keeping it ceil for now cuz looks best
        numSpots = (numSpots <= 0) ? 1 : numSpots; // TODO: Can't have 0 windows? Or maybe just return if 0

        if (numSpots != 1) {
            startOffset = endsSpacing;
            actualBtwSpacing = isRespaceBtwBtw ? midLength/(numSpots - 1) : targetBtwSpacing; // To avoid divide by 0
        } else {
            startOffset = fullLength/2; // This will position the window to center of wall
            actualBtwSpacing = default;
        }
    }


    public static bool Is3DPolygonsIntersecting() {
        
        return false;
    }

    // public static bool IsPolygonsIntersectingXZ(in Polygon polygon0, in Polygon polygon1) {
    //     Debug.Assert(polygon0.IsValid() || polygon1.IsValid(), "Invalid polygon");
    //     if (polygon0.NumVertices > polygon1.NumVertices) { // Minor optimization that potentially reduces for loop iterations
    //         return SeparatingAxisTheorem(polygon0, polygon1) && SeparatingAxisTheorem(polygon1, polygon0);
    //     } else {
    //         return SeparatingAxisTheorem(polygon1, polygon0) && SeparatingAxisTheorem(polygon0, polygon1);
    //     }
    // }
    // static bool SeparatingAxisTheorem(in Polygon polygon0, in Polygon polygon1) { // TODO: Add XZ suffix?
    //     for (int i = 0; i < polygon0.NumVertices; i++) {
    //         float3 edgeRight = -polygon0.GetEdgeRight(i);
    //         float3 edgeVert = polygon0.GetVertexPoint(i);

    //         CommonLib.DebugVector(polygon0.GetEdgeMidpoint(i), edgeRight*0.2f, Color.blue, 0.015f, 0.1f);

    //         bool isOverlapping = false;
    //         for (int j = 0; j < polygon1.NumVertices; j++) { // TODO: Use a MathLib PointToPlane function like below
    //             float projectedDistance = math.dot(edgeRight, polygon1.GetVertexPoint(j) - edgeVert);
    //             // float projectedDistance = PointToPlaneDistanceSigned(polygon1.GetVertexPoint(j), edgeRight, edgeVert);
    //             if (projectedDistance < 0) {
    //                 isOverlapping = true; // Do not add break statement
    //             }
    //         }
    //         if (!isOverlapping) {
    //             return false;
    //         }
    //     }
    //     return true;
    // }

    // // SutherlandHodgman, burstified from Habrador
    // public static NativeList<float3> ClipPolygon(in Polygon polygon, Polygon polygonClip, bool isProjClipToWorldXZ) {
    //     Debug.Assert(polygonClip.IsConvex(out bool _, 0f), "polygonClip must be convex");
    //     Debug.Assert(!isProjClipToWorldXZ || !IsVectorXZPlane(polygonClip.Plane.normal), "If isProjClipToXZ == true, then polygonClip cannot be vertical (i.e. normal cannot be XZ plane)");
        
    //     // if (isProjClipToXZ) { polygonClip.ProjToXZPlane(); CommonLib.DebugPolygon(polygonClip, Color.black, 0f, 0.025f, false, 0.1f); }

    //     NativeList<float3> clipVertexPoints = new NativeList<float3>(polygon.NumVertices, Allocator.Temp);
    //     clipVertexPoints.AddRange(polygon.VertexPointsRO);

    //     NativeList<float3> clipVertexPointsTemp = new NativeList<float3>(polygon.NumVertices, Allocator.Temp);

    //     for (int i = 0; i < polygonClip.NumVertices; i++) {
    //         // TODO: ClipNormal should be perpendicular to up()
    //         float3 edgeVector = polygonClip.GetEdgeVector(i);
    //         float3 planeNormal = polygonClip.Plane.normal;
    //         if (isProjClipToWorldXZ) { edgeVector.y = 0; planeNormal = math.up(); }

    //         float3 edgeRight_ClipNormal = -polygonClip.GetEdgeRight(i); // Clipping plane normal, points inwards from polygon perspective

    //         float3 edgeVert_ClipPos = polygonClip.GetVertexPoint(i); // Clipping plane pos, arbitrarily on the clipping plane
    //         float edgeVert_ClipPlaneDist = CalcPlaneDistance(edgeRight_ClipNormal, edgeVert_ClipPos); // Clipping plane dist

    //         CommonLib.DebugVector(polygonClip.GetEdgeMidpoint(i), edgeRight_ClipNormal*0.2f, Color.blue, 0.015f, 0.1f);

    //         // bool isOverlapping = false;
    //         for (int j = 0; j < clipVertexPoints.Length; j++) {

    //             float3 vert0Point = clipVertexPoints[j];
    //             float3 vert1Point = clipVertexPoints[NextIndex(j, clipVertexPoints.Length)];

    //             float point0ToClipDist = PointToPlaneDistanceSigned(vert0Point, edgeRight_ClipNormal, edgeVert_ClipPos); // TODO: Make negative or flip Leq/Geq below
    //             float point1ToClipDist = PointToPlaneDistanceSigned(vert1Point, edgeRight_ClipNormal, edgeVert_ClipPos);

    //             //TODO: What will happen if they are exactly 0? Should maybe use a tolerance of 0.001

    //             //Case 1. Both are outside (= to the right), do nothing 

    //             float3 rayVector = vert1Point - vert0Point;
    //             if (point0ToClipDist >= 0f && point1ToClipDist >= 0f) { // Case 2. Both are inside (= to the left), save vert1Point
    //                 clipVertexPointsTemp.Add(vert1Point);
    //             } else if (point0ToClipDist < 0f && point1ToClipDist >= 0f) { // Case 3. Outside -> Inside, save intersection point and vert1Point
    //                 if (!IsRayPlaneIntersecting(vert0Point, math.normalize(rayVector), math.length(rayVector)*5f, edgeRight_ClipNormal, edgeVert_ClipPlaneDist, out float3 nearestPointToPlane)) {
    //                     Debug.Assert(false, "Glitch");
    //                 }
    //                 clipVertexPointsTemp.Add(nearestPointToPlane);

    //                 clipVertexPointsTemp.Add(vert1Point);

    //             } else if (point0ToClipDist >= 0f && point1ToClipDist < 0f) { // Case 4. Inside -> Outside, save intersection point
    //                 if (!IsRayPlaneIntersecting(vert0Point, math.normalize(rayVector), math.length(rayVector)*5f, edgeRight_ClipNormal, edgeVert_ClipPlaneDist, out float3 nearestPointToPlane)) {
    //                     Debug.Assert(false, "Glitch");
    //                 }
    //                 clipVertexPointsTemp.Add(nearestPointToPlane);
    //             }

    //             // if (isAddRayClipPlaneIntersection) {
    //             //     float3 rayVector = vert1Point - vert0Point;
    //             //     if (IsRayPlaneIntersecting(vert0Point, math.normalize(rayVector), math.length(rayVector), edgeRight_ClipNormal, edgeVert_ClipPlaneDist, out float3 nearestPointToPlane)) {
    //             //         clipVertexPointsTemp.Add(nearestPointToPlane);
    //             //     } else {
    //             //         Debug.Assert(false, "Glitch");
    //             //     }
    //             // }
    //         }

    //         //Add the new vertices to the list of vertices
    //         clipVertexPoints.Clear();

    //         clipVertexPoints.AddRange(clipVertexPointsTemp.AsArray());

    //         clipVertexPointsTemp.Clear();
    //     }
    //     return clipVertexPoints;
    // }

    // TODO: Is Grid an incorrect word? Grid should mean both vertical and horizontal gridlines. This function only does vertical (Same TODO as below)
    public static NativeList<float2> SegGridIntersections(float2 rayStart, float2 rayEnd, float2 bottomLeftCorner, float cellSize, bool is_OnlyX = false) {
        NativeList<float2> intersectionPoints = new NativeList<float2>((int)(math.length(rayEnd - rayStart)/cellSize), Allocator.Temp);
        // int numXGridLinesBtw = math.abs(endCoords.x - startCoords.x)+1;
        // int numYGridLinesBtw = (math.abs(endCoords.y - startCoords.y)+1) * (is_OnlyX ? 0 : 1);
        // intersectionPoints.SetCapacity(numXGridLinesBtw + numYGridLinesBtw);
        SegGridIntersections(rayStart, rayEnd, ref intersectionPoints, bottomLeftCorner, cellSize, is_OnlyX);
        return intersectionPoints;
    }
    public static NativeList<float2> SegGridIntersections(float2 segStart, float2 segEnd, ref NativeList<float2> intersectionPoints, float2 bottomLeftCorner, float cellSize, bool is_OnlyX = false) {
        Debug.Assert(!IsEpsEqual(segStart, segEnd, 0.001f), "start and end cannot be the same point");
        Debug.Assert(math.all(bottomLeftCorner <= segStart) && math.all(bottomLeftCorner <= segEnd), $"bottomLeftCorner must be LEQ start and end (X and Y), bottomLeftCorner: {bottomLeftCorner},  segStart: {segStart},  segEnd: {segEnd}");

        float2 segVector = segEnd - segStart;
        float2 startRelativeToBottomLeftPos = (segStart - bottomLeftCorner) / cellSize; // Needed

        int2 startCoords = WorldToCellCoords(segStart, bottomLeftCorner, cellSize);
        int2 endCoords = WorldToCellCoords(segEnd, bottomLeftCorner, cellSize);

        int signY = (segVector.y > 0) ? 1 : -1;
        int signX = (segVector.x > 0) ? 1 : -1;
        float slopeM = segVector.y / segVector.x;
        if (segVector.x == 0f) { slopeM = 100000f; }
        float interceptB = (-slopeM * startRelativeToBottomLeftPos.x) + startRelativeToBottomLeftPos.y;

        int numXGridLinesBtw = math.abs(endCoords.x - startCoords.x);
        int boolX = (signX > 0) ? 1 : 0; // For offsetting x axis values when seg is negative x
        int boolY = (signY > 0) ? 1 : 0;

        int currentX = startCoords.x + (1 - boolX);
        int currentY = startCoords.y;
        for (int i = 0; i < numXGridLinesBtw; i++) {
            int X = startCoords.x + (i * signX) + boolX;
            float YExact = slopeM * X + interceptB;
            int Y = (int)YExact; // y = m * x + b (where b is initial z position)

            
            if (!is_OnlyX) {
                for (int Y0 = currentY; Y0 != Y; Y0 += signY) {
                    float XExact = ((Y0 + boolY) - interceptB) / slopeM; // x = (y - b)/m
                    intersectionPoints.Add(bottomLeftCorner + (cellSize * new float2(XExact, (Y0 + boolY))));

                    if (math.abs(Y0 - Y) > 100) { Debug.Assert(false, "Too many, infinite loop, may be outside bottomLeftCorner"); break; }
                }
            }
            intersectionPoints.Add(bottomLeftCorner + (cellSize * new float2((currentX + signX), YExact)));
            currentX = X;  currentY = Y;
        }

        if (!is_OnlyX) {
            for (int Y0 = currentY; Y0 != endCoords.y; Y0 += signY) {
                float XExact = ((Y0 + boolY) - interceptB) / slopeM; // x = (y - b)/m
                intersectionPoints.Add(bottomLeftCorner + (cellSize * new float2(XExact, (Y0 + boolY))));

                if (math.abs(Y0 - endCoords.y) > 100) { Debug.Assert(false, "Too many, infinite loop, may be outside bottomLeftCorner"); break; }
            }
        }

        return intersectionPoints;
    }

    // // TODO: is_OnlyX seems like it should not be a parameter and always given to SegGridIntersections()'s parameter as true
    // // TODO: Is Grid an incorrect word? Grid should mean both vertical and horizontal gridlines. This function only does vertical (Same TODO as above)
    // // Takes only the xz plane of polygon
    // public static void PolygonGridIntersectionsTopBottom(in Polygon polygon, float3 bottomLeftCorner, float cellSize,
    //     out NativeList<float2> bottomEdgeIntersections, out NativeList<float2> topEdgeIntersections, bool is_OnlyX = false) // Add check if ray is out of bounds
    // {
    //     Debug.Assert(polygon.IsValid(), "Invalid polygon");
    //     // Debug.Log("polygon.IsGroundCCW: " + polygon.IsGroundCCW);
    //     Debug.Assert(polygon.NumVertices > 0, "Empty polygon");

    //     bottomEdgeIntersections = new NativeList<float2>(1, Allocator.Temp); // All cell Intersections of only rasterized bottom edge of polygon, and only one cell coord per X
    //     topEdgeIntersections = new NativeList<float2>(1, Allocator.Temp);    // Same but for top edge
    //     NativeList<float2> currentEdgeIntersections; // Will reference either bottomEdge or topEdge Intersections

    //     // Find the first instance around the polygon that edges switch from top to bottom (end) or vice versa:
    //     int startingEdgeIndex = FindFirstEdgeSwitchingEnds(polygon, out bool isCurrentEdgeTop);
    //     currentEdgeIntersections = isCurrentEdgeTop ? topEdgeIntersections : bottomEdgeIntersections;
    //     Debug.Assert(startingEdgeIndex != -1, "Broken polygon");

    //     // int colorIndex = 0; // Debug
    //     for (int i0 = startingEdgeIndex; i0 < polygon.NumVertices + startingEdgeIndex; i0++) { // Now goes around polygon edges and adds to current end Intersections
    //         int index = i0;
    //         if (i0 >= polygon.NumVertices) { // TODO: Replace with mod
    //             index = i0 - polygon.NumVertices;
    //         }
    //         int i = i0 % polygon.NumVertices; // Pretty sure this is the answer to the above TODO
    //         Debug.Assert(i == index, "Indices not equal");

    //         float3 edgeVector = polygon.GetEdgeVectorWise(index, polygon.IsGroundCCW);
    //         // CommonLib.CreatePrimitive(PrimitiveType.Cube, edge.CalcMidpoint(), new float3(0.05f), CommonLib.CycleColors[colorIndex++]);

    //         bool isTopEdge = edgeVector.x > 0;
    //         if (isTopEdge ^ isCurrentEdgeTop) { // If detect a switch of ends based on edge direction, switch current Intersections and don't remove last element
    //             // Debug.Log("Switching ends");
    //             isCurrentEdgeTop = isTopEdge; // Update our bool variable that tells us which end we are currently on
    //             currentEdgeIntersections = isTopEdge ? topEdgeIntersections : bottomEdgeIntersections; // Switch to correct end Intersections list (top or bottom)
    //         }
    //         float3 edgeStart = polygon.GetVertexPointWise(index, polygon.IsGroundCCW);  float3 edgeEnd = polygon.GetNextVertexPointWise(index, polygon.IsGroundCCW);
    //         SegGridIntersections(edgeStart.xz, edgeEnd.xz, ref currentEdgeIntersections, bottomLeftCorner.xz, cellSize, is_OnlyX);
    //     }
    // }

    // public static NativeList<int2> RasterPolygon(in Polygon polygon, float3 bottomLeftCorner, float cellSize, out IntRect XYBounds, bool isOnlyCellCenterInside = false, int inset = 0) // Add check if ray is out of bounds
    // {
    //     Debug.Assert(polygon.IsValid(), "Invalid polygon");

    //     RasterPolygonTopBottom(polygon, bottomLeftCorner, cellSize, out NativeList<int2> bottomEdgeCellCoords, out NativeList<int2> topEdgeCellCoords);

    //     NativeList<int2> rasterCellCoords = new NativeList<int2>(1, Allocator.Temp); // All cell coords of fully rasterized polygon (if a cell is barely clipped, it will still be included)
    //     // bottomEdgeCellCoords.(default, 0);
    //     Debug.Assert(bottomEdgeCellCoords.Length == topEdgeCellCoords.Length);
    //     int numXCoords = bottomEdgeCellCoords.Length;
    //     XYBounds = new IntRect(bottomEdgeCellCoords[numXCoords-1].x, int.MinValue, bottomEdgeCellCoords[0].x, int.MaxValue);
    //     Debug.Assert(XYBounds.xMin <= XYBounds.xMax, "minX must be less than or equal to maxX");

    //     for (int i = inset; i < numXCoords - inset; i++) { // Now iterate from bottomY to topY for each X and add new int2(X, Y) to final raster coords
    //         int X = bottomEdgeCellCoords[i].x;
    //         int bottomY = bottomEdgeCellCoords[i].y; // (numXCoords - 1) - i
    //         int topY = topEdgeCellCoords[ReverseIndex(i, numXCoords)].y; // topEdgeCellCoords are in reverse order of bottomEdgeCellCoords
    //         XYBounds.yMin = math.min(bottomY, XYBounds.yMin);
    //         XYBounds.yMax = math.max(topY, XYBounds.yMax);

    //         Debug.Assert(bottomY <= topY, "topY must be greater than or equal to bottomY");
    //         for (int Y = bottomY + inset; Y <= topY - inset; Y++) {
    //             // Debug.Log("Is 1D: " + Index2DTo1D(new int2(X, Y)));
    //             if (isOnlyCellCenterInside && !polygon.IsPointInConvex(CellCoordsToWorld(new int2(X, Y), bottomLeftCorner, cellSize))) { continue; }
    //             rasterCellCoords.Add(new int2(X, Y));
    //         }
    //     }
    //     return rasterCellCoords;
    // }
    // static int FindFirstEdgeSwitchingEnds(in Polygon polygon, out bool isStartingEdgeTop) {
    //     Debug.Assert(polygon.IsValid(), "Invalid polygon");
    //     bool isCurrentEdgeTop = polygon.GetEdgeVectorWise(0, polygon.IsGroundCCW).x > 0;
    //     for (int i = 1; i < polygon.NumVertices; i++) { // Finds the first instance around the polygon that edges switch from top to bottom (end) or vice versa
    //         bool isTopEdge = polygon.GetEdgeVectorWise(i, polygon.IsGroundCCW).x > 0;
    //         if (isTopEdge ^ isCurrentEdgeTop) {
    //             isStartingEdgeTop = isTopEdge;
    //             return i;
    //         }
    //     }
    //     isStartingEdgeTop = false;
    //     return -1;
    // }
    // public static void RasterPolygonTopBottom(in Polygon polygon, float3 bottomLeftCorner, float cellSize,
    //     out NativeList<int2> bottomEdgeCellCoords, out NativeList<int2> topEdgeCellCoords) // Add check if ray is out of bounds
    // {
    //     Debug.Assert(polygon.IsValid(), "Invalid polygon");
    //     // Debug.Log("polygon.IsGroundCCW: " + polygon.IsGroundCCW);
    //     Debug.Assert(polygon.NumVertices > 0, "Empty polygon");

    //     bottomEdgeCellCoords = new NativeList<int2>(1, Allocator.Temp); // All cell coords of only rasterized bottom edge of polygon, and only one cell coord per X
    //     topEdgeCellCoords = new NativeList<int2>(1, Allocator.Temp);    // Same but for top edge
    //     NativeList<int2> currentEdgeCellCoords; // Will reference either bottomEdge or topEdge CellCoords

    //     // Find the first instance around the polygon that edges switch from top to bottom (end) or vice versa:
    //     int startingEdgeIndex = FindFirstEdgeSwitchingEnds(polygon, out bool isCurrentEdgeTop);
    //     currentEdgeCellCoords = isCurrentEdgeTop ? topEdgeCellCoords : bottomEdgeCellCoords;
    //     currentEdgeCellCoords.Add(new int2(int.MaxValue)); // To make the RemoveAt last index below work for the first edge iteration below (i.e. this will be removed)
    //     // Debug.Log("startingEdgeIndex: " + startingEdgeIndex);
    //     // Debug.Log("isTopEdge: " + isTopEdge);
    //     Debug.Assert(startingEdgeIndex != -1, "Broken polygon");

    //     // int colorIndex = 0; // Debug
    //     for (int i0 = startingEdgeIndex; i0 < polygon.NumVertices + startingEdgeIndex; i0++) { // Now goes around polygon edges and adds to current end Coords
    //         int index = i0;
    //         if (i0 >= polygon.NumVertices) { // TODO: Replace with mod
    //             index = i0 - polygon.NumVertices;
    //         }
    //         int i = i0 % polygon.NumVertices; // Pretty sure this is the answer to the above TODO
    //         Debug.Assert(i == index, "Indices not equal");

    //         float3 edgeVector = polygon.GetEdgeVectorWise(index, polygon.IsGroundCCW);
    //         // CommonLib.CreatePrimitive(PrimitiveType.Cube, edge.CalcMidpoint(), new float3(0.05f), CommonLib.CycleColors[colorIndex++]);

    //         int lastElementIndex = currentEdgeCellCoords.Length - 1; int2 lastElement = new int2(int.MaxValue);
    //         bool isTopEdge = edgeVector.x > 0;
    //         if (isTopEdge ^ isCurrentEdgeTop) { // If detect a switch of ends based on edge direction, switch current Coords and don't remove last element
    //             // Debug.Log("Switching ends");
    //             isCurrentEdgeTop = isTopEdge; // Update our bool variable that tells us which end we are currently on
    //             currentEdgeCellCoords = isTopEdge ? topEdgeCellCoords : bottomEdgeCellCoords; // Switch to correct end Coords list (top or bottom)
    //         } else {
    //             // Debug.Log("Removing last");
    //             lastElement = currentEdgeCellCoords[lastElementIndex]; // Save this to compare with next iteration's cell coord to see if will cause gap
    //             currentEdgeCellCoords.RemoveAt(lastElementIndex); // Remove last element so no duplicates because (this?) next iteration will write its coords too
    //         }
    //         bool isZPositive = edgeVector.z > 0;
    //         bool isReverseRay = isTopEdge ^ isZPositive; // Make RasterEdge work properly (enforce top is top raster and vice versa)
    //             Debug.Assert(isReverseRay == ((!isTopEdge && isZPositive) || (isTopEdge && !isZPositive)), "Is somehow NOT same as XOR");
    //         // Raster so only one cell coord per X cell (for each end)
    //         float3 edgeStart = polygon.GetVertexPointWise(index, polygon.IsGroundCCW);  float3 edgeEnd = polygon.GetNextVertexPointWise(index, polygon.IsGroundCCW);

    //             if (IsEpsEqual(edgeStart, edgeEnd, 0.001f)) {
    //                 Debug.LogError("Start and end cannot be the same");
    //                 // CommonLib.DebugPolygon(polygon, Color.blue);
    //             }

    //         RasterEdge(edgeStart.xz, edgeEnd.xz, ref currentEdgeCellCoords, bottomLeftCorner.xz, cellSize, isReverseRay);

    //         // This compares the Y value of the lastElement of the prev edge raster with the first element of this edge raster (same end) and takes the more extreme
    //         // lastElement is also technically the very first element
    //         if (lastElement.x != int.MaxValue) { // This means that Coords list was not swapped (or it's starting edge) and last element of previous iteration was removed
    //             int2 thatReplacedLastElement = currentEdgeCellCoords[lastElementIndex];
    //             // If top edge, then take the most maximum Y,        but if bottom edge, then take the lowest Y
    //             if ((isTopEdge && lastElement.y > thatReplacedLastElement.y) || (!isTopEdge && lastElement.y < thatReplacedLastElement.y)) {
    //                 currentEdgeCellCoords[lastElementIndex] = lastElement;
    //                 // Debug.Log("isTopEdge: " + isTopEdge + "  ,  Replacing");
    //                 // CommonLib.CreatePrimitive(PrimitiveType.Cube, CellCoordsToWorld(thatReplacedLastElement) + new float3(0,0.5f,0.1f), new float3(0.3f), Color.black, default, 5f);
    //                 // CommonLib.CreatePrimitive(PrimitiveType.Cube, CellCoordsToWorld(lastElement) + new float3(0,0.5f,-0.1f), new float3(0.3f), Color.white, default, 5f);
    //             }


    //             /* if (isTopEdge) {
    //                 if (lastElement.y > thatReplacedLastElement.y) {
    //                     Debug.Log("isTopEdge: " + isTopEdge + "  ,  Replacing");
    //                     currentEdgeCellCoords[lastElementIndex] = lastElement;
    //                     CommonLib.CreatePrimitive(PrimitiveType.Cube, CellCoordsToWorld(thatReplacedLastElement) + new float3(0,0.5f,0.1f), new float3(0.3f), Color.black, default, 5f);
    //                     CommonLib.CreatePrimitive(PrimitiveType.Cube, CellCoordsToWorld(lastElement) + new float3(0,0.5f,-0.1f), new float3(0.3f), Color.white, default, 5f);
    //                 }
    //             } else {
    //                 if (lastElement.y < thatReplacedLastElement.y) {
    //                     Debug.Log("isTopEdge: " + isTopEdge + "  ,  Replacing");
    //                     currentEdgeCellCoords[lastElementIndex] = lastElement;
    //                     CommonLib.CreatePrimitive(PrimitiveType.Cube, CellCoordsToWorld(thatReplacedLastElement) + new float3(0,0.5f,0.1f), new float3(0.3f), Color.black, default, 5f);
    //                     CommonLib.CreatePrimitive(PrimitiveType.Cube, CellCoordsToWorld(lastElement) + new float3(0,0.5f,-0.1f), new float3(0.3f), Color.white, default, 5f);
    //                 }
    //             } */
    //         }
    //     }
    // }

    public static void RasterEdge(float2 edgeStart, float2 edgeEnd, ref NativeList<int2> rasterCellCoords, float2 bottomLeftCorner, float cellSize, bool isReverse = false)
    {
        Debug.Assert(!IsEpsEqual(edgeStart, edgeEnd, 0.001f), $"start and end cannot be the same point; {edgeStart}, {edgeEnd}");
        Debug.Assert(math.all(bottomLeftCorner <= edgeStart) && math.all(bottomLeftCorner <= edgeEnd), $"bottomLeftCorner must be LEQ start and end (X and Y), bottomLeftCorner: {bottomLeftCorner},  edgeStart: {edgeStart},  edgeEnd: {edgeEnd}");
        // CommonLib.CreatePrimitive(PrimitiveType.Cube, edgeStart + new float3(0.1f, 0, 0), new float3(0.3f, 0.6f, 0.3f), Color.green, default, 5f);
        // CommonLib.CreatePrimitive(PrimitiveType.Cube, edgeEnd - new float3(0.1f, 0, 0), new float3(0.3f, 0.6f, 0.3f), Color.red, default, 5f);
        float2 rayVector = edgeEnd - edgeStart;
        float2 startRelativeToBottomLeftPos = (edgeStart - bottomLeftCorner) / cellSize;

        int2 startCoords = WorldToCellCoords(edgeStart, bottomLeftCorner, cellSize);
        int2 endCoords = WorldToCellCoords(edgeEnd, bottomLeftCorner, cellSize);

        int signY = (rayVector.y > 0) ? 1 : -1;
        int signX = (rayVector.x > 0) ? 1 : -1;
        float slopeM = rayVector.y / rayVector.x;
        if (rayVector.x == 0f) { slopeM = 100000f; }
        float interceptB = (-slopeM * startRelativeToBottomLeftPos.x) + startRelativeToBottomLeftPos.y;

        int numXGridLinesBtw = math.abs(endCoords.x - startCoords.x);
        int boolX = (signX > 0) ? 1 : 0; // For offsetting x axis values when ray is negative x

        int currentX = startCoords.x + (1 - boolX);  int currentY = startCoords.y;
        for (int I = 0; I < numXGridLinesBtw; I++) {
            int X = startCoords.x + (I * signX) + boolX;
            int Y = (int)(slopeM * X + interceptB); // y = m * x + b (where b is initial z position)

            int YUsing = isReverse ? currentY : Y;
            
            rasterCellCoords.Add(new int2(currentX - (1 - boolX), YUsing));
            currentX = X;  currentY = Y;
        }
        int YUsing2 = isReverse ? currentY : endCoords.y;
        rasterCellCoords.Add(new int2(currentX - (1 - boolX), YUsing2));
    }

    // public static NativeList<int2> RasterRay(RayInput seg, float2 bottomLeftCorner, float cellSize) { return RasterSeg(seg.start.xz, seg.end.xz, bottomLeftCorner, cellSize); }
    public static NativeList<int2> RasterSeg(float2 segStart, float2 segEnd, float2 bottomLeftCorner, float cellSize) {
        NativeList<int2> rasterCellCoords = new NativeList<int2>((int)(math.length(segEnd - segStart)/cellSize), Allocator.Temp); // Resize NativeList is more expensive
        RasterSeg(segStart, segEnd, ref rasterCellCoords, bottomLeftCorner, cellSize);
        return rasterCellCoords;
    }

    public static void RasterSeg(float2 segStart, float2 segEnd, ref NativeList<int2> rasterCellCoords, float2 bottomLeftCorner, float cellSize)
    {
        Debug.Assert(!IsEpsEqual(segStart, segEnd, 0.001f), "start and end cannot be the same point");
        Debug.Assert(math.all(bottomLeftCorner <= segStart) && math.all(bottomLeftCorner <= segEnd), $"bottomLeftCorner must be LEQ start and end (X and Y),  bottomLeftCorner: {bottomLeftCorner},  segStart: {segStart},  segEnd: {segEnd}");
        float2 segVector = segEnd - segStart;
        float2 startRelativeToBottomLeftPos = (segStart - bottomLeftCorner) / cellSize;

        int2 startCoords = WorldToCellCoords(segStart, bottomLeftCorner, cellSize);
        int2 endCoords = WorldToCellCoords(segEnd, bottomLeftCorner, cellSize);

        int signY = (segVector.y > 0) ? 1 : -1;
        int signX = (segVector.x > 0) ? 1 : -1;
        float slopeM = segVector.y / segVector.x;
        if (segVector.x == 0f) { slopeM = 100000f; }
        float interceptB = (-slopeM * startRelativeToBottomLeftPos.x) + startRelativeToBottomLeftPos.y;

        int numXGridLinesBtw = math.abs(endCoords.x - startCoords.x);
        int boolX = (signX > 0) ? 1 : 0; // For offsetting x axis values when ray is negative x

        int currentX = startCoords.x + (1 - boolX);  int currentY = startCoords.y;
        for (int I = 0; I < numXGridLinesBtw; I++) {
            int X = startCoords.x + (I * signX) + boolX;
            int Y = (int)(slopeM * X + interceptB); // y = m * x + b (where b is initial z position)
            
            for (int Y0 = currentY; Y0 != Y + signY; Y0 += signY)
                rasterCellCoords.Add(new int2(currentX - (1 - boolX), Y0));

            currentX = X;  currentY = Y;
        }
        for (int Y0 = currentY; Y0 != endCoords.y + signY; Y0 += signY)
            rasterCellCoords.Add(new int2(currentX - (1 - boolX), Y0));
    }

    // TODO: Call RasterRect(int2 bottomLeftCell, int2 topRightCell)
    // public static NativeList<int2> RasterRect(float3 bottomLeftPosition, float3 topRightPosition, float2 bottomLeftCorner, float cellSize) {}
    public static NativeArray<int2> RasterRect(int2 bottomLeftCell, int2 topRightCell) {
        IntRect cellsRect = new IntRect(bottomLeftCell, topRightCell);
        NativeArray<int2> rasterCellCoords = new NativeArray<int2>(cellsRect.Area(), Allocator.Temp);

        int num = 0;
        for (int X = bottomLeftCell.x; X <= topRightCell.x; X++) {
            for (int Y = bottomLeftCell.y; Y <= topRightCell.y; Y++) {
                rasterCellCoords[num++] = new int2(X, Y);
            }
        }
        return rasterCellCoords;
    }


    // // https://en.wikipedia.org/wiki/Hungarian_algorithm#Matrix_interpretation
    // [MI(AggressiveInlining)] private static bool ckmin(ref float a, float b) { if (b < a) { a = b; return true; } else { return false; } }

    // public static NativeArray<int> MinCostAssignment(in Native2DArray<float> costs) { // Hungarian
    //     int J = costs.LengthX, W = costs.LengthY;
    //     Debug.Assert(J <= W, "Definitely a correct Assertion");
    //     // job[w] = job assigned to w-th worker, or -1 if no job assigned
    //     // note: a W-th worker was added for convenience
    //     NativeArray<int> job = new (W + 1, Allocator.Temp);
    //     job.Fill(-1);
        
    //     NativeArray<float> ys = new (J, Allocator.Temp); // potentials
    //     NativeArray<float> yt = new (W + 1, Allocator.Temp); // -yt[W] will equal the sum of all deltas
    //     NativeList<float> answers = new (J, Allocator.Temp);

    //     NativeArray<float> min_to = new (W + 1, Allocator.Temp); //min_to.Fill(float.MaxValue);
    //     NativeArray<int> prv = new (W + 1, Allocator.Temp); //prv.Fill(-1); // previous worker on alternating path
    //     NativeArray<bool> in_Z = new (W + 1, Allocator.Temp); // whether worker is in Z

    //     for (int j_cur = 0; j_cur < J; ++j_cur) {  // assign j_cur-th job
    //         int w_cur = W;
    //         job[w_cur] = j_cur;
            
    //         min_to.Fill(float.MaxValue); // min reduced cost over edges from Z to worker w
    //         prv.Fill(-1);
    //         in_Z.Clear();

    //         while (job[w_cur] != -1) { // runs at most j_cur + 1 times
    //             in_Z[w_cur] = true;
    //             int j = job[w_cur];
    //             float delta = float.MaxValue;
    //             int w_next = default;
    //             for (int w = 0; w < W; ++w) {
    //                 if (!in_Z[w]) {                    // TODO: [j][w] should work too! It doesn't
    //                     if (ckmin(ref min_to.ElementAt(w), costs[j, w] - ys[j] - yt[w])) {
    //                         prv[w] = w_cur;
    //                     }
    //                     if (ckmin(ref delta, min_to[w])) {
    //                         w_next = w;
    //                     }
    //                 }
    //             }
    //             // delta will always be non-negative, except possibly during the first time
    //             // this loop runs if any entries of C[j_cur] are negative
    //             for (int w = 0; w <= W; ++w) {
    //                 if (in_Z[w]) {
    //                     ys[job[w]] += delta;
    //                     yt[w] -= delta;
    //                 } else {
    //                     min_to[w] -= delta;
    //                 }
    //             }
    //             w_cur = w_next;
    //         }
    //         // update assignments along alternating path
    //         for (int w; w_cur != W; w_cur = w) {
    //             w = prv[w_cur];
    //             job[w_cur] = job[w];
    //         }
    //         answers.AddNoResize(-yt[W]);
    //     }
    //     return job;
    // }

    // /**
    // * Sanity check: https://en.wikipedia.org/wiki/Hungarian_algorithm#Example
    // * First job (5):
    // *   clean bathroom: Bob -> 5
    // * First + second jobs (9):
    // *   clean bathroom: Bob -> 5
    // *   sweep floors: Alice -> 4
    // * First + second + third jobs (15):
    // *   clean bathroom: Alice -> 8
    // *   sweep floors: Dora -> 4
    // *   wash windows: Bob -> 3
    // */
    // public static void SimpleHungarianTest() {
    //     Native2DArray<float> costsTest = new Native2DArray<float>(3, 3, Allocator.Temp); // { [0] = 8, 5, 9, 4, 2, 4, 7, 3, 8 };
    //     costsTest[0, 0] = 8; costsTest[0, 1] = 5; costsTest[0, 2] = 9;
    //     costsTest[1, 0] = 4; costsTest[1, 1] = 2; costsTest[1, 2] = 4;
    //     costsTest[2, 0] = 7; costsTest[2, 1] = 3; costsTest[2, 2] = 8;
    //     // The final 2 is added by the algorithm
    //     NativeArray<int> expected = new NativeList<int>(3, Allocator.Temp) {0, 2, 1, 2}.AsArray(); // answersResult = {5, 9, 15}
    //     NativeArray<int> result = MinCostAssignment(costsTest);
    //     // Debug.Log("Hungarian: " + result[0] + ", " + result[1] + ", " + result[2] + ", " + result[3]);
    //     // Debug.Log("expected: " + expected[0] + ", " + expected[1] + ", " + expected[2]);
    //     Debug.Assert(MinCostAssignment(costsTest).ArraysEqual(expected), "Hungarian is wrong");
    //     Debug.Log("Sanity check finished");
    // }
}
}
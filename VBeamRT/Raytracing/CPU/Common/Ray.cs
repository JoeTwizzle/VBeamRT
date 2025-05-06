using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT.Raytracing.CPU.Common;

struct Ray
{
    public Vec3 Origin;
    public Vec3 Direction;

    public Ray(Vec3 origin, Vec3 direction)
    {
        Origin = origin;
        Direction = direction;
    }

    public static Ray CreateCameraRay(Vec2 rayCoords, Matrix4x4 viewMatrix, Matrix4x4 inverseProjectionMatrix)
    {
        // Transform the camera origin to world space
        //Vector3 origin = mul(_CameraProperites._CameraToWorld, float4(0.0, 0.0, 0.0, 1.0)).xyz;
        Vec3 origin = Vec3.Transform(Vec3.Zero, viewMatrix);
        // Invert the perspective projection of the view-space position
        Vec3 direction = Vec4.Transform(new Vec4(rayCoords, 0f, 1f), inverseProjectionMatrix).AsVector128().AsVector3();

        // Transform the direction from camera to world space
        direction = Vec3.Normalize(Vec4.Transform(new Vec4(direction, 0.0f), viewMatrix).AsVector128().AsVector3());
        return new Ray(origin, direction);
    }

    public readonly Vec3 GetHitPoint(float hitDist)
    {
        return Origin + hitDist * Direction;
    }
}

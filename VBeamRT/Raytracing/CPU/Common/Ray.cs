using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT.Raytracing.CPU.Common;

public struct Ray
{
    public Vec3 Origin;
    public Vec3 Direction;

    public Ray(Vec3 origin, Vec3 direction)
    {
        Origin = origin;
        Direction = direction;
    }

    public static Ray CreateCameraRay(Vec2 rayCoords, Matrix4x4 inverseViewMatrix, Matrix4x4 inverseProjectionMatrix)
    {
        // ndcUV is in range [0,1]; convert to NDC in [-1,1]

        // Build clip space position on near plane (z = -1 in OpenGL, or z = 1 in DirectX)
        Vec4 clipPos = new Vec4(rayCoords, -1f, 1f);

        // Transform into view (camera) space
        Vec4 viewPosH = Vec4.Transform(clipPos, inverseProjectionMatrix);
        viewPosH /= viewPosH.W; // Perspective divide

        // Interpret as a direction from the camera origin
        Vec4 viewDirH = new Vec4(viewPosH.X, viewPosH.Y, viewPosH.Z, 0f);

        // Transform origin and direction into world space
        Vec3 origin = Vec3.Transform(Vec3.Zero, inverseViewMatrix);
        Vec3 direction = Vec3.Normalize(Vec4.Transform(viewDirH, inverseViewMatrix).AsVector128().AsVector3());

        return new Ray(origin, direction);

        return new Ray(origin, direction);
    }

    public readonly Vec3 GetHitPoint(float hitDist)
    {
        return Origin + hitDist * Direction;
    }
}

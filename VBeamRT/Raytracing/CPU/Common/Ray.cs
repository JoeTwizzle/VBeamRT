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
    public float TMin;
    public Vec3 Direction;
    public float TMax;

    public Ray(Vec3 origin, Vec3 direction)
    {
        Origin = origin;
        Direction = direction;
        TMin = 0;
        TMax = float.MaxValue;
    }

    public Ray(Vec3 origin, float tMin, Vec3 direction, float tMax)
    {
        Origin = origin;
        TMin = tMin;
        Direction = direction;
        TMax = tMax;
    }

    public static Ray CreateCameraRay(Vec2 rayCoords, Matrix4x4 inverseViewMatrix, Matrix4x4 inverseProjectionMatrix)
    {
        // Convert to NDC [-1, 1] (flip Y for standard graphics pipeline)
        float x = rayCoords.X ;
        float y = rayCoords.Y ; // Flip Y-axis

        // Create clip position with robust near-plane handling
        Vec4 clipPos = new Vec4(x, y, -1f, 1f); 

        // Transform to view space
        Vec4 viewPosH = Vec4.Transform(clipPos, inverseProjectionMatrix);
        viewPosH /= viewPosH.W;

        // Create direction vector
        Vec4 viewDirH = new Vec4(viewPosH.X, viewPosH.Y, viewPosH.Z, 0f);

        // Transform to world space
        Vec3 origin = Vec3.Transform(Vec3.Zero, inverseViewMatrix);
        Vec3 direction = Vec3.Normalize(Vec4.Transform(viewDirH, inverseViewMatrix).AsVector3());

        // Apply robust direction handling
        direction = RobustDirection(direction);

        return new Ray(origin, direction);
    }

    private static Vec3 RobustDirection(Vec3 dir)
    {
        const float threshold = 1e-6f;
        const float safeValue = 1e-5f; // Small but meaningful value

        // Maintain original magnitude while ensuring no zero components
        float origMagnitude = dir.Length();

        // Handle near-zero components with sign preservation
        dir.X = MathF.Abs(dir.X) < threshold ?
            MathF.CopySign(safeValue, dir.X) : dir.X;

        dir.Y = MathF.Abs(dir.Y) < threshold ?
            MathF.CopySign(safeValue, dir.Y) : dir.Y;

        dir.Z = MathF.Abs(dir.Z) < threshold ?
            MathF.CopySign(safeValue, dir.Z) : dir.Z;

        // Restore original magnitude to preserve ray energy
        return Vec3.Normalize(dir) * origMagnitude;
    }

    public readonly Vec3 GetHitPoint(float hitDist)
    {
        return Origin + hitDist * Direction;
    }
}

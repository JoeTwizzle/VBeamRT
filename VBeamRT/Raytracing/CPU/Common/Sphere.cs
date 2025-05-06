using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT.Raytracing.CPU.Common;

struct Sphere : ISceneItem
{
    public Vec3 Position;
    public float Radius;

    public Sphere(Vec3 position, float radius)
    {
        Position = position;
        Radius = radius;
    }

    public readonly HitInfo HitTest(Ray ray)
    {
        float B = 2 * Vec3.Dot(ray.Direction * (ray.Origin - Position), Vec3.One);

        float C = Vec3.Dot((ray.Origin - Position), (ray.Origin - Position)) - Radius * Radius;

        float D = B * B - 4 * C;

        if (D < 0) return new HitInfo();

        float dRes = float.Sqrt(D);

        float t0 = ((-B) - dRes) / 2f;
        float t1 = ((-B) + dRes) / 2f;

        if (t0 <= 0 && t1 <= 0) return new HitInfo();

        return new HitInfo(float.Min(t0, t1), float.Max(t0, t1));
    }

    public readonly Vec3 GetSurfaceNormal(Vec3 hitPoint)
    {
        return (hitPoint - Position) / Radius;
    }
}

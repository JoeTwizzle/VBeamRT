using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT.Raytracing.CPU.Common;

struct Triangle : ISceneItem
{
    public Vec3 Point0;
    public Vec3 Edge1;
    public Vec3 Edge2;

    //From: https://iquilezles.org/articles/intersectors/
    public readonly HitInfo HitTest(Ray ray)
    {
        Vec3 triToRayOrigin = ray.Origin - Point0;
        Vec3 normal = Vec3.Cross(Edge1, Edge2);
        Vec3 q = Vec3.Cross(triToRayOrigin, ray.Direction);
        float d = 1.0f / Vec3.Dot(ray.Direction, normal);
        float u = d * Vec3.Dot(-q, Edge2);
        float v = d * Vec3.Dot(q, Edge1);
        float t = d * Vec3.Dot(-normal, triToRayOrigin);
        if (u < 0.0 || v < 0.0 || (u + v) > 1.0) t = float.PositiveInfinity;
        //return new Vec3( t, u, v ); // barycentric coordinates
        return new HitInfo(t, t);
    }

    public readonly Vec3 GetSurfaceNormal()
    {
        return Vec3.Cross(Edge1, Edge2);
    }
}

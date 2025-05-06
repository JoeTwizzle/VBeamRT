using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT.Raytracing.CPU.Common;

struct PointLight
{
    public Vec3 Position;
    public Vec3 Color;

    public PointLight(Vec3 position, Vec3 color)
    {
        Position = position;
        Color = color;
    }
}

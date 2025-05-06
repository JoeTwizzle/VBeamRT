using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VBeamRT.Raytracing.CPU.Common;

namespace VBeamRT.Raytracing.CPU.Whitted.Simple;

struct Material
{
    public Vec3 Color;
    public float Roughness;

    public Material(Vec3 color, float roughness)
    {
        Color = color;
        Roughness = roughness;
    }
}

sealed class Scene
{
    public List<Material> Materials;
    public List<Sphere> Spheres;
    public List<PointLight> PointLights;
    public Scene()
    {
        Materials = new List<Material>();
        Spheres = new List<Sphere>();
        PointLights = new List<PointLight>();
    }
}

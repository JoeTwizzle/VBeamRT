global using Vec2 = System.Numerics.Vector2;
global using Vec3 = System.Numerics.Vector3;
global using Vec4 = System.Numerics.Vector4;
global using Vector2 = OpenTK.Mathematics.Vector2;
global using Vector3 = OpenTK.Mathematics.Vector3;
global using Vector4 = OpenTK.Mathematics.Vector4;
using VBeamRT;
using OpenTK.Platform;
using System.Diagnostics;

namespace VBeamRT;

internal static class Program
{
    static void Main(string[] args)
    {
        using var game = new Game();
    }
}
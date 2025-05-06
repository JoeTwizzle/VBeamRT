using OpenTK.Graphics.Glx;
using OpenTK.Mathematics;
using OpenTK.Platform;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;
using VBeamRT.Raytracing.CPU.Common;
using VKGraphics;

namespace VBeamRT.Raytracing.CPU.Whitted.Simple;

class SimpleRaytracer : IRenderer
{
    const float ToRadians = (MathF.PI / 180f);
    const float ToDegrees = (180f / MathF.PI);
    const int MaxDepth = 3;
    readonly Game _game;
    readonly Scene _scene;
    GraphicsDevice _gd;
    ResourceFactory _rf;
    Swapchain _swapchain;
    Texture _mainTex;
    ResourceLayout _blitLayout;
    ResourceSet _blitSet;
    CommandList _cl;
    Pipeline _blitPipeline;
    Vector4h[] _imageBuffer;

#pragma warning disable CS8618 // Non-nullable field must contain a non-null value when exiting constructor. Consider adding the 'required' modifier or declaring as nullable.
    public SimpleRaytracer(Game game)
#pragma warning restore CS8618 // Non-nullable field must contain a non-null value when exiting constructor. Consider adding the 'required' modifier or declaring as nullable.
    {
        _game = game;
        _scene = new();
    }

    public void Initalize()
    {
        SetupRenderer();
        SetupBlitPass();
        SetupScene();
    }

    void SetupScene()
    {
        _scene.Spheres.Add(new Sphere(Vec3.UnitZ * 3, 0.5f));
        _scene.Materials.Add(new Material(new(1, 0, 0), 0));

        _scene.Spheres.Add(new Sphere(new(5, 1, 7), 1.75f));
        _scene.Materials.Add(new Material(new(0.4f, 0, 0.7f), 0.5f));

        _scene.Spheres.Add(new Sphere(new(0, -1001f, 0), 1000f));
        _scene.Materials.Add(new Material(new(0.06f, 0.83f, 0.25f), 0.5f));

        for (int i = 0; i < 32; i++)
        {
            _scene.Spheres.Add(new Sphere(new(
                (Random.Shared.NextSingle() - 0.5f) * 20,
                (Random.Shared.NextSingle() - 0.5f) * 20,
               10 + Random.Shared.NextSingle() * 50), Random.Shared.NextSingle() * 3));
            _scene.Materials.Add(new Material(new(Random.Shared.NextSingle(), Random.Shared.NextSingle(), Random.Shared.NextSingle()), Random.Shared.NextSingle()));
        }


        _scene.PointLights.Add(new PointLight(new(0, 10, 0), new(5, 3, 3)));
        _scene.PointLights.Add(new PointLight(new(10, 10, 0), new(3, 3, 4)));
    }

    void SetupRenderer()
    {
        _gd = GraphicsDevice.CreateVulkan(new GraphicsDeviceOptions(
            true, null, false, ResourceBindingModel.Improved, true, false));
        _rf = _gd.ResourceFactory;
        _swapchain = _rf.CreateSwapchain(new SwapchainDescription(
            _game.MainWindowInfo.Handle, (uint)800, 600, null, false));
    }

    void SetupBlitPass()
    {
        //Blit Pass
        var vertShaderResult = SpirvCompiler.GetSpirvBytes("Shaders/FullscreenTri/fsTriVert.vert");
        var vertShader = _rf.CreateShader(new ShaderDescription(ShaderStages.Vertex, vertShaderResult, "main"));
        var fragResult = SpirvCompiler.GetSpirvBytes("Shaders/FullscreenTri/fsTriFrag.frag");
        var fragShader = _rf.CreateShader(new ShaderDescription(ShaderStages.Fragment, fragResult, "main"));

        //Display layout
        _blitLayout = _rf.CreateResourceLayout(new ResourceLayoutDescription(
               new ResourceLayoutElementDescription("_MainSampler", ResourceKind.Sampler, ShaderStages.Fragment),
               new ResourceLayoutElementDescription("_MainTexture", ResourceKind.TextureReadOnly, ShaderStages.Fragment))
         );

        var fsTriShaderSet = new ShaderSetDescription(null, [vertShader, fragShader]);
        _blitPipeline = _rf.CreateGraphicsPipeline(new GraphicsPipelineDescription(BlendStateDescription.SingleOverrideBlend,
            DepthStencilStateDescription.Disabled, RasterizerStateDescription.CullNone,
              PrimitiveTopology.TriangleList, fsTriShaderSet, _blitLayout, _swapchain.Framebuffer.OutputDescription));

        _cl = _rf.CreateCommandList();
    }

    void ResizeGPU(int resX, int resY)
    {
        _mainTex?.Dispose();
        _mainTex = _rf.CreateTexture(new TextureDescription((uint)resX, (uint)resY,
            1, 1, 1, PixelFormat.R16G16B16A16Float, TextureUsage.Sampled | TextureUsage.Storage, TextureType.Texture2D));
        _blitSet?.Dispose();
        _blitSet = _rf.CreateResourceSet(new ResourceSetDescription(_blitLayout, _gd.LinearSampler, _mainTex));
        _swapchain.Resize((uint)resX, (uint)resY);
    }

    void ResizeCPU(int resX, int resY)
    {
        _imageBuffer = new Vector4h[resX * resY];
    }

    public void Update()
    {
        Toolkit.Window.GetFramebufferSize(_game.MainWindowInfo.Handle, out var framebufferSize);
        if (_mainTex == null || _mainTex.Width != framebufferSize.X || _mainTex.Height != framebufferSize.Y)
        {
            ResizeCPU(framebufferSize.X, framebufferSize.Y);
            ResizeGPU(framebufferSize.X, framebufferSize.Y);
        }
        VarySceneParamaters();
        RenderScene(framebufferSize.X, framebufferSize.Y);
        BlitImage(framebufferSize.X, framebufferSize.Y);
    }


    float totalTime = 0f;
    void VarySceneParamaters()
    {
        var mat = _scene.Materials[0];
        mat.Roughness = float.Sin(totalTime) * 0.5f + 0.5f;
        _scene.Materials[0] = mat;

        totalTime += _game.DeltaTime;
    }

    private void BlitImage(int xPixels, int yPixels)
    {
        _gd.UpdateTexture(_mainTex, _imageBuffer, 0, 0, 0, (uint)xPixels, (uint)yPixels, 1, 0, 0);

        _cl.Begin();
        _cl.SetPipeline(_blitPipeline);
        _cl.SetGraphicsResourceSet(0, _blitSet);
        _cl.SetFramebuffer(_swapchain.Framebuffer);
        _cl.SetFullViewport(0);
        _cl.Draw(3);
        _cl.End();
        _gd.SubmitCommands(_cl);
        _gd.SwapBuffers(_swapchain);
    }

    static Vec3 Aces(Vec3 v)
    {
        v *= 0.6f;
        float a = 2.51f;
        float b = 0.03f;
        float c = 2.43f;
        float d = 0.59f;
        float e = 0.14f;
        return Vec3.Clamp((v * (a * v + new Vec3(b))) / (v * (c * v + new Vec3(d)) + new Vec3(e)), Vec3.Zero, Vec3.One);
    }

    void RenderScene(int xPixels, int yPixels)
    {
        var viewMatrix = Matrix4x4.CreateLookTo(Vec3.Zero, Vec3.UnitZ, Vec3.UnitY);
        var projectionMatrix = Matrix4x4.CreatePerspectiveFieldOfView(
            60 * ToRadians, xPixels / (float)yPixels, 0.01f, 1000f);

        Matrix4x4.Invert(projectionMatrix, out var invereseProjectionMatrix);
        Parallel.For(0, yPixels, y =>
        {
            for (int x = 0; x < xPixels; x++)
            {
                var uvCoords = new Vec2(x / (float)xPixels, y / (float)yPixels);
                var rayCoords = uvCoords * 2f - Vec2.One;
                var ray = Ray.CreateCameraRay(rayCoords, viewMatrix, invereseProjectionMatrix);
                var color = TraceRay(ray, 0);

                Vec3 mapped = ToSrgb(Aces(color));

                _imageBuffer[x + y * xPixels] = (Vector4h)Vector4.Clamp(new Vector4(mapped.X, mapped.Y, mapped.Z, 1), Vector4.Zero, Vector4.One);
            }
        });
    }

    static Vec3 ToLinear(Vec3 src)
    {
        src.X = float.Pow(src.X, 2.2f);
        src.Y = float.Pow(src.Y, 2.2f);
        src.Z = float.Pow(src.Z, 2.2f);
        return src;
    }

    static Vec3 ToSrgb(Vec3 src)
    {
        src.X = float.Pow(src.X, 1 / 2.2f);
        src.Y = float.Pow(src.Y, 1 / 2.2f);
        src.Z = float.Pow(src.Z, 1 / 2.2f);
        return src;
    }

    Vec3 TraceRay(Ray ray, int depth)
    {
        if (depth == MaxDepth) return new Vec3();

        int closestHitIndex = -1;
        HitInfo closestHitInfo = new();
        for (int i = 0; i < _scene.Spheres.Count; i++)
        {
            var hitInfo = _scene.Spheres[i].HitTest(ray);

            if (hitInfo.HitDistNear < closestHitInfo.HitDistNear)
            {
                closestHitIndex = i;
                closestHitInfo = hitInfo;
            }
        }

        if (closestHitIndex == -1) return ToLinear(new Vec3(0.26f, 0.53f, 0.96f)) * 0.3f;

        Vec3 albedo = ToLinear(_scene.Materials[closestHitIndex].Color);
        Vec3 localColor = new(0.0f);
        var hitPos = ray.GetHitPoint(closestHitInfo.HitDistNear);
        var normal = _scene.Spheres[closestHitIndex].GetSurfaceNormal(hitPos);
        hitPos += normal * 0.0001f;
        for (int i = 0; i < _scene.PointLights.Count; i++)
        {
            Vec3 lightDir = Vec3.Normalize(_scene.PointLights[i].Position - hitPos);
            // Diffuse
            float diff = MathF.Max(Vec3.Dot(normal, lightDir), 0f);
            Vec3 diffuse = diff * albedo * ToLinear(_scene.PointLights[i].Color);

            // Specular
            Vec3 reflectDir = Vec3.Reflect(-lightDir, normal);
            float spec = float.Pow(float.Max(Vec3.Dot(-ray.Direction, reflectDir), 0f),
                    float.Lerp(4, 32, 1 - _scene.Materials[closestHitIndex].Roughness));
            Vec3 specular = spec * ToLinear(_scene.PointLights[i].Color);
            float shadowFac = 1f;
            for (int j = 0; j < _scene.Spheres.Count; j++)
            {
                var hitInfo = _scene.Spheres[j].HitTest(new Ray(hitPos, lightDir));

                if (hitInfo.HitDistNear != float.PositiveInfinity)
                {
                    shadowFac = 0;
                    break;
                }
            }

            localColor += (diffuse + specular) * shadowFac;
        }

        var reflectedColor = TraceRay(new Ray(hitPos, Vec3.Reflect(ray.Direction, normal)), depth + 1);

        return localColor + reflectedColor * float.Lerp(0.05f, 0.9f, 1 - _scene.Materials[closestHitIndex].Roughness);
    }

    public void Dispose()
    {
        _gd.WaitForIdle();
        _blitSet.Dispose();
        _blitPipeline.Dispose();
        _blitLayout.Dispose();
        _mainTex.Dispose();
        _cl.Dispose();
        _swapchain.Dispose();
        _gd.Dispose();
    }
}

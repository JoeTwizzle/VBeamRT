using OpenTK.Mathematics;
using OpenTK.Platform;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using VBeamRT.Raytracing.CPU.Common;
using VKGraphics;
using static OpenTK.Platform.Native.macOS.MacOSCursorComponent;

namespace VBeamRT.Raytracing.CPU.PathTracing.BVH;

sealed partial class BVHRaytracer : IRenderer
{

    readonly Game _game;
    BVH BVH;
    GraphicsDevice _gd;
    ResourceFactory _rf;
    Swapchain _swapchain;
    Texture _mainTex;
    ResourceLayout _blitLayout;
    ResourceSet _blitSet;
    CommandList _cl;
    Pipeline _blitPipeline;
    Vec3[] _linearAccumulationBuffer;
    Vector4[] _imageBuffer;
    GltfScene scene;
    readonly Triangle[] triangles;
    readonly Vertex[] vertices;
    public Vector2i Res;
#pragma warning disable CS8618 // Non-nullable field must contain a non-null value when exiting constructor. Consider adding the 'required' modifier or declaring as nullable.
    public BVHRaytracer(Game game)
#pragma warning restore CS8618 // Non-nullable field must contain a non-null value when exiting constructor. Consider adding the 'required' modifier or declaring as nullable.
    {
        _game = game;
        scene = GltfLoader.Load("Models/alfa_romeo_stradale_1967.glb");
        //scene = GltfLoader.Load("Models/Beta map split.glb");
        vertices = [.. scene.Vertices];
        triangles = [.. scene.Triangles];
        BVH = new BVH(vertices, triangles);
    }

    public void Initalize()
    {
        SetupRenderer();
        SetupBlitPass();
    }

    void SetupRenderer()
    {
        _gd = GraphicsDevice.CreateVulkan(new GraphicsDeviceOptions(
            true, null, false, ResourceBindingModel.Improved, true, false));
        _rf = _gd.ResourceFactory;
        _swapchain = _rf.CreateSwapchain(new SwapchainDescription(
            _game.MainWindowInfo.Handle, 800, 600, null, false));
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

    void ResizeCPU(int resX, int resY)
    {
        Console.WriteLine($"Resized to: {resX} by {resY}");
        _linearAccumulationBuffer = new Vec3[resX * resY];
        _imageBuffer = new Vector4[resX * resY];
        Res = new(resX, resY);
        _frameCount = 0;
        _mainTex?.Dispose();
        _mainTex = _rf.CreateTexture(new TextureDescription((uint)Res.X, (uint)Res.Y,
            1, 1, 1, PixelFormat.R32G32B32A32Float, TextureUsage.Sampled | TextureUsage.Storage, TextureType.Texture2D));
        _blitSet?.Dispose();
        _blitSet = _rf.CreateResourceSet(new ResourceSetDescription(_blitLayout, _gd.LinearSampler, _mainTex));
    }
    public void Update()
    {
   
        if (_game.Input.KeyPressed(Scancode.F11))
        {
            if (Toolkit.Window.GetMode(_game.MainWindowInfo.Handle) == WindowMode.WindowedFullscreen)
            {
                Toolkit.Window.SetMode(_game.MainWindowInfo.Handle, WindowMode.Normal);
            }
            else
            {
                Toolkit.Window.SetMode(_game.MainWindowInfo.Handle, WindowMode.WindowedFullscreen);
            }
        }

        Toolkit.Window.GetFramebufferSize(_game.MainWindowInfo.Handle, out var framebufferSize);

        if (_linearAccumulationBuffer == null
            || _game.Input.KeyPressed(Scancode.R)
            && framebufferSize.X * framebufferSize.Y != _linearAccumulationBuffer.Length)
        {
            ResizeCPU(framebufferSize.X, framebufferSize.Y);
        }
        if ((_mainTex == null || _mainTex.Width != framebufferSize.X || _mainTex.Height != framebufferSize.Y)
            && Toolkit.Window.GetMode(_game.MainWindowInfo.Handle) != WindowMode.Hidden)
        {
            _swapchain.Resize((uint)framebufferSize.X, (uint)framebufferSize.Y);
        }
        RenderScene(Res.X, Res.Y);
        BlitImage(Res.X, Res.Y);
        if (_game.Input.KeyPressed(Scancode.P))
        {
            Image<RgbaVector> img = new(framebufferSize.X, framebufferSize.Y);
            img.ProcessPixelRows(a =>
            {
                for (int y = 0; y < framebufferSize.Y; y++)
                {
                    var destSpan = a.GetRowSpan(framebufferSize.Y - (y + 1));
                    MemoryMarshal.Cast<Vector4, RgbaVector>(_imageBuffer.AsSpan(framebufferSize.X * y, framebufferSize.X)).CopyTo(destSpan);
                }

            });
            img.SaveAsPng("Output.png");
        }
    }


    private void BlitImage(int xPixels, int yPixels)
    {
        Toolkit.Window.GetFramebufferSize(_game.MainWindowInfo.Handle, out var fb);

        if (fb.X != _swapchain.Framebuffer.Width || _swapchain.Framebuffer.Height != fb.Y) { return; }
        Parallel.For(0, _imageBuffer.Length, i =>
        {
            var colorD = _linearAccumulationBuffer[i];
            var color = ToSrgb(Aces(new Vec3((float)colorD.X, (float)colorD.Y, (float)colorD.Z)));
            _imageBuffer[i] = new Vector4(color.X, color.Y, color.Z, 1f);
        });
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

    int _frameCount = 0;
    void RenderScene(int xPixels, int yPixels)
    {

        //var viewMatrix = Matrix4x4.CreateLookTo(Vec3.UnitY * 115 + Vec3.UnitX * 64 + Vec3.UnitZ * -65, Vec3.UnitX + Vec3.UnitZ * 0.5f, Vec3.UnitY);
        var viewMatrix = Matrix4x4.CreateLookTo(Vec3.UnitY * 8 + Vec3.UnitZ * 35 + Vec3.UnitZ * 1, -Vec3.UnitZ, Vec3.UnitY);


        var projectionMatrix = Matrix4x4.CreatePerspectiveFieldOfView(
            40 * ToRadians, xPixels / (float)yPixels, 0.01f, 1000f);
        Matrix4x4.Invert(projectionMatrix, out var inverseProjectionMatrix);
        Matrix4x4.Invert(viewMatrix, out var inverseViewMatrix);
        _frameCount++;
        var (dx, rx) = int.DivRem(xPixels, 64);
        int tilesX = dx + (rx != 0 ? 1 : 0);
        var (dy, ry) = int.DivRem(yPixels, 64);
        int tilesY = dy + (ry != 0 ? 1 : 0);
        Parallel.For(0, tilesX * tilesY, id =>
        //for (int id = 0; id < tilesX * tilesY; id++)
        {
            int tileX = id % tilesX;
            int tileY = id / tilesX;

            for (int y = tileY * 64; y < (tileY + 1) * 64; y++)
            {
                if (y >= yPixels) { continue; }
                for (int x = tileX * 64; x < (tileX + 1) * 64; x++)
                {
                    if (x >= xPixels) { continue; }
                    var uvCoords = new Vec2(x / (float)xPixels, y / (float)yPixels);

                    int idx = y * xPixels + x;
                    Vec3 current =
                     (_frameCount % 4 == 0) ?
                    TraceRayWithGradient(x, y, xPixels, yPixels, inverseViewMatrix, inverseProjectionMatrix) :
                    TraceRay(GenerateRay(uvCoords, inverseViewMatrix, inverseProjectionMatrix));
                    Vec3 prev = _linearAccumulationBuffer[x + y * xPixels];

                    _linearAccumulationBuffer[x + y * xPixels] = Vec3.Lerp(prev, current, 1f / _frameCount);
                    //_imageBuffer[x + y * xPixels] = Vector4.Clamp(new Vector4(color.X, color.Y, color.Z, 1), Vector4.Zero, Vector4.One);
                }
            }
        });
        //}
        Console.WriteLine("Samples: " + _frameCount);
    }
    Vec3 TraceRayWithGradient(int x, int y, int _width, int _height, Matrix4x4 inverseViewMatrix, Matrix4x4 inverseProjectionMatrix)
    {
        const float PixelOffset = 0.7f;  // Fraction of pixel to offset
        const float kGradientScale = 0.2f;  // Reduced gradient strength
        Vec3 kGradientClamp = new(5.0f);  // Maximum gradient magnitude
        const float kMinLuminance = 0.01f;  // Increased minimum luminance threshold

        // Create primary ray for center of pixel
        Vec2 centerUV = new Vec2((x + 0.5f) / _width, (y + 0.5f) / _height);
        Ray centerRay = GenerateRay(centerUV, inverseViewMatrix, inverseProjectionMatrix);
        Vec3 centerColor = TraceRay(centerRay);

        // Skip gradient computation for dark pixels
        if (Luminance(centerColor) < kMinLuminance)
            return centerColor;

        // Compute offsets for gradient calculation - use consistent jitter
        float jitterX = (x % 2 == 0) ? PixelOffset : -PixelOffset;
        float jitterY = (y % 2 == 0) ? PixelOffset : -PixelOffset;

        Vec2 rightUV = new Vec2((x + 0.5f + jitterX) / _width, (y + 0.5f) / _height);
        Vec2 leftUV = new Vec2((x + 0.5f - jitterX) / _width, (y + 0.5f) / _height);
        Vec2 upUV = new Vec2((x + 0.5f) / _width, (y + 0.5f - jitterY) / _height);
        Vec2 downUV = new Vec2((x + 0.5f) / _width, (y + 0.5f + jitterY) / _height);

        // Create offset rays with consistent ray differentials
        Ray rightRay = GenerateRay(rightUV, inverseViewMatrix, inverseProjectionMatrix);
        Ray leftRay = GenerateRay(leftUV, inverseViewMatrix, inverseProjectionMatrix);
        Ray upRay = GenerateRay(upUV, inverseViewMatrix, inverseProjectionMatrix);
        Ray downRay = GenerateRay(downUV, inverseViewMatrix, inverseProjectionMatrix);

        // Trace offset rays with consistent noise patterns
        Vec3 rightColor = TraceRay(rightRay);
        Vec3 leftColor = TraceRay(leftRay);
        Vec3 upColor = TraceRay(upRay);
        Vec3 downColor = TraceRay(downRay);

        // Compute gradients using central differences with clamping
        Vec3 gradientX = (rightColor - leftColor) / (2 * MathF.Abs(jitterX));
        Vec3 gradientY = (downColor - upColor) / (2 * MathF.Abs(jitterY));

        // Clamp gradients to prevent extreme values
        gradientX = Vec3.Clamp(gradientX, -kGradientClamp, kGradientClamp);
        gradientY = Vec3.Clamp(gradientY, -kGradientClamp, kGradientClamp);

        // Compute adaptive gradient scale based on local contrast
        float centerLum = Luminance(centerColor);
        float contrast = 0.5f * (
            MathF.Abs(Luminance(rightColor) - centerLum) +
            MathF.Abs(Luminance(leftColor) - centerLum) +
            MathF.Abs(Luminance(upColor) - centerLum) +
            MathF.Abs(Luminance(downColor) - centerLum)
        );

        float adaptiveScale = kGradientScale * MathF.Min(1.0f, 1.0f / (contrast + 0.1f));

        // Apply gradient-domain adjustment with gamma-aware operation
        Vec3 adjustedColor = centerColor - adaptiveScale * (gradientX + gradientY);

        // Maintain energy conservation
        adjustedColor = Vec3.Max(adjustedColor, Vec3.Zero);

        // Blend with original color to reduce artifacts
        float blendFactor = MathF.Min(1.0f, contrast * 2.0f);
        return Vec3.Lerp(centerColor, adjustedColor, blendFactor);
    }
    static Ray GenerateRay(Vec2 uv, Matrix4x4 inverseViewMatrix, Matrix4x4 inverseProjectionMatrix)
    {
        var rayCoords = uv * 2f - Vec2.One;
        var ray = Ray.CreateCameraRay(rayCoords, inverseViewMatrix, inverseProjectionMatrix);
        return ray;
    }


    static Vec3 Aces(Vec3 v)
    {
        v *= 0.6f;
        float a = 2.51f;
        float b = 0.03f;
        float c = 2.43f;
        float d = 0.59f;
        float e = 0.14f;
        return Vec3.Clamp(v * (a * v + new Vec3(b)) / (v * (c * v + new Vec3(d)) + new Vec3(e)), Vec3.Zero, Vec3.One);
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

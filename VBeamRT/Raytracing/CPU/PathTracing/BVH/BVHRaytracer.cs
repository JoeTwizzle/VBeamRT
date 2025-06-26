using OpenTK.Mathematics;
using OpenTK.Platform;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using VBeamRT.Raytracing.CPU.Common;
using VKGraphics;

namespace VBeamRT.Raytracing.CPU.PathTracing.BVH;

class BVHRaytracer : IRenderer
{
    const float ToRadians = MathF.PI / 180f;
    const float ToDegrees = 180f / MathF.PI;
    const int MaxDepth = 128;
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

#pragma warning disable CS8618 // Non-nullable field must contain a non-null value when exiting constructor. Consider adding the 'required' modifier or declaring as nullable.
    public BVHRaytracer(Game game)
#pragma warning restore CS8618 // Non-nullable field must contain a non-null value when exiting constructor. Consider adding the 'required' modifier or declaring as nullable.
    {
        _game = game;
        scene = GltfLoader.Load("Models/Beta map split.glb");
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

    void ResizeGPU(int resX, int resY)
    {
        _mainTex?.Dispose();
        _mainTex = _rf.CreateTexture(new TextureDescription((uint)resX, (uint)resY,
            1, 1, 1, PixelFormat.R32G32B32A32Float, TextureUsage.Sampled | TextureUsage.Storage, TextureType.Texture2D));
        _blitSet?.Dispose();
        _blitSet = _rf.CreateResourceSet(new ResourceSetDescription(_blitLayout, _gd.LinearSampler, _mainTex));
        _swapchain.Resize((uint)resX, (uint)resY);
    }

    void ResizeCPU(int resX, int resY)
    {
        Console.WriteLine($"Resized to: {resX} by {resY}");
        _linearAccumulationBuffer = new Vec3[resX * resY];
        _imageBuffer = new Vector4[resX * resY];
        _frameCount = 0;
    }

    public void Update()
    {
        Toolkit.Window.GetFramebufferSize(_game.MainWindowInfo.Handle, out var framebufferSize);
        if (_linearAccumulationBuffer == null || framebufferSize.X * framebufferSize.Y != _linearAccumulationBuffer.Length)
        {
            ResizeCPU(framebufferSize.X, framebufferSize.Y);
        }
        if (_mainTex == null || _mainTex.Width != framebufferSize.X || _mainTex.Height != framebufferSize.Y)
        {
            if (Toolkit.Window.GetMode(_game.MainWindowInfo.Handle) != WindowMode.Hidden)
            {
                ResizeGPU(framebufferSize.X, framebufferSize.Y);
            }
        }
        RenderScene(framebufferSize.X, framebufferSize.Y);
        BlitImage(framebufferSize.X, framebufferSize.Y);
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
    int _frameCount = 0;
    void RenderScene(int xPixels, int yPixels)
    {
        var viewMatrix = Matrix4x4.CreateLookTo(Vec3.UnitY * 80, Vec3.UnitX, Vec3.UnitY);


        var projectionMatrix = Matrix4x4.CreatePerspectiveFieldOfView(
            60 * ToRadians, xPixels / (float)yPixels, 0.01f, 1000f);
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
                    var rayCoords = uvCoords * 2f - Vec2.One;
                    var ray = Ray.CreateCameraRay(rayCoords, inverseViewMatrix, inverseProjectionMatrix);
                    Vec3 current = TraceRay(ray);
                    Vec3 prev = _linearAccumulationBuffer[x + y * xPixels];

                    _linearAccumulationBuffer[x + y * xPixels] = Vec3.Lerp(prev, current, 1f / _frameCount);
                    //_imageBuffer[x + y * xPixels] = Vector4.Clamp(new Vector4(color.X, color.Y, color.Z, 1), Vector4.Zero, Vector4.One);
                }
            }
        });
        //}
        Console.WriteLine("Samples: " + _frameCount);
    }
    private static readonly ThreadLocal<PCG32> rng = new(() =>
    {
        ulong threadSeed = (ulong)Environment.TickCount ^ (ulong)Environment.CurrentManagedThreadId;
        return new PCG32(threadSeed);
    });

    Vec3 TraceRay(Ray ray)
    {
        Vec3 throughput = Vec3.One;
        Vec3 radiance = Vec3.Zero;
        bool specularBounce = true; // Start as true to capture emissive surfaces
        int triTests = 0;
        int boxTests = 0;


        for (int bounce = 0; bounce < MaxDepth; bounce++)
        {
            if (!BVH.IntersectClosest(ray, out HitInfo hit))
            {
                // HDR environment mapping with physical units
                Vec3 unitDir = Vec3.Normalize(ray.Direction);
                float t = 0.5f * (unitDir.Y + 1.0f);

                // Sun disk approximation
                float sun = Math.Clamp(Vec3.Dot(unitDir, ToLinear(new Vec3(0.35f, 0.9f, 0.2f))), 0, 1);
                sun = MathF.Pow(sun, 512) * 20f;

                // Improved sky model
                Vec3 sky = Vec3.Lerp(
                    ToLinear(new Vec3(0.4f, 0.5f, 0.9f)) * 0.2f,
                    ToLinear(new Vec3(0.8f, 0.9f, 1.0f)) * 1.5f,
                    t) + new Vec3(sun, sun * 0.9f, sun * 0.6f);

                //radiance += throughput * sky;
                triTests += hit.TriTests;
                boxTests += hit.BoxTests;
                break;
            }
            triTests += hit.TriTests;
            boxTests += hit.BoxTests;

            var prim = scene.Primitives[hit.PrimIndex];
            var mat = scene.Materials[prim.MaterialIndex];
            Vec3 albedo = mat.AlbedoTextureId >= 0 ?
              scene.Images[scene.Textures[mat.AlbedoTextureId]].Sample(hit.UV).AsVector3() :
                mat.Albedo;

            // Normal mapping
            Vec3 normal = hit.Normal;
            Vec3 tangent;
            Vec3 bitangent;
            if (mat.NormalTextureId >= 0 && mat.NormalTextureId < scene.Textures.Count)
            {
                tangent = CreateOrthonormalBasis(normal);
                bitangent = Vec3.Cross(normal, tangent);
                Vec3 texNormal = scene.Images[scene.Textures[mat.NormalTextureId]].Sample(hit.UV).AsVector3() * 2 - Vec3.One;
                normal = Vec3.Normalize(
                    tangent * texNormal.X +
                    bitangent * texNormal.Y +
                    normal * texNormal.Z);
            }

            // Add emission with multiple importance sampling
            //if (bounce == 0 || specularBounce)
            {
                Vec3 emission;
                if (mat.EmissionTextureId >= 0)
                {
                    emission = scene.Images[scene.Textures[mat.EmissionTextureId]].Sample(hit.UV).AsVector3()*10;
                }
                else
                {
                    emission = mat.Emission;
                }

                radiance += throughput * emission;
            }

            // Prepare surface information
            Vec3 hitPoint = ray.GetHitPoint(hit.Distance) + normal * 0.001f;
            Vec3 shadingNormal = FaceForward(normal, ray.Direction);
            tangent = CreateOrthonormalBasis(shadingNormal);
            bitangent = Vec3.Cross(shadingNormal, tangent);

            // Material properties
            float roughness = float.Max(mat.Roughness, 0.05f);
            float metallic = mat.Metallic;

            // Sample new direction based on material properties
            Vec3 worldDir;
            float pdf;
            Vec3 brdf;

            // Direct light sampling (Next Event Estimation)
            //if (bounce < 3 && roughness > 0.1f && scene.Lights.Count > 0)
            //{
            //    int lightIdx = (int)(RandomFloat() * scene.Lights.Count);
            //    var light = scene.Lights[lightIdx];

            //    (Vec3 lightPoint, Vec3 lightNormal, float lightPdf) = light.Sample(hitPoint);
            //    Vec3 toLight = lightPoint - hitPoint;
            //    float distSq = toLight.LengthSquared();
            //    float dist = MathF.Sqrt(distSq);
            //    Vec3 lightDir = toLight / dist;

            //    float NdotL = Math.Max(0, Vec3.Dot(shadingNormal, lightDir));
            //    float LdotN = Math.Max(0, Vec3.Dot(lightNormal, -lightDir));

            //    if (NdotL > 0 && LdotN > 0)
            //    {
            //        Ray shadowRay = new Ray(hitPoint, lightDir, 0.001f, dist - 0.002f);
            //        if (!BVH.IntersectAny(shadowRay))
            //        {
            //            // BRDF evaluation
            //            Vec3 halfVec = Vec3.Normalize(-ray.Direction + lightDir);
            //            float NdotH = Math.Max(0, Vec3.Dot(shadingNormal, halfVec));
            //            float NdotV = Math.Max(0, Vec3.Dot(shadingNormal, -ray.Direction));

            //            // Disney BRDF approximation
            //            Vec3 F0 = Vec3.Lerp(new Vec3(0.04f), albedo, metallic);
            //            Vec3 F = FresnelSchlick(NdotV, F0);
            //            float D = DistributionGGX(NdotH, roughness);
            //            float G = GeometrySmith(NdotV, NdotL, roughness);
            //            Vec3 specular = (F * D * G) / (4 * NdotV * NdotL + 0.001f);

            //            Vec3 kS = F;
            //            Vec3 kD = (Vec3.One - kS) * (1 - metallic);
            //            Vec3 diffuse = kD * albedo / MathF.PI;

            //            Vec3 brdfVal = diffuse + specular;
            //            float cosTheta = Math.Max(NdotL, 0.001f);

            //            // Light contribution
            //            Vec3 lightContrib = light.Emission * brdfVal * cosTheta * LdotN / (distSq * lightPdf);
            //            radiance += throughput * lightContrib;
            //        }
            //    }
            //}

            // BRDF Importance Sampling
            if (roughness < 0.1f && metallic > 0.7f)  // Specular material
            {
                // GGX importance sampling for specular
                Vec2 r = Random2();
                Vec3 halfVector = SampleGGX(r.X, r.Y, roughness, shadingNormal, tangent, bitangent);
                worldDir = Vec3.Reflect(ray.Direction, halfVector);

                // Calculate PDF and BRDF
                float NdotV = Math.Max(0, Vec3.Dot(shadingNormal, -ray.Direction));
                float NdotL = Math.Max(0, Vec3.Dot(shadingNormal, worldDir));
                float NdotH = Math.Max(0, Vec3.Dot(shadingNormal, halfVector));

                Vec3 F0 = Vec3.Lerp(new Vec3(0.04f), albedo, metallic);
                Vec3 F = FresnelSchlick(Math.Max(0, Vec3.Dot(halfVector, -ray.Direction)), F0);
                float D = DistributionGGX(NdotH, roughness);
                pdf = (D * NdotH) / (4 * Math.Max(1e-8f, Vec3.Dot(-ray.Direction, halfVector)));
                brdf = F * D / (4 * NdotV * NdotL + 1e-8f);

                specularBounce = true;
            }
            else  // Diffuse or glossy material
            {
                float r1 = RandomFloat();
                float r2 = RandomFloat();
                // Precomputed cosine-weighted sampling
                float cosTheta = float.Sqrt(1 - r2);
                float sinTheta = float.Sqrt(r2);
                float phi = 2 * float.Pi * r1;

                // Local space direction
                Vec3 localDir = new Vec3(
                    MathF.Cos(phi) * sinTheta,
                    MathF.Sin(phi) * sinTheta,
                    cosTheta
                );

                // Transform to world space
                worldDir = localDir.X * tangent + localDir.Y * bitangent + localDir.Z * shadingNormal;

                // Compute PDF for cosine-weighted hemisphere
                pdf = cosTheta / MathF.PI;

                // Disney BRDF with metallic and roughness
                float NdotV = Math.Max(0, Vec3.Dot(shadingNormal, -ray.Direction));
                float NdotL = Math.Max(0, Vec3.Dot(shadingNormal, worldDir));
                Vec3 halfVec = Vec3.Normalize(-ray.Direction + worldDir);
                float NdotH = Math.Max(0, Vec3.Dot(shadingNormal, halfVec));

                Vec3 F0 = Vec3.Lerp(new Vec3(0.04f), albedo, metallic);
                Vec3 F = FresnelSchlick(Math.Max(0, Vec3.Dot(halfVec, -ray.Direction)), F0);
                float D = DistributionGGX(NdotH, roughness);
                float G = GeometrySmith(NdotV, NdotL, roughness);

                Vec3 specular = (F * D * G) / (4 * NdotV * NdotL + 0.001f);
                Vec3 kS = F;
                Vec3 kD = (Vec3.One - kS) * (1 - metallic);
                Vec3 diffuse = kD * albedo / MathF.PI;

                brdf = diffuse + specular;
                specularBounce = false;
            }

            // Update throughput with BRDF and PDF
            float cosThetaOut = MathF.Max(1e-5f, Vec3.Dot(shadingNormal, worldDir));
            throughput *= brdf * cosThetaOut / pdf;

            // Update ray for next bounce
            ray = new Ray(hitPoint, worldDir);

            // Russian Roulette termination with energy preservation
            if (bounce > 2)
            {
                float q = Math.Max(0.05f, 1 - MathF.Max(throughput.X, MathF.Max(throughput.Y, throughput.Z)));
                if (RandomFloat() < q) break;
                throughput /= (1 - q);
            }
        }

        return radiance;
    }

    // Helper functions
    float DistributionGGX(float NdotH, float roughness)
    {
        float a = roughness * roughness;
        float a2 = a * a;
        float denom = NdotH * NdotH * (a2 - 1) + 1;
        return a2 / (MathF.PI * denom * denom);
    }

    float GeometrySchlickGGX(float NdotV, float roughness)
    {
        float r = (roughness + 1);
        float k = (r * r) / 8;
        return NdotV / (NdotV * (1 - k) + k);
    }

    float GeometrySmith(float NdotV, float NdotL, float roughness)
    {
        return GeometrySchlickGGX(NdotV, roughness) * GeometrySchlickGGX(NdotL, roughness);
    }

    Vec3 FresnelSchlick(float cosTheta, Vec3 F0)
    {
        return F0 + (Vec3.One - F0) * MathF.Pow(1 - cosTheta, 5);
    }

    Vec3 SampleGGX(float u1, float u2, float roughness, Vec3 normal, Vec3 tangent, Vec3 bitangent)
    {
        float a = roughness * roughness;
        float phi = 2 * MathF.PI * u1;
        float cosTheta = MathF.Sqrt((1 - u2) / (1 + (a * a - 1) * u2));
        float sinTheta = MathF.Sqrt(1 - cosTheta * cosTheta);

        Vec3 halfVector = new Vec3(
            MathF.Cos(phi) * sinTheta,
            MathF.Sin(phi) * sinTheta,
            cosTheta
        );

        return halfVector.X * tangent + halfVector.Y * bitangent + halfVector.Z * normal;
    }

    static Vec3 CreateOrthonormalBasis(Vec3 normal)
    {
        Vec3 tangent;
        if (MathF.Abs(normal.X) > MathF.Abs(normal.Y))
            tangent = Vec3.Normalize(new Vec3(normal.Z, 0, -normal.X));
        else
            tangent = Vec3.Normalize(new Vec3(0, -normal.Z, normal.Y));
        return tangent;
    }

    static Vec3 FaceForward(Vec3 normal, Vec3 direction)
    {
        return Vec3.Dot(normal, direction) < 0 ? normal : -normal;
    }

    static Vec2 Random2()
    {
        return rng.Value.NextVec2();
    }

    static float RandomFloat()
    {
        return rng.Value.NextFloat();
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

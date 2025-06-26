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

    // Reservoir sampling structures
    struct ReservoirSample
    {
        public Vec3 Direction;
        public Vec3 Radiance;
        public float Pdf;
        public float Weight;
    }

    struct Reservoir
    {
        public ReservoirSample Sample;
        public float WSum;
        public int M;

        public void Init()
        {
            WSum = 0;
            M = 0;
            Sample = default;
        }

        public static Reservoir Update(Reservoir self, ReservoirSample candidate, float weight)
        {
            self.WSum += weight;
            self.M++;
            if (rng.Value!.NextFloat() < weight / self.WSum)
            {
                self.Sample = candidate;
            }
            return self;
        }

        public static Reservoir Combine(Reservoir self, Reservoir other, float p_q, float p_p)
        {
            float weight = other.WSum / Math.Max(1, other.M) * p_p / Math.Max(1e-6f, p_q);
            self = Update(self, other.Sample, weight);
            return self;
        }
    }

    // Global buffers and state
    private Reservoir[] _currentReservoirs;
    private Reservoir[] _previousReservoirs;
    private Vec3[] _positionBuffer;
    private Vec3[] _normalBuffer;
    private int _frameCount = 0;
    private int _width = 0;
    private static readonly ThreadLocal<PCG32> rng = new(() =>
    {
        ulong threadSeed = (ulong)Environment.TickCount ^ (ulong)Environment.CurrentManagedThreadId;
        return new PCG32(threadSeed);
    });

    private Vec3 GetSkyRadiance(Vec3 direction)
    {
        return new();
        Vec3 unitDir = Vec3.Normalize(direction);
        float t = 0.5f * (unitDir.Y + 1.0f);

        // Sun disk approximation
        Vec3 sunDir = Vec3.Normalize(new Vec3(0.35f, 0.9f, 0.2f));
        float sun = Math.Clamp(Vec3.Dot(unitDir, sunDir), 0, 1);
        sun = MathF.Pow(sun, 512) * 20f;

        // Improved sky model
        Vec3 sky = Vec3.Lerp(
            new Vec3(0.4f, 0.5f, 0.9f) * 0.2f,
            new Vec3(0.8f, 0.9f, 1.0f) * 1.5f,
            t) + new Vec3(sun, sun * 0.9f, sun * 0.6f);

        return sky;
    }

    private Vec3 EvaluateBRDF(Vec3 albedo, float metallic, float roughness,
                             Vec3 wo, Vec3 wi, Vec3 normal)
    {
        float NdotV = Math.Max(1e-5f, Vec3.Dot(normal, wo));
        float NdotL = Math.Max(1e-5f, Vec3.Dot(normal, wi));
        Vec3 halfVec = Vec3.Normalize(wo + wi);
        float NdotH = Math.Max(1e-5f, Vec3.Dot(normal, halfVec));
        float HdotV = Math.Max(1e-5f, Vec3.Dot(halfVec, wo));

        Vec3 F0 = Vec3.Lerp(new Vec3(0.04f), albedo, metallic);
        Vec3 F = F0 + (Vec3.One - F0) * MathF.Pow(1 - HdotV, 5);

        float a = roughness * roughness;
        float a2 = a * a;
        float denom = NdotH * NdotH * (a2 - 1) + 1;
        float D = a2 / (MathF.PI * denom * denom);

        float k = (roughness + 1) * (roughness + 1) / 8;
        float G1 = NdotV / (NdotV * (1 - k) + k);
        float G2 = NdotL / (NdotL * (1 - k) + k);
        float G = G1 * G2;

        Vec3 specular = (F * D * G) / (4 * NdotV * NdotL + 1e-5f);
        Vec3 kS = F;
        Vec3 kD = (Vec3.One - kS) * (1 - metallic);
        Vec3 diffuse = kD * albedo / MathF.PI;

        return diffuse + specular;
    }
    Vec3 TraceRay(Ray ray, int pixelIdx)
    {
        Vec3 throughput = Vec3.One;
        Vec3 radiance = Vec3.Zero;
        bool specularBounce = true;
        Vec3 hitPoint = Vec3.Zero;
        Vec3 shadingNormal = Vec3.Zero;
        Vec3 albedo = Vec3.One;
        float roughness = 0.5f;
        float metallic = 0.0f;

        for (int bounce = 0; bounce < MaxDepth; bounce++)
        {
            if (!BVH.IntersectClosest(ray, out HitInfo hit))
            {
                // Handle environment lighting
                if (bounce == 0)
                {
                    radiance += Vec3.Min(throughput * GetSkyRadiance(ray.Direction), new Vec3(0.5f));
                }
                break;
            }

            var prim = scene.Primitives[hit.PrimIndex];
            var mat = scene.Materials[prim.MaterialIndex];
            albedo = mat.AlbedoTextureId >= 0 ?
                scene.Images[scene.Textures[mat.AlbedoTextureId]].Sample(hit.UV).AsVector3() :
                mat.Albedo;

            Vec3 normal = hit.Normal;
            if (mat.NormalTextureId >= 0 && mat.NormalTextureId < scene.Textures.Count)
            {
                Vec3 tangent2 = CreateOrthonormalBasis(normal);
                Vec3 bitangent2 = Vec3.Cross(normal, tangent2);
                Vec3 texNormal = scene.Images[scene.Textures[mat.NormalTextureId]].Sample(hit.UV).AsVector3() * 2 - Vec3.One;
                normal = Vec3.Normalize(
                    tangent2 * texNormal.X +
                    bitangent2 * texNormal.Y +
                    normal * texNormal.Z);
            }

            // Prepare surface information
            hitPoint = ray.GetHitPoint(hit.Distance) + normal * 0.001f;
            shadingNormal = FaceForward(normal, ray.Direction);
            Vec3 tangent = CreateOrthonormalBasis(shadingNormal);
            Vec3 bitangent = Vec3.Cross(shadingNormal, tangent);

            // Material properties
            roughness = Math.Max(mat.Roughness, 0.05f);
            metallic = mat.Metallic;

            // Add emission - ALWAYS added regardless of bounce or material type
            Vec3 emission = mat.EmissionTextureId >= 0 ?
                scene.Images[scene.Textures[mat.EmissionTextureId]].Sample(hit.UV).AsVector3() * 5 :
                mat.Emission;
            radiance += Vec3.Min(throughput * emission, new Vec3(0.9f)); 

            // ReSTIR for environment lighting ONLY on first bounce
            if (bounce == 0)
            {
                _positionBuffer[pixelIdx] = hitPoint;
                _normalBuffer[pixelIdx] = shadingNormal;

                Reservoir reservoir = new Reservoir();
                reservoir.Init();
                const int M = 32;

                // Generate candidate samples
                for (int i = 0; i < M; i++)
                {
                    // Cosine-weighted hemisphere sampling
                    float r1 = rng.Value.NextFloat();
                    float r2 = rng.Value.NextFloat();
                    float phi = 2 * MathF.PI * r1;
                    float cosTheta = MathF.Sqrt(1 - r2);
                    float sinTheta = MathF.Sqrt(r2);

                    Vec3 localDir = new Vec3(
                        MathF.Cos(phi) * sinTheta,
                        MathF.Sin(phi) * sinTheta,
                        cosTheta
                    );

                    Vec3 worldDir2 =
                        tangent * localDir.X +
                        bitangent * localDir.Y +
                        shadingNormal * localDir.Z;

                    float pdf2 = cosTheta / MathF.PI;
                    Vec3 envRadiance = GetSkyRadiance(worldDir2);

                    // Evaluate BRDF
                    Vec3 wi = worldDir2;
                    Vec3 wo = -ray.Direction;
                    Vec3 brdf2 = EvaluateBRDF(albedo, metallic, roughness, wo, wi, shadingNormal);
                    float cosThetaOut2 = Math.Max(1e-5f, Vec3.Dot(shadingNormal, wi));

                    Vec3 unshadowed = brdf2 * envRadiance * cosThetaOut2;
                    float weight = Luminance(unshadowed);

                    // Remove the first-frame hack - use actual computed weight
                    ReservoirSample candidate = new ReservoirSample
                    {
                        Direction = worldDir2,
                        Radiance = envRadiance,
                        Pdf = pdf2,
                        Weight = weight
                    };

                    reservoir = Reservoir.Update(reservoir, candidate, weight);
                }

                // Temporal reuse from previous frame
                if (_frameCount > 1)
                {
                    Reservoir prev = _previousReservoirs[pixelIdx];
                    Vec3 prevHitPos = _positionBuffer[pixelIdx];
                    Vec3 prevNormal = _normalBuffer[pixelIdx];

                    float posDiff = (hitPoint - prevHitPos).LengthSquared();
                    float normalDiff = 1 - Vec3.Dot(shadingNormal, prevNormal);

                    if (posDiff < 0.1f && normalDiff < 0.05f && prev.M > 0)
                    {
                        // Recalculate weight with current BRDF
                        Vec3 wi = prev.Sample.Direction;
                        Vec3 wo = -ray.Direction;
                        Vec3 brdf2 = EvaluateBRDF(albedo, metallic, roughness, wo, wi, shadingNormal);
                        float cosThetaOut2 = Math.Max(1e-5f, Vec3.Dot(shadingNormal, wi));
                        Vec3 unshadowed = brdf2 * prev.Sample.Radiance * cosThetaOut2;
                        float newWeight = Luminance(unshadowed);

                        // Only combine if we have valid weights
                        if (!float.IsNaN(newWeight) && newWeight > 0)
                        {
                            reservoir = Reservoir.Combine(reservoir, prev, prev.Sample.Weight, newWeight);
                        }
                    }
                }

                // Spatial reuse (3x3 neighborhood)
                int[] offsets = { -1, 1, -_width, _width, -_width - 1, -_width + 1, _width - 1, _width + 1, 0 };
                for (int i = 0; i < offsets.Length; i++)
                {
                    int neighborIdx = pixelIdx + offsets[i];
                    if (neighborIdx < 0 || neighborIdx >= _positionBuffer.Length)
                        continue;

                    Vec3 neighborPos = _positionBuffer[neighborIdx];
                    Vec3 neighborNormal = _normalBuffer[neighborIdx];

                    float posDiff = (hitPoint - neighborPos).LengthSquared();
                    float normalDiff = 1 - Vec3.Dot(shadingNormal, neighborNormal);

                    // Only reuse valid reservoirs
                    if (posDiff < 0.2f && normalDiff < 0.1f &&
                        _currentReservoirs[neighborIdx].M > 0)
                    {
                        reservoir = Reservoir.Combine(reservoir, _currentReservoirs[neighborIdx], 1.0f, 1.0f);
                    }
                }

                // Evaluate selected sample if reservoir is valid
                if (reservoir.M > 0 && reservoir.WSum > 0 && reservoir.Sample.Pdf > 0)
                {
                    Ray shadowRay = new Ray(hitPoint, reservoir.Sample.Direction);
                    if (!BVH.IntersectAny(shadowRay))
                    {
                        Vec3 wi = reservoir.Sample.Direction;
                        Vec3 wo = -ray.Direction;
                        Vec3 brdf2 = EvaluateBRDF(albedo, metallic, roughness, wo, wi, shadingNormal);
                        float cosThetaOut2 = Math.Max(1e-5f, Vec3.Dot(shadingNormal, wi));
                        Vec3 contrib = brdf2 * reservoir.Sample.Radiance * cosThetaOut2;

                        // Calculate ReSTIR weight with safeguards
                        float restirWeight = (reservoir.WSum / reservoir.M) / reservoir.Sample.Pdf;
                        if (!float.IsNaN(restirWeight) && !float.IsInfinity(restirWeight))
                        {
                            radiance += Vec3.Min(throughput * contrib * restirWeight, new Vec3(0.5f));
                        }
                    }
                }

                // Save reservoir for next frame
                _currentReservoirs[pixelIdx] = reservoir;
            }

            // Sample new direction for next bounce
            Vec3 worldDir;
            float pdf;
            Vec3 brdf;

            if (roughness < 0.1f && metallic > 0.7f)
            {
                // GGX importance sampling
                Vec2 r = new Vec2(rng.Value.NextFloat(), rng.Value.NextFloat());
                Vec3 halfVector = SampleGGX(r.X, r.Y, roughness, shadingNormal, tangent, bitangent);
                worldDir = Vec3.Reflect(ray.Direction, halfVector);

                // Calculate PDF and BRDF
                float NdotV = Math.Max(0, Vec3.Dot(shadingNormal, -ray.Direction));
                float NdotL = Math.Max(0, Vec3.Dot(shadingNormal, worldDir));
                float NdotH = Math.Max(0, Vec3.Dot(shadingNormal, halfVector));

                Vec3 F0 = Vec3.Lerp(new Vec3(0.04f), albedo, metallic);
                Vec3 F = FresnelSchlick(Math.Max(0, Vec3.Dot(halfVector, -ray.Direction)), F0);
                float D = DistributionGGX(NdotH, roughness);
                pdf = (D * NdotH) / (4 * MathF.Max(1e-8f, Vec3.Dot(-ray.Direction, halfVector)));
                brdf = F * D / (4 * NdotV * NdotL + 1e-8f);

                specularBounce = true;
            }
            else
            {
                // Cosine-weighted sampling
                float r1 = rng.Value!.NextFloat();
                float r2 = rng.Value!.NextFloat();
                float phi = 2 * MathF.PI * r1;
                float cosTheta = MathF.Sqrt(1 - r2);
                float sinTheta = MathF.Sqrt(r2);

                Vec3 localDir = new Vec3(
                    MathF.Cos(phi) * sinTheta,
                    MathF.Sin(phi) * sinTheta,
                    cosTheta
                );

                worldDir =
                    tangent * localDir.X +
                    bitangent * localDir.Y +
                    shadingNormal * localDir.Z;

                pdf = cosTheta / MathF.PI;

                // Disney BRDF
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

            // Update throughput
            float cosThetaOut = MathF.Max(1e-5f, Vec3.Dot(shadingNormal, worldDir));
            throughput *= brdf * cosThetaOut / pdf;
            ray = new Ray(hitPoint, worldDir);

            // Russian Roulette termination
            if (bounce > 2)
            {
                float throughputLum = Luminance(throughput);
                float q = Math.Max(0.05f, 1 - throughputLum);

                // Add firefly prevention - terminate more aggressively on bright paths
                if (throughputLum > 0.2f)
                    q = Math.Min(q * 2f, 0.99f);

                if (rng.Value.NextFloat() < q)
                    break;

                throughput /= (1 - q);
            }
        }

        // Final NaN check for debugging
        if (float.IsNaN(radiance.X) || float.IsNaN(radiance.Y) || float.IsNaN(radiance.Z))
        {
            return new Vec3(0, 0, 0); // Bright red for NaN values
        }

        return radiance;
    }

    // Helper functions
    private float DistributionGGX(float NdotH, float roughness)
    {
        float a = roughness * roughness;
        float a2 = a * a;
        float denom = NdotH * NdotH * (a2 - 1) + 1;
        return a2 / (MathF.PI * denom * denom);
    }

    private float GeometrySchlickGGX(float NdotV, float roughness)
    {
        float r = (roughness + 1);
        float k = (r * r) / 8;
        return NdotV / (NdotV * (1 - k) + k);
    }

    private float GeometrySmith(float NdotV, float NdotL, float roughness)
    {
        return GeometrySchlickGGX(NdotV, roughness) * GeometrySchlickGGX(NdotL, roughness);
    }

    private Vec3 FresnelSchlick(float cosTheta, Vec3 F0)
    {
        return F0 + (Vec3.One - F0) * MathF.Pow(1 - cosTheta, 5);
    }

    private Vec3 SampleGGX(float u1, float u2, float roughness, Vec3 normal, Vec3 tangent, Vec3 bitangent)
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

        return
            tangent * halfVector.X +
            bitangent * halfVector.Y +
            normal * halfVector.Z;
    }

    private float Luminance(Vec3 color) =>
        0.2126f * color.X + 0.7152f * color.Y + 0.0722f * color.Z;

    // Rendering entry point
    void RenderScene(int xPixels, int yPixels)
    {
        // Initialize on first frame
        if (_frameCount == 0)
        {
            _width = xPixels;
            int size = xPixels * yPixels;
            _currentReservoirs = new Reservoir[size];
            _previousReservoirs = new Reservoir[size];
            _positionBuffer = new Vec3[size];
            _normalBuffer = new Vec3[size];

            // Initialize reservoirs
            for (int i = 0; i < size; i++)
            {
                _currentReservoirs[i].Init();
                _previousReservoirs[i].Init();
            }
        }

        var viewMatrix = Matrix4x4.CreateLookTo(Vec3.UnitY * 80, Vec3.UnitX, Vec3.UnitY);
        var projectionMatrix = Matrix4x4.CreatePerspectiveFieldOfView(
            60 * ToRadians, xPixels / (float)yPixels, 0.01f, 1000f);
        Matrix4x4.Invert(projectionMatrix, out var inverseProjectionMatrix);
        Matrix4x4.Invert(viewMatrix, out var inverseViewMatrix);

        _frameCount++;

        // Swap reservoirs for temporal reuse
        var temp = _previousReservoirs;
        _previousReservoirs = _currentReservoirs;
        _currentReservoirs = temp;

        // Initialize current frame reservoirs
        for (int i = 0; i < _currentReservoirs.Length; i++)
        {
            _currentReservoirs[i].Init();
        }

        // Tiled rendering
        var (dx, rx) = int.DivRem(xPixels, 64);
        int tilesX = dx + (rx != 0 ? 1 : 0);
        var (dy, ry) = int.DivRem(yPixels, 64);
        int tilesY = dy + (ry != 0 ? 1 : 0);

        Parallel.For(0, tilesX * tilesY, id =>
        {
            int tileX = id % tilesX;
            int tileY = id / tilesX;

            for (int y = tileY * 64; y < (tileY + 1) * 64; y++)
            {
                if (y >= yPixels) continue;
                for (int x = tileX * 64; x < (tileX + 1) * 64; x++)
                {
                    if (x >= xPixels) continue;
                    int pixelIdx = x + y * xPixels;

                    var uvCoords = new Vec2(x / (float)xPixels, y / (float)yPixels);
                    var rayCoords = uvCoords * 2f - Vec2.One;
                    var ray = Ray.CreateCameraRay(rayCoords, inverseViewMatrix, inverseProjectionMatrix);

                    Vec3 color = TraceRay(ray, pixelIdx);

                    // Incremental accumulation
                    Vec3 prev = _linearAccumulationBuffer[pixelIdx];
                    _linearAccumulationBuffer[pixelIdx] = prev + (color - prev) * (1.0f / _frameCount);
                }
            }
        });

        Console.WriteLine($"Frame: {_frameCount}");
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

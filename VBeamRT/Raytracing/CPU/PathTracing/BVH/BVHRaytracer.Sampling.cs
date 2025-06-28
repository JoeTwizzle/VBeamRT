using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using VBeamRT.Raytracing.CPU.Common;

namespace VBeamRT.Raytracing.CPU.PathTracing.BVH;

sealed partial class BVHRaytracer : IRenderer
{
    const float ToRadians = MathF.PI / 180f;
    const float ToDegrees = 180f / MathF.PI;
    const int MaxDepth = 32;
    private static readonly ThreadLocal<PCG32> rng = new(() =>
    {
        ulong threadSeed = (ulong)Environment.TickCount ^ (ulong)Environment.CurrentManagedThreadId;
        return new PCG32(threadSeed);
    });

    static Vec2 Random2()
    {
        return rng.Value!.NextVec2();
    }

    static float RandomFloat()
    {
        return rng.Value!.NextFloat();
    }

    static float Fract(float val)
    {
        return val % 1f;
    }

    static Vec3 Fract(Vec3 vec)
    {
        return new Vec3(vec.X % 1f, vec.Y % 1f, vec.Z % 1f);
    }
    static Vec2 Floor(Vec2 vec)
    {
        return new Vec2(float.Floor(vec.X), float.Floor(vec.Y));
    }
    static float Hash12(Vec2 p)
    {
        Vec3 p3 = Fract(new Vec3(p.X, p.Y, p.X) * .1031f);
        p3 += new Vec3(Vec3.Dot(p3, new Vec3(p3.Y, p3.Z, p3.X) + new Vec3(33.33f)));
        return Fract((p3.X + p3.Y) * p3.Z);
    }

    static float Hash13(Vec3 p3)
    {
        p3 = Fract(p3 * .1031f);
        p3 += new Vec3(Vec3.Dot(p3, new Vec3(p3.Z, p3.Y, p3.X) + new Vec3(31.32f)));
        return Fract((p3.X + p3.Y) * p3.Z);
    }
    static float Smoothstep(float edge0, float edge1, float x)
    {
        // Scale, and clamp x to 0..1 range
        x = float.Clamp((x - edge0) / (edge1 - edge0), 0, 1);

        return x * x * (3.0f - 2.0f * x);
    }
    static float Stars(Vec3 rd)
    {
        if (rd.Y <= 0.0) return 0.0f;

        float stars = 0.0f;
        Vec2 uv = Vec2.Divide(new(rd.X, rd.Z), new Vec2(rd.Y + 1f));

        Vec2 id = Floor(uv * 700f + new Vec2(234f));
        float star = Hash12(id);
        float brightness = Smoothstep(0.98f, 1.0f, star);
        stars += brightness * 0.5f;

        return stars * rd.Y;
    }

    static Vec3 GetSkyColor(Vec3 rd, Vec3 sunDir)
    {
        float sunHeight = sunDir.Y;
        float dayTime = Smoothstep(-0.1f, 0.1f, sunHeight);
        float duskDawn = Smoothstep(-0.3f, 0.3f, sunHeight) * (1.0f - dayTime);
        float night = 1.0f - Smoothstep(-0.3f, 0.0f, sunHeight);

        // colors
        Vec3 dayZenith = new Vec3(0.2f, 0.4f, 0.8f);
        Vec3 dayHorizon = new Vec3(0.8f, 0.9f, 1.0f);
        Vec3 duskZenith = new Vec3(0.2f, 0.15f, 0.3f);
        Vec3 duskHorizon = new Vec3(1.0f, 0.5f, 0.2f);
        Vec3 nightZenith = new Vec3(0.02f, 0.02f, 0.04f);
        Vec3 nightHorizon = new Vec3(0.04f, 0.05f, 0.08f);

        float horizon = float.Pow(1.0f - float.Max(rd.Y, 0.0f), 6.0f + night);

        // mix colors for each time of day
        Vec3 dayColor = Vec3.Lerp(dayZenith, dayHorizon, horizon);
        Vec3 duskColor = Vec3.Lerp(duskZenith, duskHorizon, horizon);
        Vec3 nightColor = Vec3.Lerp(nightZenith, nightHorizon, horizon);

        Vec3 skyColor = Vec3.Lerp(nightColor, duskColor, duskDawn);
        skyColor = Vec3.Lerp(skyColor, dayColor, dayTime);

        //float sunSpot = float.Max(Vec3.Dot(rd, sunDir), 0.0f);
        //float sunMask = float.Pow(sunSpot, 64.0f) * dayTime;
        float sun = float.Max(0.0f, Smoothstep(0.5f, 1.5f, Vec3.Dot(rd, sunDir))) * 0.5f;
        float sunDisc = Smoothstep(0.999f, 0.99925f, Vec3.Dot(rd, sunDir));

        float starsIntensity = Stars(rd);

        Vec3 moonDir = -sunDir;
        float moonGlow = float.Pow(float.Max(Vec3.Dot(rd, moonDir), 0.0f), 32.0f) * 0.5f;
        Vec3 moonlight = new Vec3(0.6f, 0.7f, 0.9f) * moonGlow;
        float moonDisc = Smoothstep(0.9995f, 1.0f, Vec3.Dot(rd, moonDir));

        return skyColor + new Vec3((sun + sunDisc) * dayTime) + ((moonlight + new Vec3(moonDisc + starsIntensity)) * night);
    }
    Vec3 TraceRay(Ray ray)
    {
        Vec3 throughput = Vec3.One;
        Vec3 radiance = Vec3.Zero;
        int triTests = 0;
        int boxTests = 0;
        int prevTri = -1;
        float currentIOR = 1.0f;  // Start in air

        // Firefly reduction state
        float pathLuminance = 0;
        const float MaxPathLuminance = 1000f;
        const float FireflyClampThreshold = 25f;

        // Change to while-loop for better bounce control
        int bounce = 0;
        while (bounce < MaxDepth)
        {
            if (!BVH.IntersectClosest(ray, out HitInfo hit, prevTri))
            {
                Vec3 sky = GetSkyColor(ray.Direction, Vec3.Normalize(new Vec3(0.5f, 0.5f, 0)));
                Vec3 envContrib = throughput * sky;
                float envLum = Luminance(envContrib);
                if (envLum > FireflyClampThreshold)
                {
                    envContrib *= FireflyClampThreshold / (envLum + 0.00001f);
                }
                radiance += envContrib;
                break;
            }

            triTests += hit.TriTests;
            boxTests += hit.BoxTests;

            var prim = scene.Primitives[hit.PrimIndex];
            var mat = scene.Materials[prim.MaterialIndex];
            //if (prim.MaterialIndex == 19)
            //{
            //    Console.WriteLine();
            //}
            // Sample albedo with alpha channel
            Vec4 albedo = mat.AlbedoTextureId >= 0 ?
                scene.Images[scene.Textures[mat.AlbedoTextureId]].Sample(hit.UV, FilterMode.Point) * mat.Albedo :
                mat.Albedo;
            float alpha = albedo.W;
            prevTri = hit.TriIndex;
            // ========================================
            // NEW: Alpha masking (cutout transparency)
            // ========================================
            if (mat.AlphaBlendMode == AlphaBlendMode.Clip && alpha < 0.999f)
            {
                // Pass through without interaction
                Vec3 newOrigin = ray.GetHitPoint(hit.Distance) + ray.Direction * 0.01f;
                ray = new Ray(newOrigin, ray.Direction);
                // Prevent self-intersection
                continue;  // Retain same bounce count
            }

            // Normal mapping
            Vec3 normal = hit.Normal;
            Vec3 tangent;
            Vec3 bitangent;
            if (mat.NormalTextureId >= 0 && mat.NormalTextureId < scene.Textures.Count)
            {
                tangent = CreateOrthonormalBasis(normal);
                bitangent = Vec3.Cross(normal, tangent);
                Vec3 texNormal = scene.Images[scene.Textures[mat.NormalTextureId]].Sample(hit.UV, FilterMode.Point).AsVector3() * 2 - Vec3.One;
                normal = Vec3.Normalize(
                    tangent * texNormal.X +
                    bitangent * texNormal.Y +
                    normal * texNormal.Z);
            }

            Vec3 emission;
            if (mat.EmissionTextureId >= 0)
            {
                var color = scene.Images[scene.Textures[mat.EmissionTextureId]].Sample(hit.UV, FilterMode.Point).AsVector3();

                // Special handling for specific materials
                if ((prim.MaterialIndex == 38 || prim.MaterialIndex == 59 || prim.MaterialIndex == 60) &&
                    Vec3.Dot(Vec3.Normalize(color), Vec3.Normalize(Vec3.One)) < 0.98f)
                {
                    color = Vec3.Zero;
                }
                else
                {
                    color += new Vec3(0.65f, 0.3f, 0);
                }
                emission = color * 25;
            }
            else
            {
                emission = mat.Emission;
            }

            // Firefly clamp for emission
            if (mat.AlphaBlendMode != AlphaBlendMode.Blend || alpha > 0.01f)
            {
                Vec3 emissionContrib = throughput * emission;
                float emissionLum = Luminance(emissionContrib);
                if (emissionLum > FireflyClampThreshold)
                {
                    emissionContrib *= FireflyClampThreshold / (emissionLum + 1e-6f);
                }
                radiance += emissionContrib;
            }

            // Prepare surface information
            Vec3 hitPoint = ray.GetHitPoint(hit.Distance) + normal * 0.001f;
            Vec3 shadingNormal = FaceForward(normal, ray.Direction);
            tangent = CreateOrthonormalBasis(shadingNormal);
            bitangent = Vec3.Cross(shadingNormal, tangent);

            // Material properties
            float roughness = Math.Max(mat.Roughness, 0.05f);
            float metallic = mat.Metallic;
            bool isTransmissive = mat.Transmission > 0;
            if (ray.Direction.X == 0)
            {
                Console.WriteLine();
            }
            Vec3 worldDir;
            float pdf;
            Vec3 brdf = Vec3.One;  // Default to white for transparency

            // ======================================================
            // NEW: Smooth transparency blending using alpha channel
            // ======================================================
            if (mat.AlphaBlendMode == AlphaBlendMode.Blend && alpha < 0.999f)
            {
                // Blend between surface interaction and transmission
                if (RandomFloat() < alpha)
                {
                    // Process surface interaction (keep existing code)
                    goto SurfaceInteraction;
                }
                else
                {
                    // Straight transmission (no change in direction)
                    worldDir = ray.Direction;
                    pdf = 1.0f;

                    // Apply transmission strength ONLY (no albedo multiplication)
                    throughput *= mat.Transmission;

                    // Update ray for next bounce
                    hitPoint = ray.GetHitPoint(hit.Distance) + worldDir * 0.001f;
                    ray = new Ray(hitPoint, worldDir);
                    prevTri = hit.TriIndex;
                    bounce++;
                    continue;
                }
            }

        SurfaceInteraction:
            // Handle transmissive materials (glass-like)
            if (isTransmissive)
            {
                Vec3 incident = -ray.Direction; // Incident vector (points toward surface)
                float cosi = Vec3.Dot(incident, shadingNormal); // Positive when entering
                bool isEntering = cosi > 0;

                // Determine IORs based on entering/exiting material
                float etaI = isEntering ? currentIOR : mat.IOR;
                float etaT = isEntering ? mat.IOR : currentIOR;
                float eta = etaI / (etaT + 0.00001f);

                // Calculate Fresnel reflection probability
                float F0 = MathF.Pow((etaT - etaI) / (etaT + etaI + 0.00001f), 2);
                float F_dielectric = F0 + (1 - F0) * MathF.Pow(1 - MathF.Abs(cosi), 5);

                // Choose between reflection and refraction
                float r = RandomFloat();
                bool isReflection = r < F_dielectric;

                if (isReflection)
                {
                    // Reflection direction
                    worldDir = Vec3.Reflect(incident, shadingNormal);
                }
                else
                {
                    // Refraction direction (returns zero on TIR)
                    worldDir = Refract(incident, shadingNormal, eta);
                    if (worldDir.LengthSquared() < 1e-5f)
                    {
                        // Total internal reflection - fallback to reflection
                        worldDir = Vec3.Reflect(incident, shadingNormal);
                        isReflection = true;
                    }
                    else
                    {
                        // Update current IOR for next bounce when exiting
                        currentIOR = isEntering ? mat.IOR : 1.0f;
                    }
                }

                // Calculate outgoing cosine
                float cosThetaOut = isReflection ?
                    float.Abs(cosi) :
                    float.Abs(Vec3.Dot(worldDir, shadingNormal));

                // Apply solid angle scaling factor for refraction

                Vec3 brdfAttenuation = Vec3.One;
                if (!isReflection)
                {
                    // For refraction, apply albedo tint
                    brdfAttenuation = albedo.AsVector3();
                }

                // Apply solid angle scaling factor for refraction
                if (!isReflection)
                {
                    brdfAttenuation *= (etaI * etaI) / (etaT * etaT + 1e-6f);
                }


                // Update PDF and throughput
                pdf = isReflection ? F_dielectric : (1 - F_dielectric);
                throughput *= brdfAttenuation * cosThetaOut / (pdf + 1e-6f);

                // Adjust hit point to prevent self-intersection
                hitPoint = ray.GetHitPoint(hit.Distance) + worldDir * 0.001f;
            }

            // Sample warping for specular materials
            if (roughness < 0.3f && metallic > 0.5f)
            {
                Vec2 r = Random2();
                float warpFactor = (1.0f - roughness * 2.0f) + 0.00001f;

                if (r.X < warpFactor)
                {
                    // Warped GGX sampling
                    Vec3 halfVector = SampleGGX(r.X / warpFactor, r.Y, roughness, shadingNormal, tangent, bitangent);
                    worldDir = Vec3.Reflect(ray.Direction, halfVector);

                    // Calculate PDF for warped sampling
                    float NdotV = float.Max(0, Vec3.Dot(shadingNormal, -ray.Direction));
                    float NdotL = float.Max(0, Vec3.Dot(shadingNormal, worldDir));
                    float NdotH = float.Max(0, Vec3.Dot(shadingNormal, halfVector));

                    // NEW: Specular-aware F0 calculation
                    Vec3 F0 = CalculateF0(mat, albedo.AsVector3(), metallic);

                    Vec3 F = FresnelSchlick(float.Max(0, Vec3.Dot(halfVector, -ray.Direction)), F0);
                    float D = DistributionGGX(NdotH, roughness);
                    pdf = (D * NdotH) / (4 * float.Max(1e-8f, Vec3.Dot(-ray.Direction, halfVector)));
                    brdf = F * D / (4 * NdotV * NdotL + 1e-8f);
                }
                else
                {
                    // Fallback to cosine sampling
                    float r1 = r.X / ((1 - warpFactor) + 0.00001f);
                    float r2 = r.Y;
                    float cosTheta = float.Sqrt(1 - r2);
                    float sinTheta = float.Sqrt(r2);
                    float phi = 2 * float.Pi * r1;

                    Vec3 localDir = new Vec3(
                        float.Cos(phi) * sinTheta,
                        float.Sin(phi) * sinTheta,
                        cosTheta
                    );

                    worldDir = localDir.X * tangent + localDir.Y * bitangent + localDir.Z * shadingNormal;
                    pdf = cosTheta / MathF.PI;

                    // NEW: Specular-aware F0 calculation
                    Vec3 F0 = CalculateF0(mat, albedo.AsVector3(), metallic);

                    // Standard BRDF evaluation
                    float NdotV = float.Max(0, Vec3.Dot(shadingNormal, -ray.Direction));
                    float NdotL = float.Max(0, Vec3.Dot(shadingNormal, worldDir));
                    Vec3 halfVec = Vec3.Normalize(-ray.Direction + worldDir);
                    float NdotH = float.Max(0, Vec3.Dot(shadingNormal, halfVec));
                    Vec3 F = FresnelSchlick(float.Max(0, Vec3.Dot(halfVec, -ray.Direction)), F0);
                    float D = DistributionGGX(NdotH, roughness);
                    float G = GeometrySmith(NdotV, NdotL, roughness);
                    Vec3 specular = (F * D * G) / (4 * NdotV * NdotL + 0.001f);
                    Vec3 kS = F;
                    Vec3 kD = (Vec3.One - kS) * (1 - metallic);
                    Vec3 diffuse = kD * albedo.AsVector3() / float.Pi;
                    brdf = diffuse + specular;
                }
            }
            else
            {
                // Standard sampling for non-specular materials
                float r1 = RandomFloat();
                float r2 = RandomFloat();
                float cosTheta = float.Sqrt(1 - r2);
                float sinTheta = float.Sqrt(r2);
                float phi = 2 * float.Pi * r1;

                Vec3 localDir = new Vec3(
                    float.Cos(phi) * sinTheta,
                    float.Sin(phi) * sinTheta,
                    cosTheta
                );

                worldDir = localDir.X * tangent + localDir.Y * bitangent + localDir.Z * shadingNormal;
                pdf = cosTheta / float.Pi;

                // NEW: Specular-aware F0 calculation
                Vec3 F0 = CalculateF0(mat, albedo.AsVector3(), metallic);

                // BRDF evaluation
                float NdotV = Math.Max(0, Vec3.Dot(shadingNormal, -ray.Direction));
                float NdotL = Math.Max(0, Vec3.Dot(shadingNormal, worldDir));
                Vec3 halfVec = Vec3.Normalize(-ray.Direction + worldDir);
                float NdotH = Math.Max(0, Vec3.Dot(shadingNormal, halfVec));
                Vec3 F = FresnelSchlick(Math.Max(0, Vec3.Dot(halfVec, -ray.Direction)), F0);
                float D = DistributionGGX(NdotH, roughness);
                float G = GeometrySmith(NdotV, NdotL, roughness);
                Vec3 specular = (F * D * G) / (4 * NdotV * NdotL + 0.001f);
                Vec3 kS = F;
                Vec3 kD = (Vec3.One - kS) * (1 - metallic);
                Vec3 diffuse = kD * albedo.AsVector3() / float.Pi;
                brdf = diffuse + specular;
            }

            // Apply alpha attenuation to surface interactions
            if (mat.AlphaBlendMode == AlphaBlendMode.Blend && alpha < 0.999f)
            {
                throughput *= alpha;
            }

            {
                // Energy-based path termination
                float cosThetaOut = MathF.Max(1e-5f, Vec3.Dot(shadingNormal, worldDir));
                Vec3 newThroughput = throughput * brdf * cosThetaOut / (pdf + 1e-6f);
                float newLuminance = Luminance(newThroughput);
                if (pathLuminance + newLuminance > MaxPathLuminance)
                {
                    break;
                }

                throughput = newThroughput;
                pathLuminance += newLuminance;
                ray = new Ray(hitPoint, worldDir);
                prevTri = hit.TriIndex;
                bounce++;

                // Russian Roulette termination
                if (bounce > 2)
                {
                    float throughputLum = Luminance(throughput);
                    float q = Math.Max(0.05f, 1 - throughputLum);
                    if (throughputLum > 10f)
                    {
                        q = Math.Min(q * 1.5f, 0.99f);
                    }
                    if (RandomFloat() < q)
                    {
                        break;
                    }
                    throughput /= ((1 - q) + 0.00001f);
                }
            }
        }

        // Final firefly clamp
        float radianceLum = Luminance(radiance);
        if (radianceLum > FireflyClampThreshold)
        {
            radiance *= FireflyClampThreshold / radianceLum;
        }

        return radiance;
    }

    static Vec3 Refract(Vec3 I, Vec3 N, float eta)
    {
        float cosi = Math.Clamp(Vec3.Dot(I, N), -1, 1);
        float k = 1 - eta * eta * (1 - cosi * cosi);
        if (k < 0) return Vec3.Zero;
        return eta * I - (eta * cosi + MathF.Sqrt(k)) * N;
    }
    static Vec3 CalculateF0(Material mat, Vec3 albedoColor, float metallic)
    {
        // For high IOR materials (specular-glossiness workflow)
        if (mat.IOR > 100f)
        {
            // Calculate Fresnel reflectance at normal incidence
            float f0 = (mat.IOR - 1) / (mat.IOR + 1);
            f0 = f0 * f0;  // Square for Fresnel reflectance

            // Apply specular factor tint
            return mat.SpecularFactor * f0;
        }

        // Standard metallic workflow
        return Vec3.Lerp(new Vec3(0.04f), albedoColor, metallic);
    }
    static float Luminance(Vec3 c)
    {
        float lum = 0.2126f * c.X + 0.7152f * c.Y + 0.0722f * c.Z;
        return float.IsFinite(lum) ? lum : 0;
    }
    // Helper functions
    static float DistributionGGX(float NdotH, float roughness)
    {
        float a = roughness * roughness;
        float a2 = a * a;
        float denom = NdotH * NdotH * (a2 - 1) + 1;
        return a2 / (MathF.PI * denom * denom);
    }

    static float GeometrySchlickGGX(float NdotV, float roughness)
    {
        float r = (roughness + 1);
        float k = (r * r) / 8;
        return NdotV / (NdotV * (1 - k) + k);
    }

    static float GeometrySmith(float NdotV, float NdotL, float roughness)
    {
        return GeometrySchlickGGX(NdotV, roughness) * GeometrySchlickGGX(NdotL, roughness);
    }

    static Vec3 FresnelSchlick(float cosTheta, Vec3 F0)
    {
        return F0 + (Vec3.One - F0) * MathF.Pow(1 - cosTheta, 5);
    }

    static Vec3 SampleGGX(float u1, float u2, float roughness, Vec3 normal, Vec3 tangent, Vec3 bitangent)
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
        {
            tangent = Vec3.Normalize(new Vec3(normal.Z, 0, -normal.X));
        }
        else
        {
            tangent = Vec3.Normalize(new Vec3(0, -normal.Z, normal.Y));
        }

        return tangent;
    }

    static Vec3 FaceForward(Vec3 normal, Vec3 direction)
    {
        return Vec3.Dot(normal, direction) < 0 ? normal : -normal;
    }
}

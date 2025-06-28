using OpenTK.Mathematics;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Nodes;
using VBeamRT.Raytracing.CPU.PathTracing.BVH;
using VKGraphics;

namespace VBeamRT.Raytracing.CPU.Common;
public sealed class AnimationChannel
{
    public string NodeName;
    // "translation", "rotation", "scale"
    public string Path;
    public float[] Times;
    // Vec3 or Quaternion
    public Vector4[] Values;
}
public enum FilterMode
{
    Point,
    Bilinear
}

public sealed class Image
{
    public int Width { get; }
    public int Height { get; }
    private readonly Vec4[] _data; // RGB data

    public Image(int width, int height)
    {
        Width = width;
        Height = height;
        _data = new Vec4[width * height];
    }

    public void SetPixel(int x, int y, Vec4 color) => _data[y * Width + x] = color;
    public static Image LoadFromBytes(byte[] data)
    {
        using var image = SixLabors.ImageSharp.Image.Load<Rgba32>(data);
        var texture = new Image(image.Width, image.Height);
        // Apply gamma correction to linear RGB
        image.ProcessPixelRows(accessor =>
        {
            for (int y = 0; y < accessor.Height; y++)
            {
                var row = accessor.GetRowSpan(y);
                for (int x = 0; x < row.Length; x++)
                {
                    var pixel = row[x];
                    texture.SetPixel(x, y, new Vec4(
                        pixel.R / 255f,
                        pixel.G / 255f,
                        pixel.B / 255f,
                        pixel.A / 255f)
                    );
                }
            }
        });
        return texture;
    }
    public Vec4 Sample(Vec2 uv, FilterMode filterMode)
    {
        // Bilinear filtering
        float u = Math.Clamp(uv.X, 0, 1);
        float v = Math.Clamp(uv.Y, 0, 1);

        // Convert to pixel coordinates with safety margins
        u *= (Width - 1);
        v *= (Height - 1);

        int x0 = (int)u;
        int y0 = (int)v;

        if (filterMode == FilterMode.Point)
        {
            return GetPixel(x0, y0);
        }
        else if (filterMode == FilterMode.Bilinear)
        {

            int x1 = Math.Min(x0 + 1, Width - 1);
            int y1 = Math.Min(y0 + 1, Height - 1);

            // Ensure coordinates stay within bounds
            x0 = Math.Clamp(x0, 0, Width - 1);
            y0 = Math.Clamp(y0, 0, Height - 1);
            x1 = Math.Clamp(x1, 0, Width - 1);
            y1 = Math.Clamp(y1, 0, Height - 1);

            float fracU = u - x0;
            float fracV = v - y0;


            Vec4 c00 = GetPixel(x0, y0);
            Vec4 c10 = GetPixel(x1, y0);
            Vec4 c01 = GetPixel(x0, y1);
            Vec4 c11 = GetPixel(x1, y1);

            return Vec4.Lerp(
                Vec4.Lerp(c00, c10, fracU),
                Vec4.Lerp(c01, c11, fracU),
                fracV);
        }
        return GetPixel(x0, y0);
    }
    private Vec4 GetPixel(int x, int y)
    {
        // Safety check for coordinates
        x = Math.Clamp(x, 0, Width - 1);
        y = Math.Clamp(y, 0, Height - 1);
        return _data[y * Width + x];
    }
}
public enum AlphaBlendMode
{
    None,
    Clip,
    Blend
}

public struct Material
{
    public Vec4 Albedo;
    public Vec3 Emission;
    public Vec3 SpecularFactor;
    public AlphaBlendMode AlphaBlendMode;
    public float Roughness;
    public float Metallic;
    public float IOR;
    public float Transmission;
    public int AlbedoTextureId;
    public int EmissionTextureId;
    public int NormalTextureId;
    public int MetallicRoughnessTextureId;
}

public struct Light
{
    public Vec3 Position;
    public Vec3 Color;
    public float Intensity;
    public LightType Type;
}

public enum LightType
{
    Point,
    Directional,
    Spot,
    Area
}
public sealed class GltfScene
{
    public List<PrimitiveData> Primitives = [];
    public List<Vertex> Vertices = [];
    public List<Triangle> Triangles = [];
    public List<Material> Materials = [];
    public List<Light> Lights = [];
    public List<AnimationChannel> Animations = [];
    public List<Image> Images = [];
    public List<int> Textures = [];
}

public struct PrimitiveData
{
    public int MaterialIndex;

    public PrimitiveData(int materialIndex)
    {
        MaterialIndex = materialIndex;
    }
}
public static class GltfLoader
{
    public static GltfScene Load(string path)
    {
        byte[] file = File.ReadAllBytes(path);
        bool isGlb = file.Length > 4 && BitConverter.ToUInt32(file, 0) == 0x46546C67; // "glTF"

        string json;
        byte[]? binBuffer = null;

        if (isGlb)
        {
            using var ms = new MemoryStream(file);
            using var br = new BinaryReader(ms);
            br.ReadUInt32(); // magic
            br.ReadUInt32(); // version
            int length = br.ReadInt32();
            int jsonChunkLength = br.ReadInt32();
            br.ReadUInt32(); // JSON chunk type
            json = System.Text.Encoding.UTF8.GetString(br.ReadBytes(jsonChunkLength));

            if (ms.Position < ms.Length)
            {
                int binLen = br.ReadInt32();
                br.ReadUInt32(); // BIN_CHUNK
                binBuffer = br.ReadBytes(binLen);
            }
        }
        else
        {
            json = File.ReadAllText(path);
        }

        var doc = JsonDocument.Parse(json);
        var root = doc.RootElement;

        var buffers = root.GetProperty("buffers").EnumerateArray().ToArray();
        var bufferDatas = new byte[buffers.Length][];
        for (int i = 0; i < buffers.Length; i++)
        {

            var buf = buffers[i];

            string? uri = null;
            if (buf.TryGetProperty("uri", out var uriProp))
            {
                uri = uriProp.GetString();
            }

            if (isGlb && string.IsNullOrEmpty(uri))
            {
                // Use embedded GLB binary chunk
                bufferDatas[i] = binBuffer!;
            }
            else
            {
                string filePath = Path.Combine(Path.GetDirectoryName(path)!, uri!);
                bufferDatas[i] = File.ReadAllBytes(filePath);
            }

        }

        var scene = new GltfScene();
        LoadMaterials(root, scene);
        LoadTextures(root, bufferDatas, scene, Environment.CurrentDirectory);
        LoadNodes(root, bufferDatas, scene);
        LoadMeshes(root, bufferDatas, scene);
        LoadAnimations(root, bufferDatas, scene);
        return scene;
    }
    private static void LoadMaterials(JsonElement root, GltfScene scene)
    {
        scene.Materials.Add(new Material
        {
            Albedo = new Vec4(0.8f, 0.8f, 0.8f, 1f),
            Emission = Vec3.Zero,
            Roughness = 1.0f,
            Metallic = 0.0f,
            IOR = 1.0f,
            AlphaBlendMode = AlphaBlendMode.None,
            Transmission = 1.0f,
            AlbedoTextureId = -1,
            NormalTextureId = -1,
            MetallicRoughnessTextureId = -1
        });
        if (!root.TryGetProperty("materials", out var materials)) return;

        foreach (var mat in materials.EnumerateArray())
        {
            var material = new Material
            {
                Albedo = new Vec4(0.8f, 0, 0.8f, 1f),
                Emission = Vec3.Zero,
                Roughness = 1.0f,
                Metallic = 0.0f,
                IOR = 1.0f,
                Transmission = 1.0f,
                AlphaBlendMode = AlphaBlendMode.None,
                AlbedoTextureId = -1,
                NormalTextureId = -1,
                EmissionTextureId = -1,
                MetallicRoughnessTextureId = -1
            };
            if (mat.TryGetProperty("name", out var name) && name.GetString() == "forMayaAOlambert17")
            {
                Console.WriteLine();
            }
            // PBR metallic-roughness parameters
            if (mat.TryGetProperty("pbrMetallicRoughness", out var pbr))
            {
                if (pbr.TryGetProperty("baseColorFactor", out var baseColor))
                {
                    material.Albedo = new Vec4(
                        baseColor[0].GetSingle(),
                        baseColor[1].GetSingle(),
                        baseColor[2].GetSingle(),
                        baseColor[3].GetSingle()
                    );
                }

                if (pbr.TryGetProperty("metallicFactor", out var metallic))
                    material.Metallic = metallic.GetSingle();

                if (pbr.TryGetProperty("roughnessFactor", out var roughness))
                    material.Roughness = roughness.GetSingle();

                // Texture handling
                if (pbr.TryGetProperty("baseColorTexture", out var albedoTexture))
                {
                    material.AlbedoTextureId = albedoTexture.GetProperty("index").GetInt32();
                }

                if (pbr.TryGetProperty("metallicRoughnessTexture", out var MetallicRoughnessTexture))
                {
                    material.MetallicRoughnessTextureId = MetallicRoughnessTexture.GetProperty("index").GetInt32();
                }
            }
            if (mat.TryGetProperty("alphaMode", out var alphaMode))
            {
                var am = alphaMode.GetString();
                if (am == "OPAQUE")
                {
                    material.AlphaBlendMode = AlphaBlendMode.None;
                }
                else if (am == "MASK")
                {
                    material.AlphaBlendMode = AlphaBlendMode.Clip;
                }
                else
                {
                    material.AlphaBlendMode = AlphaBlendMode.Blend;
                }
            }

            if (mat.TryGetProperty("normalTexture", out var normalTexture))
            {
                material.NormalTextureId = normalTexture.GetProperty("index").GetInt32();
            }
            // Emissive properties
            if (mat.TryGetProperty("emissiveFactor", out var emission))
            {
                material.Emission = new Vec3(
                    emission[0].GetSingle(),
                    emission[1].GetSingle(),
                    emission[2].GetSingle()
                );
            }
            if (mat.TryGetProperty("emissiveTexture", out var emissionTexture))
            {
                material.EmissionTextureId = emissionTexture.GetProperty("index").GetInt32();
            }

            // Transmission/extensions
            if (mat.TryGetProperty("extensions", out var extensions))
            {
                // KHR_materials_transmission
                if (extensions.TryGetProperty("KHR_materials_transmission", out var transmissionExt))
                {
                    material.Transmission = transmissionExt.GetProperty("transmissionFactor").GetSingle();
                }

                // KHR_materials_ior
                if (extensions.TryGetProperty("KHR_materials_ior", out var iorExt))
                {
                    material.IOR = iorExt.GetProperty("ior").GetSingle();
                }

                if (extensions.TryGetProperty("KHR_materials_pbrSpecularGlossiness", out var pbrSpecGlossExt))
                {
                    if (pbrSpecGlossExt.TryGetProperty("diffuseFactor", out var diffuse))
                    {
                        material.Albedo = new(
                            diffuse[0].GetSingle(),
                            diffuse[1].GetSingle(),
                            diffuse[2].GetSingle(),
                            diffuse[3].GetSingle());
                    }
                    else
                    {
                        material.Albedo = new(1);
                    }

                    if (pbrSpecGlossExt.TryGetProperty("diffuseTexture", out var albedoTexture))
                    {
                        material.AlbedoTextureId = albedoTexture.GetProperty("index").GetInt32();
                    }

                    if (pbrSpecGlossExt.TryGetProperty("glossinessFactor", out var gloss))
                    {
                        material.Roughness = 1f - gloss.GetSingle();
                    }
                    else
                    {
                        material.Roughness = 0;
                    }

                    Vec3 specFactor;
                    if (pbrSpecGlossExt.TryGetProperty("specularFactor", out var spec))
                    {
                        specFactor = new(
                            spec[0].GetSingle(),
                            spec[1].GetSingle(),
                            spec[2].GetSingle());
                    }
                    else
                    {
                        specFactor = new(1);
                    }


                    // BLENDER'S APPROACH: Always set IOR to 1000
                    material.IOR = 1000f;

                    // Store specular factor as a separate material property
                    material.SpecularFactor = specFactor;

                    // For metallic workflow conversion:
                    // Use diffuse as base color, and specular factor as F0 tint
                    material.Metallic = 0.0f; // Treat as dielectric initially

                    // For materials with black diffuse and white specular, treat as metal
                    if (material.Albedo.AsVector3().LengthSquared() < 0.01f &&
                        specFactor.LengthSquared() > 0.9f)
                    {
                        material.Metallic = 1.0f;
                        material.Albedo = new Vec4(specFactor, material.Albedo.W);
                    }
                }
            }

            scene.Materials.Add(material);
        }
    }


    private static void LoadNodes(JsonElement root, byte[][] buffers, GltfScene scene)
    {
        if (!root.TryGetProperty("nodes", out var nodes)) return;
        if (!root.TryGetProperty("extensions", out var extensions)) return;
        if (!extensions.TryGetProperty("KHR_lights_punctual", out var lightsExt)) return;
        if (!lightsExt.TryGetProperty("lights", out var lights)) return;

        // Parse lights
        var lightDefinitions = new Dictionary<int, Light>();
        foreach (var (light, idx) in lights.EnumerateArray().Select((v, i) => (v, i)))
        {
            var lightData = new Light
            {
                Color = Vec3.One,
                Intensity = 1.0f,
                Type = LightType.Point
            };

            if (light.TryGetProperty("color", out var color))
            {
                lightData.Color = new Vec3(
                    color[0].GetSingle(),
                    color[1].GetSingle(),
                    color[2].GetSingle()
                );
            }

            if (light.TryGetProperty("intensity", out var intensity))
                lightData.Intensity = intensity.GetSingle();

            if (light.TryGetProperty("type", out var type))
            {
                lightData.Type = type.GetString() switch
                {
                    "directional" => LightType.Directional,
                    "spot" => LightType.Spot,
                    _ => LightType.Point
                };
            }

            lightDefinitions[idx] = lightData;
        }

        // Assign lights to nodes
        foreach (var (node, nodeIdx) in nodes.EnumerateArray().Select((v, i) => (v, i)))
        {
            if (!node.TryGetProperty("extensions", out var nodeExt)) continue;
            if (!nodeExt.TryGetProperty("KHR_lights_punctual", out var nodeLightExt)) continue;

            int lightIndex = nodeLightExt.GetProperty("light").GetInt32();
            if (lightDefinitions.TryGetValue(lightIndex, out var light))
            {
                // Get node transform
                Vec3 position = Vec3.Zero;
                if (node.TryGetProperty("translation", out var translation))
                {
                    position = new Vec3(
                        translation[0].GetSingle(),
                        translation[1].GetSingle(),
                        translation[2].GetSingle()
                    );
                }

                light.Position = position;
                scene.Lights.Add(light);
            }
        }
    }
    private static void LoadAnimations(JsonElement root, byte[][] buffers, GltfScene scene)
    {
        if (!root.TryGetProperty("animations", out var anims)) return;
        var bViews = root.GetProperty("bufferViews").EnumerateArray().ToArray();
        var accessors = root.GetProperty("accessors").EnumerateArray().ToArray();

        foreach (var a in anims.EnumerateArray())
        {
            foreach (var chan in a.GetProperty("channels").EnumerateArray())
            {
                var sampler = a.GetProperty("samplers")[chan.GetProperty("sampler").GetInt32()];
                var target = chan.GetProperty("target");
                string path = target.GetProperty("path").GetString()!;
                string nodeName = target.GetProperty("node").GetInt32().ToString();

                var inputAcc = accessors[sampler.GetProperty("input").GetInt32()];
                var outputAcc = accessors[sampler.GetProperty("output").GetInt32()];

                float[] times = ReadScalarArray(inputAcc, bViews, buffers);
                var values = path == "rotation"
                    ? ReadQuaternionArray(outputAcc, bViews, buffers)
                    : ExtendScalarsToVector4(ReadScalarArray(outputAcc, bViews, buffers), path);

                scene.Animations.Add(new AnimationChannel
                {
                    NodeName = nodeName,
                    Path = path,
                    Times = times,
                    Values = values
                });
            }
        }
    }
    private static void LoadTextures(JsonElement root, byte[][] buffers, GltfScene scene, string basePath)
    {
        if (!root.TryGetProperty("images", out var images)) return;

        foreach (var image in images.EnumerateArray())
        {
            byte[]? imageData = null;

            // 1. Handle embedded data URI
            if (image.TryGetProperty("uri", out var uri))
            {
                string uriStr = uri.GetString()!;
                if (uriStr.StartsWith("data:image/"))
                {
                    string base64 = uriStr.Split(',')[1];
                    imageData = Convert.FromBase64String(base64);
                }
                else
                {
                    string filePath = Path.Combine(basePath, uriStr);
                    imageData = File.ReadAllBytes(filePath);
                }
            }
            // 2. Handle buffer view reference
            else if (image.TryGetProperty("bufferView", out var bufferView))
            {
                int viewIndex = bufferView.GetInt32();
                var view = root.GetProperty("bufferViews")[viewIndex];
                int offset = view.GetProperty("byteOffset").GetInt32OrDefault(0);
                int length = view.GetProperty("byteLength").GetInt32();
                int bufferIndex = view.GetProperty("buffer").GetInt32();
                imageData = new byte[length];
                Array.Copy(buffers[bufferIndex], offset, imageData, 0, length);
            }
            // 3. Handle KTX2/BasisU compressed textures
            else if (image.TryGetProperty("extensions", out var extensions))
            {
                // Placeholder for KTX2/BasisU decompression
                Console.WriteLine("Warning: Compressed textures not supported");
                continue;
            }

            scene.Images.Add(Image.LoadFromBytes(imageData!));
        }

        if (!root.TryGetProperty("textures", out var textures)) return;
        foreach (var texture in textures.EnumerateArray())
        {
            if (texture.TryGetProperty("source", out var index))
            {
                scene.Textures.Add(index.GetInt32());
            }
        }
    }
    private static void LoadMeshes(JsonElement root, byte[][] buffers, GltfScene scene)
    {
        var bufferViews = root.GetProperty("bufferViews").EnumerateArray().ToArray();
        var accessors = root.GetProperty("accessors").EnumerateArray().ToArray();
        if (!root.TryGetProperty("meshes", out var meshes)) return;

        foreach (var mesh in meshes.EnumerateArray())
        {
            foreach (var prim in mesh.GetProperty("primitives").EnumerateArray())
            {
                ReadPrimitive(prim, root, bufferViews, accessors, buffers, scene);
            }
        }
    }

    private static void ReadPrimitive(JsonElement prim, JsonElement root,
        JsonElement[] bViews, JsonElement[] accessors, byte[][] buffers, GltfScene scene)
    {
        var attrib = prim.GetProperty("attributes");
        var posAcc = accessors[attrib.GetProperty("POSITION").GetInt32()];
        var normAcc = attrib.TryGetProperty("NORMAL", out var n) ? accessors[n.GetInt32()] : default;
        var uvAcc = attrib.TryGetProperty("TEXCOORD_0", out var uv) ? accessors[uv.GetInt32()] : default;
        var idxAcc = accessors[prim.GetProperty("indices").GetInt32()];

        var posArr = ReadVector3Array(posAcc, bViews, buffers);
        var normArr = normAcc.ValueKind != JsonValueKind.Undefined ? ReadVector3Array(normAcc, bViews, buffers) : null;
        var uvArr = uvAcc.ValueKind != JsonValueKind.Undefined ? ReadVector2Array(uvAcc, bViews, buffers) : null;
        var idxArr = ReadIndices(idxAcc, bViews, buffers);
        int materialIndex = 0;
        if (prim.TryGetProperty("material", out var matIndexElement))
        {
            materialIndex = matIndexElement.GetInt32() + 1; // Offset by 1 (0 is default)
        }
        int baseVertIndex = scene.Vertices.Count;
        for (int i = 0; i < posArr.Length; i++)
        {
            scene.Vertices.Add(new Vertex
            {
                Position = posArr[i],
                Normal = normArr?.ElementAtOrDefault(i) ?? Vec3.UnitY,
                UV = uvArr?.ElementAtOrDefault(i) ?? Vec2.Zero
            });
        }

        for (int i = 0; i < idxArr.Length; i += 3)
        {
            scene.Triangles.Add(new Triangle
            {
                Index0 = baseVertIndex + idxArr[i],
                Index1 = baseVertIndex + idxArr[i + 1],
                Index2 = baseVertIndex + idxArr[i + 2],
                PrimIndex = scene.Primitives.Count
            });
        }
        scene.Primitives.Add(new PrimitiveData(materialIndex));

    }

    private static Vec3[] ReadVector3Array(byte[] buffer, JsonArray accessors, JsonArray views, int accessorIndex)
    {
        var accessor = accessors[accessorIndex]!.AsObject();
        var view = views[accessor["bufferView"]!.GetValue<int>()]!.AsObject();
        int offset = view["byteOffset"]?.GetValue<int>() ?? 0;
        offset += accessor["byteOffset"]?.GetValue<int>() ?? 0;
        int count = accessor["count"]!.GetValue<int>();
        int stride = view["byteStride"]?.GetValue<int>() ?? 12;

        Vec3[] result = new Vec3[count];
        for (int i = 0; i < count; i++)
        {
            int baseOffset = offset + i * stride;
            float x = BitConverter.ToSingle(buffer, baseOffset);
            float y = BitConverter.ToSingle(buffer, baseOffset + 4);
            float z = BitConverter.ToSingle(buffer, baseOffset + 8);
            result[i] = new Vec3(x, y, z);
        }
        return result;
    }
    public static Vec3[] ReadVector3Array(JsonElement accessor, JsonElement[] bufferViews, byte[][] buffers)
    {
        int count = accessor.GetProperty("count").GetInt32();
        int viewIndex = accessor.GetProperty("bufferView").GetInt32();
        var view = bufferViews[viewIndex];
        int viewOffset = view.TryGetProperty("byteOffset", out var vo) ? vo.GetInt32() : 0;
        int accessorOffset = accessor.TryGetProperty("byteOffset", out var ao) ? ao.GetInt32() : 0;
        int offset = viewOffset + accessorOffset;
        int bufferIndex = view.GetProperty("buffer").GetInt32();
        byte[] data = buffers[bufferIndex];

        int stride = view.TryGetProperty("byteStride", out var strideProp) ? strideProp.GetInt32() : 12;

        Vec3[] result = new Vec3[count];
        for (int i = 0; i < count; i++)
        {
            int index = offset + i * stride;
            float x = BitConverter.ToSingle(data, index);
            float y = BitConverter.ToSingle(data, index + 4);
            float z = BitConverter.ToSingle(data, index + 8);
            result[i] = new Vec3(x, y, z);
        }
        return result;
    }

    public static Vec2[] ReadVector2Array(JsonElement accessor, JsonElement[] bufferViews, byte[][] buffers)
    {
        int count = accessor.GetProperty("count").GetInt32();
        int viewIndex = accessor.GetProperty("bufferView").GetInt32();
        var view = bufferViews[viewIndex];
        int viewOffset = view.TryGetProperty("byteOffset", out var vo) ? vo.GetInt32() : 0;
        int accessorOffset = accessor.TryGetProperty("byteOffset", out var ao) ? ao.GetInt32() : 0;
        int offset = viewOffset + accessorOffset;
        int bufferIndex = view.GetProperty("buffer").GetInt32();
        byte[] data = buffers[bufferIndex];

        int stride = view.TryGetProperty("byteStride", out var strideProp) ? strideProp.GetInt32() : 8;

        Vec2[] result = new Vec2[count];
        for (int i = 0; i < count; i++)
        {
            int index = offset + i * stride;
            float x = BitConverter.ToSingle(data, index);
            float y = BitConverter.ToSingle(data, index + 4);
            result[i] = new Vec2(x, y);
        }
        return result;
    }

    public static int[] ReadIndices(JsonElement accessor, JsonElement[] bufferViews, byte[][] buffers)
    {
        int count = accessor.GetProperty("count").GetInt32();
        int viewIndex = accessor.GetProperty("bufferView").GetInt32();
        var view = bufferViews[viewIndex];
        int viewOffset = view.TryGetProperty("byteOffset", out var vo) ? vo.GetInt32() : 0;
        int accessorOffset = accessor.TryGetProperty("byteOffset", out var ao) ? ao.GetInt32() : 0;
        int offset = viewOffset + accessorOffset;
        int bufferIndex = view.GetProperty("buffer").GetInt32();
        byte[] data = buffers[bufferIndex];

        int componentType = accessor.GetProperty("componentType").GetInt32(); // 5121, 5123, 5125
        int[] result = new int[count];

        switch (componentType)
        {
            case 5121: // UNSIGNED_BYTE
                for (int i = 0; i < count; i++)
                {
                    result[i] = data[offset + i];
                }

                break;
            case 5123: // UNSIGNED_SHORT
                for (int i = 0; i < count; i++)
                {
                    result[i] = BitConverter.ToUInt16(data, offset + i * 2);
                }

                break;
            case 5125: // UNSIGNED_INT
                for (int i = 0; i < count; i++)
                {
                    result[i] = BitConverter.ToInt32(data, offset + i * 4);
                }

                break;
            default:
                throw new Exception("Unsupported index component type: " + componentType);
        }

        return result;
    }

    public static float[] ReadScalarArray(JsonElement accessor, JsonElement[] bufferViews, byte[][] buffers)
    {
        int count = accessor.GetProperty("count").GetInt32();
        int viewIndex = accessor.GetProperty("bufferView").GetInt32();
        var view = bufferViews[viewIndex];
        int offset = view.GetProperty("byteOffset").GetInt32() + accessor.GetProperty("byteOffset").GetInt32OrDefault(0);
        int bufferIndex = view.GetProperty("buffer").GetInt32();
        byte[] data = buffers[bufferIndex];

        int stride = view.TryGetProperty("byteStride", out var strideProp) ? strideProp.GetInt32() : 4;

        float[] result = new float[count];
        for (int i = 0; i < count; i++)
        {
            result[i] = BitConverter.ToSingle(data, offset + i * stride);
        }
        return result;
    }

    public static Vector4[] ReadQuaternionArray(JsonElement accessor, JsonElement[] bufferViews, byte[][] buffers)
    {
        int count = accessor.GetProperty("count").GetInt32();
        int viewIndex = accessor.GetProperty("bufferView").GetInt32();
        var view = bufferViews[viewIndex];
        int offset = view.GetProperty("byteOffset").GetInt32() + accessor.GetProperty("byteOffset").GetInt32OrDefault(0);
        int bufferIndex = view.GetProperty("buffer").GetInt32();
        byte[] data = buffers[bufferIndex];

        int stride = view.TryGetProperty("byteStride", out var strideProp) ? strideProp.GetInt32() : 16;

        Vector4[] result = new Vector4[count];
        for (int i = 0; i < count; i++)
        {
            int baseOffset = offset + i * stride;
            float x = BitConverter.ToSingle(data, baseOffset);
            float y = BitConverter.ToSingle(data, baseOffset + 4);
            float z = BitConverter.ToSingle(data, baseOffset + 8);
            float w = BitConverter.ToSingle(data, baseOffset + 12);
            result[i] = new Vector4(x, y, z, w);
        }
        return result;
    }

    public static Vector4[] ExtendScalarsToVector4(float[] flatArray, string path)
    {
        Vector4[] result = new Vector4[flatArray.Length / 3];
        for (int i = 0; i < result.Length; i++)
        {
            float x = flatArray[i * 3 + 0];
            float y = flatArray[i * 3 + 1];
            float z = flatArray[i * 3 + 2];
            result[i] = new Vector4(x, y, z, 0);
        }
        return result;
    }

    private static int GetInt32OrDefault(this JsonElement elem, int defaultVal)
    {
        return elem.ValueKind == JsonValueKind.Undefined ? defaultVal : elem.GetInt32();
    }

    private static Vec2[] ReadVector2Array(byte[] buffer, JsonArray accessors, JsonArray views, int accessorIndex)
    {
        var accessor = accessors[accessorIndex]!.AsObject();
        var view = views[accessor["bufferView"]!.GetValue<int>()]!.AsObject();
        int offset = view["byteOffset"]?.GetValue<int>() ?? 0;
        offset += accessor["byteOffset"]?.GetValue<int>() ?? 0;
        int count = accessor["count"]!.GetValue<int>();
        int stride = view["byteStride"]?.GetValue<int>() ?? 8;

        Vec2[] result = new Vec2[count];
        for (int i = 0; i < count; i++)
        {
            int baseOffset = offset + i * stride;
            float x = BitConverter.ToSingle(buffer, baseOffset);
            float y = BitConverter.ToSingle(buffer, baseOffset + 4);
            result[i] = new Vec2(x, y);
        }
        return result;
    }

    private static int[] ReadIndices(byte[] buffer, JsonArray accessors, JsonArray views, int accessorIndex)
    {
        var accessor = accessors[accessorIndex]!.AsObject();
        var view = views[accessor["bufferView"]!.GetValue<int>()]!.AsObject();
        int offset = view["byteOffset"]?.GetValue<int>() ?? 0;
        offset += accessor["byteOffset"]?.GetValue<int>() ?? 0;
        int count = accessor["count"]!.GetValue<int>();
        string componentType = accessor["componentType"]!.ToString();

        int[] result = new int[count];

        int componentSize = componentType switch
        {
            "5121" => 1, // UNSIGNED_BYTE
            "5123" => 2, // UNSIGNED_SHORT
            "5125" => 4, // UNSIGNED_INT
            _ => throw new Exception("Unsupported index component type.")
        };

        for (int i = 0; i < count; i++)
        {
            int indexOffset = offset + i * componentSize;
            result[i] = componentSize switch
            {
                1 => buffer[indexOffset],
                2 => BitConverter.ToUInt16(buffer, indexOffset),
                4 => BitConverter.ToInt32(buffer, indexOffset),
                _ => throw new Exception("Unsupported index component size.")
            };
        }

        return result;
    }
}

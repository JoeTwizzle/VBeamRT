using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;
using VBeamRT.Raytracing.CPU.Common;

namespace VBeamRT.Raytracing.CPU.PathTracing.BVH;



public struct Vertex
{
    public Vec3 Position;
    public Vec3 Normal;
    public Vec2 UV;
}

public struct Triangle
{
    public int Index0, Index1, Index2;
    public int PrimIndex;
}

public struct HitInfo
{
    public float Distance;       // t along ray
    public int PrimIndex;    // hit triangle index
    public int BoxTests;    // hit triangle index
    public int TriTests;    // hit triangle index
    public float BarycentricU;   // u barycentric coord
    public float BarycentricV;   // v barycentric coord
    public Vec3 Normal;       // interpolated normal
    public Vec2 UV;           // interpolated uv
}

public struct AABB
{
    public Vec3 Min;
    public Vec3 Max;

    public AABB(Vec3 min, Vec3 max)
    {
        Min = min;
        Max = max;
    }
    public readonly float SurfaceArea()
    {
        Vec3 len = Max - Min;
        return 2f * (len.X * len.Y + len.X * len.Z + len.Y * len.Z);
    }

    public readonly bool Intersect(Ray ray, float tMin, float tMax)
    {
        for (int i = 0; i < 3; i++)
        {
            float invD = 1f / ray.Direction[i];
            float t0 = (Min[i] - ray.Origin[i]) * invD;
            float t1 = (Max[i] - ray.Origin[i]) * invD;
            if (invD < 0.0f) (t0, t1) = (t1, t0);
            tMin = Math.Max(t0, tMin);
            tMax = Math.Min(t1, tMax);
            if (tMax <= tMin)
                return false;
        }
        return true;
    }

    public static AABB Union(AABB a, AABB b)
    {
        return new AABB
        {
            Min = Vec3.Min(a.Min, b.Min),
            Max = Vec3.Max(a.Max, b.Max)
        };
    }

    public static AABB FromTriangle(Vertex[] vertices, Triangle tri)
    {
        var v0 = vertices[tri.Index0].Position;
        var v1 = vertices[tri.Index1].Position;
        var v2 = vertices[tri.Index2].Position;
        return new AABB
        {
            Min = Vec3.Min(Vec3.Min(v0, v1), v2),
            Max = Vec3.Max(Vec3.Max(v0, v1), v2)
        };
    }
}

class BVHNode
{
    public AABB Bounds;

    // For leaf nodes:
    public int TriangleStart;  // Start index of triangles in leaf
    public int TriangleCount;  // Number of triangles in leaf

    // For interior nodes:
    public BVHNode? Left;
    public BVHNode? Right;

    public bool IsLeaf => TriangleCount > 0;

    public static BVHNode CreateLeaf(AABB bounds, int start, int count)
    {
        return new BVHNode
        {
            Bounds = bounds,
            TriangleStart = start,
            TriangleCount = count,
            Left = default,
            Right = default
        };
    }

    public static BVHNode CreateInterior(AABB bounds, BVHNode left, BVHNode right)
    {
        return new BVHNode
        {
            Bounds = bounds,
            TriangleStart = 0,
            TriangleCount = 0,
            Left = left,
            Right = right
        };
    }
}
/// <summary>
/// A surface area heuristic based BVH-2 implementation
/// </summary>
public sealed partial class BVH
{
    private readonly Vertex[] vertices;
    private readonly Triangle[] triangles;
    private readonly FlatBVHNode[] flatBVH;

    public BVH(Vertex[] vertices, Triangle[] triangles)
    {
        this.vertices = vertices;
        this.triangles = triangles;
        var root = Build(0, triangles.Length);
        List<FlatBVHNode> list = [];
        FlattenRecursive(root, list);
        flatBVH = [.. list];
    }

    private const int BinCount = 16;  // Increased for better SAH quality
    private const int LeafThreshold = 8;  // Increased leaf size
    private const int MaxDepth = 64;
    private const float TraversalCost = 1.0f;  // Explicit traversal cost
    private const float IntersectionCost = 2.0f;  // Explicit intersection cost

    struct BinInfo
    {
        public int Count;
        public Vec3 Min;
        public Vec3 Max;

        public void Reset()
        {
            Count = 0;
            Min = new Vec3(float.PositiveInfinity);
            Max = new Vec3(float.NegativeInfinity);
        }

        public void Add(AABB aabb)
        {
            Min = Vec3.Min(Min, aabb.Min);
            Max = Vec3.Max(Max, aabb.Max);
            Count++;
        }

        public AABB ToAABB() => new AABB(Min, Max);
    }

    private BVHNode Build(int start, int count, int depth = 0)
    {
        AABB bounds = ComputeBounds(start, count);

        // Leaf creation conditions
        if (count <= LeafThreshold || depth >= MaxDepth)
        {
            return MakeLeaf(start, count, bounds);
        }

        // First try SAH split
        if (TryFindBestSplit(start, count, bounds, out int axis, out int splitBin, out int mid))
        {
            return new BVHNode
            {
                Bounds = bounds,
                Left = Build(start, mid - start, depth + 1),
                Right = Build(mid, start + count - mid, depth + 1)
            };
        }

        // SAH failed - try spatial median fallback
        Vec3 extent = bounds.Max - bounds.Min;
        axis = 0;
        if (extent.Y > extent.X) axis = 1;
        if (extent.Z > extent[axis]) axis = 2;
        float splitPos = (bounds.Min[axis] + bounds.Max[axis]) * 0.5f;
        mid = PartitionTriangles(start, count, axis, splitPos);

        // Final fallback to leaf if needed
        if (mid == start || mid == start + count)
        {
            return MakeLeaf(start, count, bounds);
        }

        return new BVHNode
        {
            Bounds = bounds,
            Left = Build(start, mid - start, depth + 1),
            Right = Build(mid, start + count - mid, depth + 1)
        };
    }

    private bool TryFindBestSplit(int start, int count, AABB bounds, out int bestAxis, out int bestBin, out int mid)
    {
        bestAxis = -1;
        bestBin = -1;
        mid = 0;

        // Precompute centroid bounds
        AABB centroidBounds = ComputeCentroidBounds(start, count);
        Vec3 extent = centroidBounds.Max - centroidBounds.Min;

        // Degenerate centroid bounds check
        if (extent.X < 1e-5f && extent.Y < 1e-5f && extent.Z < 1e-5f)
        {
            return false;
        }

        float bestCost = float.PositiveInfinity;
        Span<BinInfo> bins = stackalloc BinInfo[BinCount];
        Span<AABB> leftBounds = stackalloc AABB[BinCount - 1];
        Span<int> leftCounts = stackalloc int[BinCount - 1];
        Span<AABB> rightBounds = stackalloc AABB[BinCount - 1];
        Span<int> rightCounts = stackalloc int[BinCount - 1];

        for (int axis = 0; axis < 3; axis++)
        {
            if (extent[axis] < 1e-5f) continue;

            float binScale = BinCount / extent[axis];
            for (int i = 0; i < BinCount; i++) bins[i].Reset();

            // Bin triangles
            for (int i = start; i < start + count; i++)
            {
                Vec3 centroid = Centroid(vertices, triangles[i]);
                int binIndex = Math.Clamp((int)((centroid[axis] - centroidBounds.Min[axis]) * binScale), 0, BinCount - 1);
                bins[binIndex].Add(AABB.FromTriangle(vertices, triangles[i]));
            }

            // Prefix sums (left side)
            AABB leftBound = new(new Vec3(float.PositiveInfinity), new Vec3(float.NegativeInfinity));
            int leftCount = 0;
            for (int i = 0; i < BinCount - 1; i++)
            {
                if (bins[i].Count > 0)
                {
                    leftBound = leftCount == 0 ? bins[i].ToAABB() : AABB.Union(leftBound, bins[i].ToAABB());
                    leftCount += bins[i].Count;
                }
                leftBounds[i] = leftBound;
                leftCounts[i] = leftCount;
            }

            // Suffix sums (right side)
            AABB rightBound = new(new Vec3(float.PositiveInfinity), new Vec3(float.NegativeInfinity));
            int rightCount = 0;
            for (int i = BinCount - 1; i > 0; i--)
            {
                if (bins[i].Count > 0)
                {
                    rightBound = rightCount == 0 ? bins[i].ToAABB() : AABB.Union(rightBound, bins[i].ToAABB());
                    rightCount += bins[i].Count;
                }
                rightBounds[i - 1] = rightBound;
                rightCounts[i - 1] = rightCount;
            }

            // Evaluate SAH costs
            for (int i = 0; i < BinCount - 1; i++)
            {
                if (leftCounts[i] == 0 || rightCounts[i] == 0) continue;

                // Updated SAH cost with explicit costs
                float cost = TraversalCost + IntersectionCost *
                    (leftCounts[i] * leftBounds[i].SurfaceArea() +
                     rightCounts[i] * rightBounds[i].SurfaceArea()) /
                    bounds.SurfaceArea();

                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestBin = i;
                }
            }
        }

        if (bestAxis == -1) return false;

        // Compare against leaf cost
        float leafCost = IntersectionCost * count;
        if (bestCost >= leafCost) return false;

        // Calculate split position
        float splitPos = centroidBounds.Min[bestAxis] + (bestBin + 1) * (extent[bestAxis] / BinCount);
        mid = PartitionTriangles(start, count, bestAxis, splitPos);

        // Validate split quality
        if (mid == start || mid == start + count)
        {
            return false;
        }

        return true;
    }

    // Existing helper methods (ComputeBounds, ComputeCentroidBounds, 
    // MakeLeaf, PartitionTriangles, Centroid) remain unchanged


    private AABB ComputeBounds(int start, int count)
    {
        // Initialize bounds with the first triangle in the range
        AABB bounds = AABB.FromTriangle(vertices, triangles[start]);

        // Expand bounds to include all triangles in the range
        for (int i = start + 1; i < start + count; i++)
        {
            AABB triBounds = AABB.FromTriangle(vertices, triangles[i]);
            bounds = AABB.Union(bounds, triBounds);
        }

        return bounds;
    }

    private AABB ComputeCentroidBounds(int start, int count)
    {
        Vec3 firstCentroid = Centroid(vertices, triangles[start]);
        Vec3 min = firstCentroid;
        Vec3 max = firstCentroid;

        for (int i = start + 1; i < start + count; i++)
        {
            Vec3 centroid = Centroid(vertices, triangles[i]);
            min = Vec3.Min(min, centroid);
            max = Vec3.Max(max, centroid);
        }

        return new AABB(min, max);
    }

    private BVHNode MakeLeaf(int start, int count, AABB bounds)
    {
        // Make a leaf node storing all triangles in this node, or recurse if count > threshold (optional)
        return new BVHNode
        {
            TriangleStart = start,
            TriangleCount = count,
            Bounds = bounds
        };
    }

    private int PartitionTriangles(int start, int count, int axis, float splitPos)
    {
        int i = start;
        int j = start + count - 1;

        while (i <= j)
        {
            float centroid = Centroid(vertices, triangles[i])[axis];
            if (centroid < splitPos)
            {
                i++;
            }
            else
            {
                (triangles[i], triangles[j]) = (triangles[j], triangles[i]);
                j--;
            }
        }

        return i;
    }

    static Vec3 Centroid(Vertex[] vertices, Triangle tri)
    {
        var v0 = vertices[tri.Index0].Position;
        var v1 = vertices[tri.Index1].Position;
        var v2 = vertices[tri.Index2].Position;
        return (v0 + v1 + v2) / 3f;
    }
}

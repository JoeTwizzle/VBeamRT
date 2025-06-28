using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Text;
using System.Threading.Tasks;
using VBeamRT.Raytracing.CPU.Common;

namespace VBeamRT.Raytracing.CPU.PathTracing.BVH;
public sealed partial class BVH
{
    public struct FlatBVHNode
    {
        public AABB Bounds;
        public int LeftChild;   // Index of left child, or -1 if leaf
        public int RightChild;  // Index of right child, or -1 if leaf
        public int TriangleStart; // Index into triangle array if leaf, -1 otherwise
        public int TriangleCount; // Number of triangles, 0 otherwise
        public int EscapeIndex;   // Index to jump to if skipping this node (used for stackless)
    }



    private static int FlattenRecursive(BVHNode node, List<FlatBVHNode> list)
    {
        int currentIndex = list.Count;

        // Determine if leaf node
        bool isLeaf = node.IsLeaf;

        FlatBVHNode flat = new FlatBVHNode
        {
            Bounds = node.Bounds,
            TriangleStart = isLeaf ? node.TriangleStart : -1,
            TriangleCount = isLeaf ? node.TriangleCount : 0,
            LeftChild = -1,
            RightChild = -1,
            EscapeIndex = -1
        };

        list.Add(flat); // Placeholder for now

        if (isLeaf)
        {
            // Leaf node, no children
            return currentIndex;
        }

        // Interior node - flatten children recursively
        int leftIndex = FlattenRecursive(node.Left, list);
        int rightIndex = FlattenRecursive(node.Right, list);

        // Update flat node with children indices and escape index
        flat.LeftChild = leftIndex;
        flat.RightChild = rightIndex;
        flat.EscapeIndex = list.Count; // Index to jump after this subtree

        list[currentIndex] = flat;

        return currentIndex;
    }

    public bool IntersectClosest(Ray ray, out HitInfo hitInfo, int ignoredTri = -1)
    {
        hitInfo = new HitInfo { Distance = ray.TMax, TriIndex = -1, PrimIndex = -1, BoxTests = 0, TriTests = 0 };

        Vec3 invDir = new Vec3(1f) / ray.Direction;

        Vector3i sign = new Vector3i(
            ray.Direction.X < 0 ? 1 : 0,
            ray.Direction.Y < 0 ? 1 : 0,
            ray.Direction.Z < 0 ? 1 : 0);

        const int MaxStackDepth = 64;
        Span<int> stack = stackalloc int[MaxStackDepth];
        int stackPtr = 0;
        stack[stackPtr++] = 0;  // root node index
        Span<float> distances = stackalloc float[4];

        while (stackPtr > 0)
        {
            var node = flatBVH[stack[--stackPtr]];
            hitInfo.BoxTests++;
            // Skip if ray misses node AABB or farther than current closest hit
            if (!IntersectAABB(node.Bounds, ray.Origin, sign, invDir, hitInfo.Distance, out _))
                continue;


            //if (!Intersections( ray.Origin, invDir, sign, [node.Bounds], hitInfo.Distance, out _))
            //    continue;

            // Leaf node test: TriangleCount > 0 means leaf
            if (node.TriangleCount > 0)
            {
                int start = node.TriangleStart;
                int end = start + node.TriangleCount;
                float currentMinDistance = hitInfo.Distance;
                int i = start;

                // Vectorized processing for batches of 4 triangles
                if (Vector128.IsHardwareAccelerated && (end - start) >= 4)
                {
                    const float EPSILON = 1e-6f;
                    Vector128<float> epsilonVec = Vector128.Create(ray.TMin);
                    Vector128<float> oneVec = Vector128.Create(1.0001f);
                    Vector128<float> zeroVec = Vector128<float>.Zero;
                    Vector128<float> maxValueVec = Vector128.Create(float.MaxValue);

                    Vector128<float> currentMinDistVec = Vector128.Create(currentMinDistance);
                    Vector128<float> rayDx = Vector128.Create(ray.Direction.X);
                    Vector128<float> rayDy = Vector128.Create(ray.Direction.Y);
                    Vector128<float> rayDz = Vector128.Create(ray.Direction.Z);
                    Vector128<float> rayOx = Vector128.Create(ray.Origin.X);
                    Vector128<float> rayOy = Vector128.Create(ray.Origin.Y);
                    Vector128<float> rayOz = Vector128.Create(ray.Origin.Z);

                    for (; i <= end - 4; i += 4)
                    {
                        // Load 4 triangles
                        ref Triangle tri0 = ref triangles[i];
                        ref Triangle tri1 = ref triangles[i + 1];
                        ref Triangle tri2 = ref triangles[i + 2];
                        ref Triangle tri3 = ref triangles[i + 3];

                        // Gather vertex positions for v0, v1, v2
                        Vector128<float> v0x = Vector128.Create(
                            vertices[tri0.Index0].Position.X,
                            vertices[tri1.Index0].Position.X,
                            vertices[tri2.Index0].Position.X,
                            vertices[tri3.Index0].Position.X
                        );
                        Vector128<float> v0y = Vector128.Create(
                            vertices[tri0.Index0].Position.Y,
                            vertices[tri1.Index0].Position.Y,
                            vertices[tri2.Index0].Position.Y,
                            vertices[tri3.Index0].Position.Y
                        );
                        Vector128<float> v0z = Vector128.Create(
                            vertices[tri0.Index0].Position.Z,
                            vertices[tri1.Index0].Position.Z,
                            vertices[tri2.Index0].Position.Z,
                            vertices[tri3.Index0].Position.Z
                        );

                        Vector128<float> v1x = Vector128.Create(
                            vertices[tri0.Index1].Position.X,
                            vertices[tri1.Index1].Position.X,
                            vertices[tri2.Index1].Position.X,
                            vertices[tri3.Index1].Position.X
                        );
                        Vector128<float> v1y = Vector128.Create(
                            vertices[tri0.Index1].Position.Y,
                            vertices[tri1.Index1].Position.Y,
                            vertices[tri2.Index1].Position.Y,
                            vertices[tri3.Index1].Position.Y
                        );
                        Vector128<float> v1z = Vector128.Create(
                            vertices[tri0.Index1].Position.Z,
                            vertices[tri1.Index1].Position.Z,
                            vertices[tri2.Index1].Position.Z,
                            vertices[tri3.Index1].Position.Z
                        );

                        Vector128<float> v2x = Vector128.Create(
                            vertices[tri0.Index2].Position.X,
                            vertices[tri1.Index2].Position.X,
                            vertices[tri2.Index2].Position.X,
                            vertices[tri3.Index2].Position.X
                        );
                        Vector128<float> v2y = Vector128.Create(
                            vertices[tri0.Index2].Position.Y,
                            vertices[tri1.Index2].Position.Y,
                            vertices[tri2.Index2].Position.Y,
                            vertices[tri3.Index2].Position.Y
                        );
                        Vector128<float> v2z = Vector128.Create(
                            vertices[tri0.Index2].Position.Z,
                            vertices[tri1.Index2].Position.Z,
                            vertices[tri2.Index2].Position.Z,
                            vertices[tri3.Index2].Position.Z
                        );

                        // Compute edge vectors
                        Vector128<float> edge1x = v1x - v0x;
                        Vector128<float> edge1y = v1y - v0y;
                        Vector128<float> edge1z = v1z - v0z;
                        Vector128<float> edge2x = v2x - v0x;
                        Vector128<float> edge2y = v2y - v0y;
                        Vector128<float> edge2z = v2z - v0z;

                        // pVec = cross(ray.Direction, edge2)
                        Vector128<float> pVecx = rayDy * edge2z - rayDz * edge2y;
                        Vector128<float> pVecy = rayDz * edge2x - rayDx * edge2z;
                        Vector128<float> pVecz = rayDx * edge2y - rayDy * edge2x;

                        // determinant = dot(edge1, pVec)
                        Vector128<float> det = edge1x * pVecx + edge1y * pVecy + edge1z * pVecz;

                        // Check backface and parallel
                        Vector128<int> backfaceMask = Vector128.LessThan(det, epsilonVec).As<float, int>();
                        Vector128<int> parallelMask = Vector128.LessThanOrEqual(Vector128.Abs(det), epsilonVec).As<float, int>();

                        // Compute inverse determinant
                        Vector128<float> invDet = Vector128.ConditionalSelect(
                            parallelMask.AsSingle(),
                            zeroVec,
                            Vector128<float>.One / det
                        );

                        // tVec = ray.Origin - v0
                        Vector128<float> tVecx = rayOx - v0x;
                        Vector128<float> tVecy = rayOy - v0y;
                        Vector128<float> tVecz = rayOz - v0z;

                        // baryU = dot(tVec, pVec) * invDet
                        Vector128<float> baryU = (tVecx * pVecx + tVecy * pVecy + tVecz * pVecz) * invDet;

                        // Check u bounds
                        Vector128<int> uValidMask = Vector128.GreaterThanOrEqual(baryU, zeroVec).As<float, int>() &
                                                   Vector128.LessThanOrEqual(baryU, Vector128<float>.One).As<float, int>();

                        // qVec = cross(tVec, edge1)
                        Vector128<float> qVecx = tVecy * edge1z - tVecz * edge1y;
                        Vector128<float> qVecy = tVecz * edge1x - tVecx * edge1z;
                        Vector128<float> qVecz = tVecx * edge1y - tVecy * edge1x;

                        // baryV = dot(ray.Direction, qVec) * invDet
                        Vector128<float> baryV = (rayDx * qVecx + rayDy * qVecy + rayDz * qVecz) * invDet;

                        // Check v bounds
                        Vector128<float> sumUV = baryU + baryV;
                        Vector128<int> vValidMask = Vector128.GreaterThanOrEqual(baryV, zeroVec).As<float, int>() &
                                                   Vector128.LessThanOrEqual(sumUV, oneVec).As<float, int>();

                        // hitDistance = dot(edge2, qVec) * invDet
                        Vector128<float> hitDistance = (edge2x * qVecx + edge2y * qVecy + edge2z * qVecz) * invDet;

                        // Check distance validity
                        Vector128<int> distValidMask = Vector128.GreaterThan(hitDistance, epsilonVec).As<float, int>();

                        // Final hit mask
                        Vector128<int> hitMask = ~parallelMask & uValidMask & vValidMask & distValidMask;
                        Vector128<int> closerMask = Vector128.LessThan(hitDistance, currentMinDistVec).As<float, int>();
                        Vector128<int> finalHitMask = hitMask & closerMask;

                        // Update hit distances
                        Vector128<float> finalHitDist = Vector128.ConditionalSelect(
                            finalHitMask.AsSingle(),
                            hitDistance,
                            maxValueVec
                        );

                        // Extract distances and find closest hit
                        finalHitDist.StoreUnsafe(ref MemoryMarshal.GetReference(distances));

                        float minDist = float.PositiveInfinity;
                        int minIndex = -1;
                        for (int j = 0; j < 4; j++)
                        {
                            if (distances[j] < minDist)
                            {
                                minDist = distances[j];
                                minIndex = j;
                            }
                        }
                        hitInfo.TriTests += 4;
                        // Update hit info if we found a closer intersection
                        if (minIndex >= 0 && i + minIndex != ignoredTri && minDist > ray.TMin && minDist < currentMinDistance)
                        {
                            currentMinDistance = minDist;
                            currentMinDistVec = Vector128.Create(minDist);
                            ref Triangle hitTri = ref triangles[i + minIndex];

                            hitInfo.Distance = minDist;
                            hitInfo.PrimIndex = hitTri.PrimIndex;
                            hitInfo.TriIndex = i + minIndex;
                            hitInfo.BarycentricU = baryU.GetElement(minIndex);
                            hitInfo.BarycentricV = baryV.GetElement(minIndex);

                            // Interpolate normal and UV
                            float u = hitInfo.BarycentricU;
                            float v = hitInfo.BarycentricV;
                            float w = 1f - u - v;

                            ref Vertex v0 = ref vertices[hitTri.Index0];
                            ref Vertex v1 = ref vertices[hitTri.Index1];
                            ref Vertex v2 = ref vertices[hitTri.Index2];

                            // Backface handling
                            bool backface = backfaceMask.GetElement(minIndex) != 0;
                            float normalSign = backface ? -1f : 1f;

                            // Normal interpolation
                            Vec3 normal = Vec3.Normalize(
                                v0.Normal * w +
                                v1.Normal * u +
                                v2.Normal * v
                            ) * normalSign;

                            // UV interpolation
                            Vec2 uv = v0.UV * w + v1.UV * u + v2.UV * v;

                            hitInfo.Normal = normal;
                            hitInfo.UV = uv;
                        }
                    }
                }

                // Process remaining triangles with scalar method
                for (; i < end; i++)
                {
                    if (i == ignoredTri) continue;
                    ref Triangle tri = ref triangles[i];
                    hitInfo.TriTests++;
                    if (RayIntersectsTriangle(ray, tri, out float t, out float u, out float v, out bool backface) &&
                        t < currentMinDistance && t > ray.TMin)
                    {
                        currentMinDistance = t;
                        hitInfo.Distance = t;
                        hitInfo.PrimIndex = tri.PrimIndex;
                        hitInfo.BarycentricU = u;
                        hitInfo.BarycentricV = v;
                        hitInfo.TriIndex = i;

                        float w = 1f - u - v;
                        ref Vertex v0 = ref vertices[tri.Index0];
                        ref Vertex v1 = ref vertices[tri.Index1];
                        ref Vertex v2 = ref vertices[tri.Index2];

                        // Normal interpolation
                        Vec3 normal = Vec3.Normalize(
                            v0.Normal * w +
                            v1.Normal * u +
                            v2.Normal * v
                        ) * (backface ? -1f : 1f);

                        // UV interpolation
                        Vec2 uv = v0.UV * w + v1.UV * u + v2.UV * v;

                        hitInfo.Normal = normal;
                        hitInfo.UV = uv;
                    }
                }
            }
            else
            {
                // Internal node: test both children and push nearer first
                int left = node.LeftChild;
                int right = node.RightChild;
                IntersectAABB2(flatBVH[left].Bounds, flatBVH[right].Bounds, ray.Origin, ray.TMin, sign, invDir, hitInfo.Distance,
                    out bool hitLeft, out float tminLeft, out bool hitRight, out float tminRight);
                //bool hitLeft = IntersectAABB(flatBVH[left].Bounds, ray.Origin, sign, invDir,
                //                            hitInfo.Distance, out float tminLeft);
                //bool hitRight = IntersectAABB(flatBVH[right].Bounds, ray.Origin, sign, invDir,
                //                             hitInfo.Distance, out float tminRight);
                hitInfo.BoxTests += 2;
                if (hitLeft && hitRight)
                {
                    if (tminLeft > tminRight)
                    {
                        // Push farther (left) first, then nearer (right)
                        stack[stackPtr++] = left;
                        stack[stackPtr++] = right;
                    }
                    else
                    {
                        stack[stackPtr++] = right;
                        stack[stackPtr++] = left;
                    }
                }
                else if (hitLeft) stack[stackPtr++] = left;
                else if (hitRight) stack[stackPtr++] = right;
            }
        }

        return hitInfo.PrimIndex >= 0;
    }

    // Approximate squared distance to AABB center
    private static float DistanceToAABB(Ray ray, AABB box)
    {
        Vec3 center = (box.Min + box.Max) * 0.5f;
        return (center - ray.Origin).LengthSquared();
    }

    private const float EPSILON = 1e-6f;

    public static void IntersectRayWithTrianglesSIMD(
        Ray ray,
        ReadOnlySpan<Vertex> verts,
        ReadOnlySpan<int> primIndices,
        int startIndex,
        int count,
        ReadOnlySpan<Vertex> vertices,
        ref HitInfo hitInfo)
    {
        int simdWidth = Vector<float>.Count;

        Span<float> tVals = stackalloc float[simdWidth];
        Span<float> uVals = stackalloc float[simdWidth];
        Span<float> vVals = stackalloc float[simdWidth];
        for (int i = startIndex; i < startIndex + count; i += simdWidth)
        {
            int batchSize = Math.Min(simdWidth, startIndex + count - i);


            Vector<float> tHit = Vector.Create(float.MaxValue);
            Vector<int> indices = Vector.Create(-1);

            Vec3 origin = ray.Origin;
            Vec3 dir = ray.Direction;

            for (int j = 0; j < batchSize; j++)
            {
                Vertex v0 = verts[(i + j) * 3 + 0];
                Vertex v1 = verts[(i + j) * 3 + 1];
                Vertex v2 = verts[(i + j) * 3 + 2];

                Vec3 edge1 = v1.Position - v0.Position;
                Vec3 edge2 = v2.Position - v0.Position;

                Vec3 h = Vec3.Cross(dir, edge2);
                float a = Vec3.Dot(edge1, h);
                if (a > -EPSILON && a < EPSILON) continue;

                float f = 1.0f / a;
                Vec3 s = origin - v0.Position;
                float u = f * Vec3.Dot(s, h);
                if (u < 0.0f || u > 1.0f) continue;

                Vec3 q = Vec3.Cross(s, edge1);
                float v = f * Vec3.Dot(dir, q);
                if (v < 0.0f || u + v > 1.0f) continue;

                float t = f * Vec3.Dot(edge2, q);
                if (t > EPSILON && t < hitInfo.Distance)
                {
                    hitInfo.Distance = t;
                    hitInfo.PrimIndex = primIndices[i + j];
                    hitInfo.BarycentricU = u;
                    hitInfo.BarycentricV = v;


                    float w = 1f - u - v;

                    Vec3 n0 = v0.Normal;
                    Vec3 n1 = v1.Normal;
                    Vec3 n2 = v2.Normal;
                    hitInfo.Normal = Vec3.Normalize(n0 * w + n1 * u + n2 * v);

                    Vec2 uv0 = v0.UV;
                    Vec2 uv1 = v1.UV;
                    Vec2 uv2 = v2.UV;
                    hitInfo.UV = uv0 * w + uv1 * u + uv2 * v;
                }
            }
        }
    }

    [SkipLocalsInit]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool RayIntersectsTriangle(
    Ray ray,
    Triangle triangle,
    out float hitDistance,
    out float barycentricU,
    out float barycentricV,
    out bool hitBackface)
    {
        // Load vertices using SIMD registers
        ref Vec3 v0 = ref vertices[triangle.Index0].Position;
        ref Vec3 v1 = ref vertices[triangle.Index1].Position;
        ref Vec3 v2 = ref vertices[triangle.Index2].Position;

        const float EPSILON = 1e-6f;

        if (Vector128.IsHardwareAccelerated)
        {
            // Load all vertices into SIMD registers (interleaved)
            Vector128<float> v0vec = v0.AsVector128();
            Vector128<float> v1vec = v1.AsVector128();
            Vector128<float> v2vec = v2.AsVector128();

            // Calculate edge vectors
            Vector128<float> edge1 = Vector128.Subtract(v1vec, v0vec);
            Vector128<float> edge2 = Vector128.Subtract(v2vec, v0vec);

            // Load ray data
            Vector128<float> dirVec = ray.Direction.AsVector128();
            Vector128<float> originVec = ray.Origin.AsVector128();

            // Compute pVec = cross(ray.Direction, edge2)
            Vector128<float> pVec = CrossSimd(dirVec, edge2);

            // Compute determinant = dot(edge1, pVec)
            float determinant = DotSimd(edge1, pVec);

            // Check for backface and near-parallel ray
            hitBackface = determinant < EPSILON;
            float absDet = MathF.Abs(determinant);
            bool parallel = absDet < EPSILON;

            // Compute inverse determinant if not parallel
            float invDet = parallel ? 0f : 1.0f / determinant;

            // Calculate tVec = ray.Origin - vertex0
            Vector128<float> tVec = Vector128.Subtract(originVec, v0vec);

            // Compute barycentricU = dot(tVec, pVec) * invDet
            barycentricU = DotSimd(tVec, pVec) * invDet;
            bool uValid = barycentricU >= 0f && barycentricU <= 1f;

            // Compute qVec = cross(tVec, edge1)
            Vector128<float> qVec = CrossSimd(tVec, edge1);

            // Compute barycentricV = dot(ray.Direction, qVec) * invDet
            barycentricV = DotSimd(dirVec, qVec) * invDet;
            bool vValid = barycentricV >= 0f && (barycentricU + barycentricV) <= 1.0001f;  // Small epsilon for FP precision

            // Compute hit distance = dot(edge2, qVec) * invDet
            hitDistance = DotSimd(edge2, qVec) * invDet;
            bool distanceValid = hitDistance > ray.TMin;

            // Combine all conditions
            bool hit = !parallel && uValid && vValid && distanceValid;

            // Handle miss cases
            if (!hit)
            {
                hitDistance = 0;
                if (parallel) barycentricU = barycentricV = 0;
                else if (!uValid) barycentricV = 0;
            }

            return hit;
        }
        else
        {
            // Fallback to scalar implementation
            Vec3 edge1 = v1 - v0;
            Vec3 edge2 = v2 - v0;

            Vec3 pVec = Vec3.Cross(ray.Direction, edge2);
            float determinant = Vec3.Dot(edge1, pVec);
            hitBackface = determinant < EPSILON;

            if (MathF.Abs(determinant) < EPSILON)
            {
                hitDistance = 0;
                barycentricU = barycentricV = 0;
                return false;
            }

            float invDet = 1.0f / determinant;
            Vec3 tVec = ray.Origin - v0;

            barycentricU = Vec3.Dot(tVec, pVec) * invDet;
            if (barycentricU < 0 || barycentricU > 1)
            {
                hitDistance = 0;
                barycentricV = 0;
                return false;
            }

            Vec3 qVec = Vec3.Cross(tVec, edge1);
            barycentricV = Vec3.Dot(ray.Direction, qVec) * invDet;
            if (barycentricV < 0 || barycentricU + barycentricV > 1)
            {
                hitDistance = 0;
                return false;
            }

            hitDistance = Vec3.Dot(edge2, qVec) * invDet;
            return hitDistance > ray.TMin;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector128<float> CrossSimd(Vector128<float> a, Vector128<float> b)
    {
        // Shuffle vectors for cross product components
        Vector128<float> a_yzx = Vector128.Shuffle(a, Vector128.Create(1, 2, 0, 3));
        Vector128<float> b_zxy = Vector128.Shuffle(b, Vector128.Create(2, 0, 1, 3));
        Vector128<float> a_zxy = Vector128.Shuffle(a, Vector128.Create(2, 0, 1, 3));
        Vector128<float> b_yzx = Vector128.Shuffle(b, Vector128.Create(1, 2, 0, 3));

        // cross = (a_yzx * b_zxy) - (a_zxy * b_yzx)
        return Vector128.Subtract(
            Vector128.Multiply(a_yzx, b_zxy),
            Vector128.Multiply(a_zxy, b_yzx)
        );
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float DotSimd(Vector128<float> a, Vector128<float> b)
    {
        Vector128<float> product = Vector128.Multiply(a, b);

        // Horizontal addition: sum all components
        if (Sse3.IsSupported)
        {
            Vector128<float> sum = Sse3.HorizontalAdd(product, product);
            sum = Sse3.HorizontalAdd(sum, sum);
            return sum.GetElement(0);
        }
        else
        {
            // Fallback for non-SSE3 hardware
            return product.GetElement(0) + product.GetElement(1) + product.GetElement(2);
        }
    }

    public static unsafe void Intersections(
        in Vec3 rayOrigin,
        in Vec3 rayInvDirection,
        ReadOnlySpan<AABB> boxes,
        ReadOnlySpan<bool> signs,
        Span<Vector256<float>> ts)
    {
        Span<Vector256<float>> origin = stackalloc Vector256<float>[3];
        Span<Vector256<float>> dir_inv = stackalloc Vector256<float>[3];

        for (int d = 0; d < origin.Length; ++d)
        {
            origin[d] = Vector256.Create(rayOrigin[d]);
            dir_inv[d] = Vector256.Create(rayInvDirection[d]);
        }

        for (int i = 0; i < boxes.Length; ++i)
        {
            ref readonly var box = ref boxes[i];
            Vector256<float> tmin = Vector256<float>.Zero;
            Vector256<float> tmax = ts[i];

            for (int d = 0; d < 3; ++d)
            {
                float bmin_scalar = (signs[d] ? box.Max : box.Min)[d];
                float bmax_scalar = (signs[d] ? box.Min : box.Max)[d];


                Vector256<float> bmin = Vector256.Create(bmin_scalar);
                Vector256<float> bmax = Vector256.Create(bmax_scalar);

                Vector256<float> dmin = Vector256.Multiply(Vector256.Subtract(bmin, origin[d]), dir_inv[d]);
                Vector256<float> dmax = Vector256.Multiply(Vector256.Subtract(bmax, origin[d]), dir_inv[d]);

                tmin = Vector256.MaxNative(tmin, dmin);
                tmax = Vector256.MinNative(tmax, dmax);
            }

            Vector256<float> mask = Avx.Compare(tmin, tmax, FloatComparisonMode.OrderedLessThanOrEqualSignaling);
            ts[i] = Avx.BlendVariable(ts[i], tmin, mask);
        }
    }
    [SkipLocalsInit]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void IntersectAABB2(
    AABB leftBox, AABB rightBox,
    Vec3 rayOrigin, float minDist, Vector3i raySign, Vec3 rayInvDir, float maxDistance,
    out bool hitLeft, out float tminLeft,
    out bool hitRight, out float tminRight)
    {
        // Fallback to scalar if AVX2 not available
        if (!Vector256.IsHardwareAccelerated)
        {
            hitLeft = IntersectAABB(leftBox, rayOrigin, raySign, rayInvDir, maxDistance, out tminLeft);
            hitRight = IntersectAABB(rightBox, rayOrigin, raySign, rayInvDir, maxDistance, out tminRight);
            return;
        }

        // Precompute ray sign masks (true where raySign == 0)
        Vector256<float> maskX = Vector256.Create(raySign.X == 0 ? -1 : 0).As<int, float>();
        Vector256<float> maskY = Vector256.Create(raySign.Y == 0 ? -1 : 0).As<int, float>();
        Vector256<float> maskZ = Vector256.Create(raySign.Z == 0 ? -1 : 0).As<int, float>();

        // Load all box data in interleaved format (left, right)
        Vector256<float> minX = Vector256.Create(leftBox.Min.X, rightBox.Min.X, 0, 0, 0, 0, 0, 0);
        Vector256<float> maxX = Vector256.Create(leftBox.Max.X, rightBox.Max.X, 0, 0, 0, 0, 0, 0);
        Vector256<float> minY = Vector256.Create(leftBox.Min.Y, rightBox.Min.Y, 0, 0, 0, 0, 0, 0);
        Vector256<float> maxY = Vector256.Create(leftBox.Max.Y, rightBox.Max.Y, 0, 0, 0, 0, 0, 0);
        Vector256<float> minZ = Vector256.Create(leftBox.Min.Z, rightBox.Min.Z, 0, 0, 0, 0, 0, 0);
        Vector256<float> maxZ = Vector256.Create(leftBox.Max.Z, rightBox.Max.Z, 0, 0, 0, 0, 0, 0);

        // Select near/far planes using masks
        Vector256<float> nearX = Vector256.ConditionalSelect(maskX, minX, maxX);
        Vector256<float> farX = Vector256.ConditionalSelect(maskX, maxX, minX);
        Vector256<float> nearY = Vector256.ConditionalSelect(maskY, minY, maxY);
        Vector256<float> farY = Vector256.ConditionalSelect(maskY, maxY, minY);
        Vector256<float> nearZ = Vector256.ConditionalSelect(maskZ, minZ, maxZ);
        Vector256<float> farZ = Vector256.ConditionalSelect(maskZ, maxZ, minZ);

        // Broadcast ray data to all lanes
        Vector256<float> originX = Vector256.Create(rayOrigin.X);
        Vector256<float> originY = Vector256.Create(rayOrigin.Y);
        Vector256<float> originZ = Vector256.Create(rayOrigin.Z);
        Vector256<float> invDirX = Vector256.Create(rayInvDir.X);
        Vector256<float> invDirY = Vector256.Create(rayInvDir.Y);
        Vector256<float> invDirZ = Vector256.Create(rayInvDir.Z);

        // Calculate t values
        Vector256<float> tNearX = Vector256.Multiply(Vector256.Subtract(nearX, originX), invDirX);
        Vector256<float> tFarX = Vector256.Multiply(Vector256.Subtract(farX, originX), invDirX);
        Vector256<float> tNearY = Vector256.Multiply(Vector256.Subtract(nearY, originY), invDirY);
        Vector256<float> tFarY = Vector256.Multiply(Vector256.Subtract(farY, originY), invDirY);
        Vector256<float> tNearZ = Vector256.Multiply(Vector256.Subtract(nearZ, originZ), invDirZ);
        Vector256<float> tFarZ = Vector256.Multiply(Vector256.Subtract(farZ, originZ), invDirZ);

        // Compute min/max across axes
        Vector256<float> tMin = Vector256.MaxNative(Vector256.MaxNative(tNearX, tNearY), tNearZ);
        Vector256<float> tMax = Vector256.MinNative(Vector256.MinNative(tFarX, tFarY), tFarZ);

        // Extract results for both boxes
        tminLeft = tMin.GetElement(0);
        tminRight = tMin.GetElement(1);
        float tmaxLeft = tMax.GetElement(0);
        float tmaxRight = tMax.GetElement(1);

        // Check intersection conditions
        hitLeft = (tminLeft <= tmaxLeft) && (tminLeft < maxDistance) && (tmaxLeft > minDist);
        hitRight = (tminRight <= tmaxRight) && (tminRight < maxDistance) && (tmaxRight > minDist);
    }
    [SkipLocalsInit]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IntersectAABB(AABB box, Vec3 rayOrigin, Vector3i raySign, Vec3 rayInvDir, float maxDistance, out float dist)
    {
        dist = float.PositiveInfinity;

        // If SSE4.1 is not supported, fall back to the original scalar method
        if (Vector128.IsHardwareAccelerated)
        {
            // Load box min and max into SIMD registers
            Vector128<float> minVec = box.Min.AsVector128Unsafe();
            Vector128<float> maxVec = box.Max.AsVector128Unsafe();

            // Create mask from raySign (true where raySign != 0)
            Vector128<int> raySignVec = Vector128.Create(raySign.X, raySign.Y, raySign.Z, 0);
            Vector128<int> zero = Vector128<int>.Zero;
            Vector128<int> nonZeroMask = Vector128.Equals(raySignVec, zero);

            // Blend min/max vectors based on raySign using ConditionalSelect
            Vector128<float> near = Vector128.ConditionalSelect(
                nonZeroMask.AsSingle(),
                minVec,
                maxVec
            );

            Vector128<float> far = Vector128.ConditionalSelect(
                nonZeroMask.AsSingle(),
                maxVec,
                minVec
            );
            //Vector128<float> near = Vector128.ConditionalSelect(mask.AsSingle(), minVec, maxVec);
            //Vector128<float> far = Vector128.ConditionalSelect(mask.AsSingle(), maxVec, minVec);

            // Load ray origin and inverse direction
            Vector128<float> rayOriginVec = rayOrigin.AsVector128Unsafe();
            Vector128<float> rayInvDirVec = rayInvDir.AsVector128Unsafe();

            // Calculate t_near and t_far for all axes
            Vector128<float> t_near = Vector128.Multiply(Vector128.Subtract(near, rayOriginVec), rayInvDirVec);
            Vector128<float> t_far = Vector128.Multiply(Vector128.Subtract(far, rayOriginVec), rayInvDirVec);



            // Compute overall tmin and tmax
            float tmin = float.Max(float.Max(t_near[0], t_near[1]), t_near[2]);
            float tmax = float.Min(float.Min(t_far[0], t_far[1]), t_far[2]);

            // Early exit if no intersection
            if (tmin > tmax)
                return false;

            dist = tmin;
            return tmin < maxDistance && tmax > 0;
        }
        else
        {
            // Original scalar implementation
            float tmin = ((raySign.X == 0 ? box.Min.X : box.Max.X) - rayOrigin.X) * rayInvDir.X;
            float tmax = ((raySign.X == 0 ? box.Max.X : box.Min.X) - rayOrigin.X) * rayInvDir.X;

            float tymin = ((raySign.Y == 0 ? box.Min.Y : box.Max.Y) - rayOrigin.Y) * rayInvDir.Y;
            float tymax = ((raySign.Y == 0 ? box.Max.Y : box.Min.Y) - rayOrigin.Y) * rayInvDir.Y;

            if (tmin > tymax || tymin > tmax)
                return false;

            tmin = MathF.Max(tmin, tymin);
            tmax = MathF.Min(tmax, tymax);

            float tzmin = ((raySign.Z == 0 ? box.Min.Z : box.Max.Z) - rayOrigin.Z) * rayInvDir.Z;
            float tzmax = ((raySign.Z == 0 ? box.Max.Z : box.Min.Z) - rayOrigin.Z) * rayInvDir.Z;

            if (tmin > tzmax || tzmin > tmax)
            {
                return false;
            }

            tmin = MathF.Max(tmin, tzmin);
            tmax = MathF.Min(tmax, tzmax);
            dist = tmin;
            return tmin < maxDistance && tmax > 0 && tmin <= tmax;
        }
    }
}
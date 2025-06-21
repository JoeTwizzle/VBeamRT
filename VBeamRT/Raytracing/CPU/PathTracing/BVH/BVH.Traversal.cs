using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;
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

	public bool IntersectClosest(Ray ray, out HitInfo hitInfo)
	{
		hitInfo = new HitInfo { Distance = float.PositiveInfinity, TriangleIndex = -1 };

		Vec3 invDir = new Vec3(1f) / ray.Direction;
		Vector3i sign = new Vector3i(
			ray.Direction.X < 0 ? 1 : 0,
			ray.Direction.Y < 0 ? 1 : 0,
			ray.Direction.Z < 0 ? 1 : 0);

		const int MaxStackDepth = 64;
		Span<int> stack = stackalloc int[MaxStackDepth];
		int stackPtr = 0;
		stack[stackPtr++] = 0;  // root node index

		while (stackPtr > 0)
		{
			var node = flatBVH[stack[--stackPtr]];

			// Skip if ray misses node AABB or farther than current closest hit
			if (!IntersectAABB(node.Bounds, ray.Origin, sign, invDir, hitInfo.Distance, out _))
				continue;

			// Leaf node test: TriangleCount > 0 means leaf
			if (node.TriangleCount > 0)
			{
				int start = node.TriangleStart;
				int end = start + node.TriangleCount;
				for (int i = start; i < end; i++)
				{
					var tri = triangles[i];

					if (RayIntersectsTriangle(ray, tri,
						out float t, out float u, out float v, out bool backface) &&
						t < hitInfo.Distance)
					{
						hitInfo.Distance = t;
						hitInfo.TriangleIndex = i;
						hitInfo.BarycentricU = u;
						hitInfo.BarycentricV = v;

						float w = 1f - u - v;

						// Interpolate normal and flip if backface
						Vec3 n0 = vertices[tri.Index0].Normal;
						Vec3 n1 = vertices[tri.Index1].Normal;
						Vec3 n2 = vertices[tri.Index2].Normal;
						hitInfo.Normal = Vec3.Normalize(n0 * w + n1 * u + n2 * v)
										 * (backface ? -1f : 1f);

						// Interpolate UV
						Vec2 uv0 = vertices[tri.Index0].UV;
						Vec2 uv1 = vertices[tri.Index1].UV;
						Vec2 uv2 = vertices[tri.Index2].UV;
						hitInfo.UV = uv0 * w + uv1 * u + uv2 * v;
					}
				}
			}
			else
			{
				// Internal node: test both children and push nearer first
				int left = node.LeftChild;
				int right = node.RightChild;

				bool hitLeft = IntersectAABB(flatBVH[left].Bounds, ray.Origin, sign, invDir,
											hitInfo.Distance, out float tminLeft);
				bool hitRight = IntersectAABB(flatBVH[right].Bounds, ray.Origin, sign, invDir,
											 hitInfo.Distance, out float tminRight);

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

		return hitInfo.TriangleIndex >= 0;
	}

	// Approximate squared distance to AABB center
	private static float DistanceToAABB(Ray ray, AABB box)
	{
		Vec3 center = (box.Min + box.Max) * 0.5f;
		return (center - ray.Origin).LengthSquared();
	}

	private bool RayIntersectsTriangle(
	Ray ray,
	Triangle triangle,
	out float hitDistance,
	out float barycentricU,
	out float barycentricV,
	out bool hitBackface)
	{
		Vec3 vertex0 = vertices[triangle.Index0].Position;
		Vec3 vertex1 = vertices[triangle.Index1].Position;
		Vec3 vertex2 = vertices[triangle.Index2].Position;

		const float EPSILON = 1e-6f;

		Vec3 edge1 = vertex1 - vertex0;
		Vec3 edge2 = vertex2 - vertex0;

		Vec3 pVec = Vec3.Cross(ray.Direction, edge2);
		float determinant = Vec3.Dot(edge1, pVec);
		hitBackface = determinant < EPSILON;
		if (MathF.Abs(determinant) < EPSILON)
		{
			hitDistance = 0;
			barycentricU = barycentricV = 0;
			return false; // Ray is parallel to triangle
		}

		float invDet = 1.0f / determinant;

		Vec3 tVec = ray.Origin - vertex0;
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
		return hitDistance > EPSILON;
	}

	public static bool IntersectAABB(AABB box, Vec3 rayOrigin, Vector3i raySign, Vec3 rayInvDir, float maxDistance, out float dist)
	{
		dist = float.PositiveInfinity;
		// X axis
		float tmin = ((raySign[0] == 0 ? box.Min.X : box.Max.X) - rayOrigin.X) * rayInvDir.X;
		float tmax = ((raySign[0] == 0 ? box.Max.X : box.Min.X) - rayOrigin.X) * rayInvDir.X;

		// Y axis
		float tymin = ((raySign[1] == 0 ? box.Min.Y : box.Max.Y) - rayOrigin.Y) * rayInvDir.Y;
		float tymax = ((raySign[1] == 0 ? box.Max.Y : box.Min.Y) - rayOrigin.Y) * rayInvDir.Y;

		// Early exit if no overlap in X/Y
		if (tmin > tymax || tymin > tmax)
			return false;

		// Clamp to overlapping range
		tmin = MathF.Max(tmin, tymin);
		tmax = MathF.Min(tmax, tymax);

		// Z axis
		float tzmin = ((raySign[2] == 0 ? box.Min.Z : box.Max.Z) - rayOrigin.Z) * rayInvDir.Z;
		float tzmax = ((raySign[2] == 0 ? box.Max.Z : box.Min.Z) - rayOrigin.Z) * rayInvDir.Z;

		// Early exit if no overlap in Z
		if (tmin > tzmax || tzmin > tmax)
			return false;

		// Clamp to overlapping range
		tmin = MathF.Max(tmin, tzmin);
		tmax = MathF.Min(tmax, tzmax);
		dist = tmin;
		// Final intersection check
		return tmin < maxDistance && tmax > 0 && tmin <= tmax;
	}
}
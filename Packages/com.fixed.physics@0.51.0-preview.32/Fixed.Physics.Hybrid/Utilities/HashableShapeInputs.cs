using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Fixed.Mathematics;
using UnityEngine;
using Hash128 = Unity.Entities.Hash128;

namespace Fixed.Physics.Authoring
{
    /// <summary>
    /// Structure for hashing input data used for convex and mesh colliders.
    /// An instance represents relevant inputs from a single mesh renderer or skinned mesh renderer contributing to the collider's geometry.
    /// </summary>
    struct HashableShapeInputs
    {
        // solver generally not expected to be precise under 0.01 units
        //TODO
        internal static readonly sfloat k_DefaultLinearPrecision = (sfloat)0.001f;

        readonly Aabb Bounds;

        public readonly int MeshKey; // TODO: currently assumes each mesh asset will have consistent data between hashes; should instead hash vertices/triangles
        public readonly float4x4 BodyFromShape;
        public readonly int IncludedIndicesStartIndex;
        public readonly int IncludedIndicesCount;
        public readonly int BlendShapeWeightsStartIndex;
        public readonly int BlendShapeWeightsCount;

        public static HashableShapeInputs FromMesh(UnityEngine.Mesh mesh, float4x4 leafToBody)
        {
            using (var includedIndices = new NativeArray<int>(0, Allocator.TempJob))
            using (var allIncludedIndices = new NativeList<int>(0, Allocator.TempJob))
            using (var blendShapeWeights = new NativeArray<sfloat>(0, Allocator.TempJob))
            using (var allBlendShapeWeights = new NativeList<sfloat>(0, Allocator.TempJob))
                return FromSkinnedMesh(mesh, leafToBody, includedIndices, allIncludedIndices, blendShapeWeights, allBlendShapeWeights);
        }

        public static HashableShapeInputs FromSkinnedMesh(
            UnityEngine.Mesh mesh, float4x4 leafToBody,
            NativeArray<int> includedIndices, NativeList<int> allIncludedIndices,
            NativeArray<sfloat> blendShapeWeights, NativeList<sfloat> allBlendShapeWeights
        )
        {
            using (var result = new NativeArray<HashableShapeInputs>(1, Allocator.TempJob))
            {
                int meshKey = default;
                Bounds bounds = default;
                NativeArray<int> tmpIndices;
                NativeList<int> tmpAllIndices;
                NativeArray<sfloat> tmpBlendWeights;
                NativeList<sfloat> tmpAllBlendWeights;
                if (mesh == null)
                {
                    tmpIndices = new NativeArray<int>(0, Allocator.TempJob);
                    tmpAllIndices = new NativeList<int>(0, Allocator.TempJob);
                    tmpBlendWeights = new NativeArray<sfloat>(0, Allocator.TempJob);
                    tmpAllBlendWeights = new NativeList<sfloat>(0, Allocator.TempJob);
                }
                else
                {
                    meshKey = mesh.GetInstanceID();
                    bounds = mesh.bounds;
                    tmpIndices = new NativeArray<int>(includedIndices, Allocator.TempJob);
                    tmpAllIndices = new NativeList<int>(allIncludedIndices.Length, Allocator.TempJob);
                    tmpAllIndices.AddRangeNoResize(allIncludedIndices);
                    tmpBlendWeights = new NativeArray<sfloat>(blendShapeWeights, Allocator.TempJob);
                    tmpAllBlendWeights = new NativeList<sfloat>(allBlendShapeWeights.Length, Allocator.TempJob);
                    tmpAllBlendWeights.AddRangeNoResize(allBlendShapeWeights);
                }
                new HashableShapeInputsFromMeshJob
                {
                    Result = result,
                    MeshKey = meshKey,
                    Bounds = bounds,
                    LeafToBody = leafToBody,
                    IncludedIndices = tmpIndices,
                    AllIncludedIndices = tmpAllIndices,
                    BlendShapeWeights = tmpBlendWeights,
                    AllBlendShapeWeights = tmpAllBlendWeights
                }.Run();

                if (includedIndices.Length > 0)
                {
                    allIncludedIndices.ResizeUninitialized(tmpAllIndices.Length);
                    allIncludedIndices.AddRange(tmpAllIndices);
                }

                if (blendShapeWeights.Length > 0)
                {
                    allBlendShapeWeights.ResizeUninitialized(tmpAllBlendWeights.Length);
                    allBlendShapeWeights.AddRange(tmpAllBlendWeights);
                }

                tmpIndices.Dispose();
                tmpAllIndices.Dispose();
                tmpBlendWeights.Dispose();
                tmpAllBlendWeights.Dispose();

                return result[0];
            }
        }

        [BurstCompile]
        struct HashableShapeInputsFromMeshJob : IJob
        {
            public NativeArray<HashableShapeInputs> Result;

            public int MeshKey;
            public Bounds Bounds;
            public float4x4 LeafToBody;
            [ReadOnly] public NativeArray<int> IncludedIndices;
            public NativeList<int> AllIncludedIndices;
            [ReadOnly] public NativeArray<sfloat> BlendShapeWeights;
            public NativeList<sfloat> AllBlendShapeWeights;

            public void Execute() => Result[0] = new HashableShapeInputs(
                MeshKey, Bounds, LeafToBody, IncludedIndices, AllIncludedIndices, BlendShapeWeights, AllBlendShapeWeights, k_DefaultLinearPrecision
            );
        }

        HashableShapeInputs(
            int meshKey, Bounds bounds, float4x4 leafToBody,
            sfloat linearPrecision/* = k_DefaultLinearPrecision*/
        )
        {
            Bounds = new Aabb { Min = bounds.min, Max = bounds.max };

            GetQuantizedTransformations(
                leafToBody, Bounds,
                out var translation, out var orientation, out var scale, out var shear,
                linearPrecision
            );

            MeshKey = meshKey;
            BodyFromShape = math.mul(
                new float4x4(new RigidTransform(orientation, translation)),
                math.mul(new float4x4(shear, sfloat.Zero), float4x4.Scale(scale))
            );
            IncludedIndicesStartIndex = BlendShapeWeightsStartIndex = -1;
            IncludedIndicesCount = BlendShapeWeightsCount = 0;
        }

        HashableShapeInputs(
            int meshKey, Bounds bounds, float4x4 leafToBody,
            NativeArray<int> includedIndices, NativeList<int> allIncludedIndices,
            NativeArray<sfloat> blendShapeWeights, NativeList<sfloat> allBlendShapeWeights,
            sfloat linearPrecision/* = k_DefaultLinearPrecision*/
        )
        {
            this = new HashableShapeInputs(meshKey, bounds, leafToBody, linearPrecision);

            if (allIncludedIndices.IsCreated)
            {
                IncludedIndicesStartIndex = allIncludedIndices.Length;
                IncludedIndicesCount = includedIndices.Length;
                allIncludedIndices.AddRange(includedIndices);
            }

            if (allBlendShapeWeights.IsCreated)
            {
                BlendShapeWeightsStartIndex = allBlendShapeWeights.Length;
                BlendShapeWeightsCount = blendShapeWeights.Length;
                allBlendShapeWeights.AddRange(blendShapeWeights);
            }
        }

        internal static void GetQuantizedTransformations(
            float4x4 leafToBody, Aabb bounds, out float4x4 transformations,
            sfloat linearPrecision/* = k_DefaultLinearPrecision*/
        )
        {
            GetQuantizedTransformations(
                leafToBody, bounds, out var t, out var r, out var s, out var sh, linearPrecision
            );
            transformations = math.mul(new float4x4(sh, sfloat.Zero), float4x4.TRS(t, r, s));
        }

        static void GetQuantizedTransformations(
            float4x4 leafToBody, Aabb bounds,
            out float3 translation, out quaternion orientationQ, out float3 scale, out float3x3 shear,
            sfloat linearPrecision/* = k_DefaultLinearPrecision*/
        )
        {
            translation = RoundToNearest(leafToBody.c3.xyz, linearPrecision);

            var farthestPoint =
                math.abs(math.lengthsq(bounds.Max) > math.lengthsq(bounds.Min) ? bounds.Max : bounds.Min);

            // round scale using precision inversely proportional to mesh size along largest axis
            // (i.e. amount to scale farthest point one unit of linear precision)
            var scalePrecision = linearPrecision / math.max(math.cmax(farthestPoint), math.FLT_MIN_NORMAL);
            scale = RoundToNearest(leafToBody.DecomposeScale(), scalePrecision);
            if (math.determinant(leafToBody) < sfloat.Zero)
                scale.x *= -sfloat.One;

            shear = new float3x3(
                leafToBody.c0.xyz / math.max(math.abs(scale.x), math.FLT_MIN_NORMAL) * math.sign(scale.x),
                leafToBody.c1.xyz / math.max(math.abs(scale.y), math.FLT_MIN_NORMAL) * math.sign(scale.y),
                leafToBody.c2.xyz / math.max(math.abs(scale.z), math.FLT_MIN_NORMAL) * math.sign(scale.z)
            );
            var orientation = float3x3.LookRotationSafe(shear.c2, shear.c1);

            // if shear is very nearly identity, hash it as identity
            // TODO: quantize shear
            shear = math.mul(shear, math.inverse(orientation));
            if (!PhysicsShapeExtensions.HasShear(new float4x4(shear, sfloat.Zero)))
                shear = float3x3.identity;

            // round orientation using precision inversely proportional to scaled mesh size
            // (i.e. radians to rotate farthest scaled point one unit of linear precision)
            var angularPrecision = math.min(linearPrecision / math.length(farthestPoint * scale), math.PI);
            var axisPrecision = math.min(math.cos(angularPrecision), math.sin(angularPrecision));
            orientation.c0 = math.normalize(RoundToNearest(orientation.c0, axisPrecision));
            orientation.c1 = math.normalize(RoundToNearest(orientation.c1, axisPrecision));
            orientation.c2 = math.normalize(RoundToNearest(orientation.c2, axisPrecision));

            translation = Sanitize(translation);
            orientationQ = new quaternion(Sanitize(orientation));
            scale = Sanitize(scale);
            shear = Sanitize(shear);
        }

        static float3 RoundToNearest(float3 value, float3 intervalPerAxis) =>
            math.round(value / intervalPerAxis) * intervalPerAxis;

        // prevents hashing negative zero
        static float3 Sanitize(float3 value)
        {
            for (var i = 0; i < 3; ++i)
            {
                if (value[i] == sfloat.Zero)
                    value[i] = sfloat.Zero;
            }
            return value;
        }

        static float3x3 Sanitize(float3x3 value)
        {
            for (var i = 0; i < 3; ++i)
                value[i] = Sanitize(value[i]);
            return value;
        }

        public static unsafe Hash128 GetHash128(
            uint uniqueIdentifier,
            ConvexHullGenerationParameters hullGenerationParameters,
            Material material,
            CollisionFilter filter,
            float4x4 bakeFromShape,
            NativeArray<HashableShapeInputs> inputs,
            NativeArray<int> allIncludedIndices,
            NativeArray<sfloat> allBlendShapeWeights,
            sfloat linearPrecision/* = k_DefaultLinearPrecision*/
        )
        {
            var numInputs = inputs.IsCreated ? inputs.Length : 0;

            // quantize shape-level transforms
            var bounds = new Aabb { Min = sfloat.MaxValue, Max = sfloat.MinValue };
            for (int i = 0; i < numInputs; i++)
            {
                var d = inputs[i];
                bounds.Include(math.mul(d.BodyFromShape, new float4(d.Bounds.Min, sfloat.One)).xyz);
                bounds.Include(math.mul(d.BodyFromShape, new float4(d.Bounds.Max, sfloat.One)).xyz);
            }
            GetQuantizedTransformations(bakeFromShape, bounds, out var bakeMatrix, linearPrecision);

            // bakeFromShape only contains scale/shear information, so only the inner 3x3 needs to contribute to the hash
            var scaleShear = new float3x3(bakeMatrix.c0.xyz, bakeMatrix.c1.xyz, bakeMatrix.c2.xyz);

            var bytes = new NativeList<byte>(Allocator.Temp);
            bytes.Append(ref uniqueIdentifier);
            bytes.Append(ref hullGenerationParameters);
            bytes.Append(ref material);
            bytes.Append(ref filter);
            bytes.Append(ref scaleShear);
            bytes.Append(inputs);
            bytes.Append(allIncludedIndices);
            bytes.Append(allBlendShapeWeights);
            return HashUtility.Hash128(bytes.GetUnsafeReadOnlyPtr(), bytes.Length);
        }
    }
}
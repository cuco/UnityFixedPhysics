#if LEGACY_PHYSICS
using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using UnityEngine;
using LegacyPhysics = UnityEngine.Physics;
using LegacyCollider = UnityEngine.Collider;
using LegacyBox = UnityEngine.BoxCollider;
using LegacyCapsule = UnityEngine.CapsuleCollider;
using LegacyMesh = UnityEngine.MeshCollider;
using LegacySphere = UnityEngine.SphereCollider;

namespace Fixed.Physics.Authoring
{
    public abstract class BaseLegacyColliderConversionSystem<T> : BaseShapeConversionSystem<T> where T : LegacyCollider
    {
        static readonly IReadOnlyDictionary<PhysicMaterialCombine, Material.CombinePolicy> k_MaterialCombineLookup =
            new Dictionary<PhysicMaterialCombine, Material.CombinePolicy>
        {
            { PhysicMaterialCombine.Average, Material.CombinePolicy.ArithmeticMean },
            { PhysicMaterialCombine.Maximum, Material.CombinePolicy.Maximum },
            { PhysicMaterialCombine.Minimum, Material.CombinePolicy.Minimum }
        };

        static PhysicMaterial DefaultMaterial
        {
            get
            {
                if (s_DefaultMaterial == null)
                    s_DefaultMaterial = new PhysicMaterial { hideFlags = HideFlags.DontSave };
                return s_DefaultMaterial;
            }
        }
        static PhysicMaterial s_DefaultMaterial;

        Material ProduceMaterial(LegacyCollider collider)
        {
            // n.b. need to manually opt in to collision events with legacy colliders if desired
            var material = new Material();
            if (collider.isTrigger)
            {
                material.CollisionResponse = CollisionResponsePolicy.RaiseTriggerEvents;
            }

            var legacyMaterial = collider.sharedMaterial;
            if (legacyMaterial == null)
                legacyMaterial = DefaultMaterial;
            else
                DeclareAssetDependency(collider.gameObject, legacyMaterial);

            material.Friction = (fp)legacyMaterial.dynamicFriction;
            if (k_MaterialCombineLookup.TryGetValue(legacyMaterial.frictionCombine, out var combine))
                material.FrictionCombinePolicy = combine;
            else
                Debug.LogWarning(
                    $"{collider.name} uses {legacyMaterial.name}, which specifies non-convertible mode {legacyMaterial.frictionCombine} for {nameof(PhysicMaterial.frictionCombine)}.",
                    collider
                );

            material.Restitution = (fp)legacyMaterial.bounciness;
            if (k_MaterialCombineLookup.TryGetValue(legacyMaterial.bounceCombine, out combine))
                material.RestitutionCombinePolicy = combine;
            else
                Debug.LogWarning(
                    $"{collider.name} uses {legacyMaterial.name}, which specifies non-convertible mode {legacyMaterial.bounceCombine} for {nameof(PhysicMaterial.bounceCombine)}.",
                    collider
                );

            return material;
        }

        internal override ShapeComputationData GenerateComputationData(
            T shape, ColliderInstance colliderInstance,
            NativeList<fp3> allConvexHullPoints, NativeList<fp3> allMeshVertices,
            NativeList<int3> allMeshTriangles, HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            return new ShapeComputationData
            {
                Instance = colliderInstance,
                Material = ProduceMaterial(shape),
                CollisionFilter = ProduceCollisionFilter(shape)
            };
        }

        protected static CollisionFilter ProduceCollisionFilter(LegacyCollider collider)
        {
            var layer = collider.gameObject.layer;
            var filter = new CollisionFilter { BelongsTo = (uint)(1 << collider.gameObject.layer) };
            for (var i = 0; i < 32; ++i)
                filter.CollidesWith |= (uint)(LegacyPhysics.GetIgnoreLayerCollision(layer, i) ? 0 : 1 << i);
            return filter;
        }

        protected override bool ShouldConvertShape(T shape) => shape.enabled;
        protected override GameObject GetPrimaryBody(T shape) => shape.GetPrimaryBody();
    }

    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("adamm", 3)]
    public sealed class LegacyBoxColliderConversionSystem : BaseLegacyColliderConversionSystem<LegacyBox>
    {
        internal override ShapeComputationData GenerateComputationData(
            LegacyBox shape, ColliderInstance colliderInstance,
            NativeList<fp3> allConvexHullPoints, NativeList<fp3> allMeshVertices, NativeList<int3> allMeshTriangles,
            HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            var res = base.GenerateComputationData(shape, colliderInstance, allConvexHullPoints, allMeshVertices, allMeshTriangles, meshAssets);
            res.ShapeType = ShapeType.Box;

            var shapeLocalToWorld = shape.transform.localToWorldMatrix;
            var worldCenter = fpmath.mul(shapeLocalToWorld, new fp4(shape.center, fp.one));
            var transformRotation  = shape.transform.rotation;
            var rigidBodyTransform = Math.DecomposeRigidBodyTransform(shapeLocalToWorld);
            var shapeFromWorld = fpmath.inverse(new fp4x4(rigidBodyTransform));

            var orientationFixup = fpmath.inverse(fpmath.mul(fpmath.inverse(transformRotation), rigidBodyTransform.rot));

            var geometry = new BoxGeometry
            {
                Center = fpmath.mul(shapeFromWorld, worldCenter).xyz,
                Orientation = orientationFixup
            };

            var linearScale = fp4x4.TRS(fp3.zero, fpmath.inverse(orientationFixup), shape.transform.lossyScale).DecomposeScale();
            geometry.Size = fpmath.abs(shape.size * linearScale);

            geometry.BevelRadius = fpmath.min(ConvexHullGenerationParameters.Default.BevelRadius, fpmath.cmin(geometry.Size) * fp.half);

            res.BoxProperties = geometry;

            return res;
        }
    }

    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("adamm", 3)]
    public sealed class LegacyCapsuleColliderConversionSystem : BaseLegacyColliderConversionSystem<LegacyCapsule>
    {
        internal override ShapeComputationData GenerateComputationData(
            LegacyCapsule shape, ColliderInstance colliderInstance,
            NativeList<fp3> allConvexHullPoints, NativeList<fp3> allMeshVertices, NativeList<int3> allMeshTriangles,
            HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            var res = base.GenerateComputationData(shape, colliderInstance, allConvexHullPoints, allMeshVertices, allMeshTriangles, meshAssets);

            res.ShapeType = ShapeType.Capsule;

            var shapeLocalToWorld   = (fp4x4)shape.transform.localToWorldMatrix;
            var transformRotation   = shape.transform.rotation;
            var rigidBodyTransform  = Math.DecomposeRigidBodyTransform(shapeLocalToWorld);
            var orientationFixup    = fpmath.inverse(fpmath.mul(fpmath.inverse(transformRotation), rigidBodyTransform.rot));
            var linearScale         = fp4x4.TRS(fp3.zero, fpmath.inverse(orientationFixup), shape.transform.lossyScale).DecomposeScale();

            // radius is max of the two non-height axes
            var radius = (fp)shape.radius * fpmath.cmax(new fp3(fpmath.abs(linearScale)) { [shape.direction] = fp.zero });

            var ax = new fp3 { [shape.direction] = fp.one };
            var vertex = ax * (fp.half * (fp)shape.height);
            var worldCenter = fpmath.mul(shapeLocalToWorld, new fp4(shape.center, fp.zero));
            var offset = fpmath.mul(fpmath.inverse(new fp4x4(rigidBodyTransform)), worldCenter).xyz - shape.center * fpmath.abs(linearScale);

            var v0 = fpmath.mul(orientationFixup, offset + ((fp3)shape.center + vertex) * fpmath.abs(linearScale) - ax * radius);
            var v1 = fpmath.mul(orientationFixup, offset + ((fp3)shape.center - vertex) * fpmath.abs(linearScale) + ax * radius);

            res.CapsuleProperties = new CapsuleGeometry { Vertex0 = v0, Vertex1 = v1, Radius = radius };

            return res;
        }
    }

    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("adamm", 3)]
    public sealed class LegacySphereColliderConversionSystem : BaseLegacyColliderConversionSystem<LegacySphere>
    {
        internal override ShapeComputationData GenerateComputationData(
            LegacySphere shape, ColliderInstance colliderInstance,
            NativeList<fp3> allConvexHullPoints, NativeList<fp3> allMeshVertices, NativeList<int3> allMeshTriangles,
            HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            var res = base.GenerateComputationData(shape, colliderInstance, allConvexHullPoints, allMeshVertices, allMeshTriangles, meshAssets);
            res.ShapeType = ShapeType.Sphere;

            var shapeLocalToWorld = shape.transform.localToWorldMatrix;
            var worldCenter = fpmath.mul(shapeLocalToWorld, new fp4(shape.center, fp.one));
            var transformRotation  = shape.transform.rotation;
            var rigidBodyTransform  = Math.DecomposeRigidBodyTransform(shapeLocalToWorld);
            var orientationFixup    = fpmath.inverse(fpmath.mul(fpmath.inverse(transformRotation), rigidBodyTransform.rot));

            var shapeFromWorld = fpmath.inverse(new fp4x4(rigidBodyTransform));
            var center = fpmath.mul(shapeFromWorld, worldCenter).xyz;

            var linearScale = fp4x4.TRS(fp3.zero, fpmath.inverse(orientationFixup), shape.transform.lossyScale).DecomposeScale();
            var radius = (fp)shape.radius * fpmath.cmax(fpmath.abs(linearScale));

            res.SphereProperties = new SphereGeometry { Center = center, Radius = radius };

            return res;
        }
    }

    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [UpdateBefore(typeof(BuildCompoundCollidersConversionSystem))]
    [ConverterVersion("adamm", 5)]
    public sealed class LegacyMeshColliderConversionSystem : BaseLegacyColliderConversionSystem<LegacyMesh>
    {
        internal override ShapeComputationData GenerateComputationData(
            LegacyMesh shape, ColliderInstance colliderInstance,
            NativeList<fp3> allConvexHullPoints, NativeList<fp3> allMeshVertices, NativeList<int3> allMeshTriangles,
            HashSet<UnityEngine.Mesh> meshAssets
        )
        {
            if (shape.sharedMesh == null)
            {
                throw new InvalidOperationException(
                    $"No {nameof(LegacyMesh.sharedMesh)} assigned to {typeof(MeshCollider)} on {shape.name}."
                );
            }

            if (!shape.sharedMesh.IsValidForConversion(shape.gameObject))
            {
                throw new InvalidOperationException(
                    $"Mesh '{shape.sharedMesh}' assigned to {typeof(MeshCollider)} on {shape.name} is not readable. Ensure that you have enabled Read/Write on its import settings."
                );
            }

            meshAssets.Add(shape.sharedMesh);

            var res = base.GenerateComputationData(shape, colliderInstance, allConvexHullPoints, allMeshVertices, allMeshTriangles, meshAssets);

            if (shape.convex)
            {
                res.ShapeType = ShapeType.ConvexHull;
                res.ConvexHullProperties.Material = res.Material;
                res.ConvexHullProperties.Filter = res.CollisionFilter;
                res.ConvexHullProperties.GenerationParameters = ConvexHullGenerationParameters.Default;
            }
            else
            {
                res.ShapeType = ShapeType.Mesh;
                res.MeshProperties.Material = res.Material;
                res.MeshProperties.Filter = res.CollisionFilter;
                res.ConvexHullProperties.GenerationParameters = default;
            }

            var transform = shape.transform;
            var rigidBodyTransform = Math.DecomposeRigidBodyTransform(transform.localToWorldMatrix);
            var bakeFromShape = fpmath.mul(fpmath.inverse(new fp4x4(rigidBodyTransform)), transform.localToWorldMatrix);

            res.Instance.Hash = HashableShapeInputs.GetHash128(
                0u,
                res.ConvexHullProperties.GenerationParameters,
                res.Material,
                res.CollisionFilter,
                bakeFromShape,
                new NativeArray<HashableShapeInputs>(1, Allocator.Temp) { [0] = HashableShapeInputs.FromMesh(shape.sharedMesh, fp4x4.identity) },
                default,
                default, HashableShapeInputs.k_DefaultLinearPrecision
            );

            if (BlobComputationContext.NeedToComputeBlobAsset(res.Instance.Hash))
            {
                if (shape.convex && TryGetRegisteredConvexInputs(res.Instance.Hash, out var convexInputs))
                {
                    res.ConvexHullProperties.PointsStart = convexInputs.PointsStart;
                    res.ConvexHullProperties.PointCount = convexInputs.PointCount;
                }
                else if (!shape.convex && TryGetRegisteredMeshInputs(res.Instance.Hash, out var meshInputs))
                {
                    res.MeshProperties.VerticesStart = meshInputs.VerticesStart;
                    res.MeshProperties.VertexCount = meshInputs.VertexCount;
                    res.MeshProperties.TrianglesStart = meshInputs.TrianglesStart;
                    res.MeshProperties.TriangleCount = meshInputs.TriangleCount;
                }
                else
                {
                    var pointCloud = new NativeList<fp3>(shape.sharedMesh.vertexCount, Allocator.Temp);
                    var triangles = new NativeList<int3>(shape.sharedMesh.triangles.Length / 3, Allocator.Temp);
                    PhysicsShapeAuthoring.AppendMeshPropertiesToNativeBuffers(
                        fp4x4.identity, shape.sharedMesh,
                        pointCloud, triangles,
                        default, default
                    );
                    for (int i = 0, count = pointCloud.Length; i < count; ++i)
                        pointCloud[i] = fpmath.mul(bakeFromShape, new fp4(pointCloud[i], fp.one)).xyz;

                    if (shape.convex)
                    {
                        res.ConvexHullProperties.PointsStart = allConvexHullPoints.Length;
                        res.ConvexHullProperties.PointCount = pointCloud.Length;
                        allConvexHullPoints.AddRange(pointCloud);
                    }
                    else
                    {
                        if (pointCloud.Length == 0 || triangles.Length == 0)
                        {
                            throw new InvalidOperationException(
                                $"Invalid mesh data associated with {shape.name}. " +
                                "Ensure that you have enabled Read/Write on the mesh's import settings."
                            );
                        }

                        res.MeshProperties.VerticesStart = allMeshVertices.Length;
                        res.MeshProperties.VertexCount = pointCloud.Length;
                        res.MeshProperties.TrianglesStart = allMeshTriangles.Length;
                        res.MeshProperties.TriangleCount = triangles.Length;
                        allMeshVertices.AddRange(pointCloud);
                        allMeshTriangles.AddRange(triangles);
                    }
                }
            }

            return res;
        }
    }
}
#endif

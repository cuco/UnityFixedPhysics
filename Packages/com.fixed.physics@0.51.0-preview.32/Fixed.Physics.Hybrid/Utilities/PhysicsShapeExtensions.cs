using System;
using System.Collections.Generic;
using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using UnityEngine;
using Hash128 = Unity.Entities.Hash128;
#if LEGACY_PHYSICS
using LegacyCollider = UnityEngine.Collider;
using LegacyRigidBody = UnityEngine.Rigidbody;
#endif

namespace Fixed.Physics.Authoring
{
    // put static UnityObject buffers in separate utility class so other methods can Burst compile
    static class PhysicsShapeExtensions_NonBursted
    {
#if LEGACY_PHYSICS
        internal static readonly List<LegacyRigidBody> s_RigidbodiesBuffer = new List<LegacyRigidBody>(16);
        internal static readonly List<LegacyCollider> s_CollidersBuffer = new List<LegacyCollider>(16);
#endif
        internal static readonly List<PhysicsBodyAuthoring> s_PhysicsBodiesBuffer = new List<PhysicsBodyAuthoring>(16);
        internal static readonly List<PhysicsShapeAuthoring> s_ShapesBuffer = new List<PhysicsShapeAuthoring>(16);
        internal static readonly List<StaticOptimizeEntity> s_StaticOptimizeEntitiesBuffer = new List<StaticOptimizeEntity>(4);
    }

    static class PhysicsShapeExtensions
    {
        internal static int GetDeviantAxis(this fp3 v)
        {
            var deviation = fpmath.abs(v - fpmath.csum(v) / (fp)3f);
            return fpmath.cmax(deviation) == deviation.z ? 2 : fpmath.cmax(deviation) == deviation.y ? 1 : 0;
        }

        internal static int GetMaxAxis(this fp3 v)
        {
            var cmax = fpmath.cmax(v);
            return cmax == v.z ? 2 : cmax == v.y ? 1 : 0;
        }

        //TODO
        static readonly fp k_HashFloatTolerance = (fp)0.01f;

        // used to hash convex hull generation properties in a way that is robust to imprecision
        internal static uint GetStableHash(
            this ConvexHullGenerationParameters generationParameters,
            ConvexHullGenerationParameters hashedParameters,
            fp tolerance/* = k_HashFloatTolerance*/
        )
        {
            var differences = new fp3(
                generationParameters.BevelRadius - hashedParameters.BevelRadius,
                generationParameters.MinimumAngle - hashedParameters.MinimumAngle,
                generationParameters.SimplificationTolerance - hashedParameters.SimplificationTolerance
            );
            return fpmath.cmax(fpmath.abs(differences)) < tolerance
                ? unchecked((uint)hashedParameters.GetHashCode())
                : unchecked((uint)generationParameters.GetHashCode());
        }

        // used to hash an array of points in a way that is robust to imprecision
        internal static unsafe uint GetStableHash(
            this NativeList<fp3> points, NativeArray<fp3> hashedPoints, fp tolerance/* = k_HashFloatTolerance*/
        )
        {
            if (points.Length != hashedPoints.Length)
                return math.hash(points.GetUnsafePtr(), UnsafeUtility.SizeOf<fp3>() * points.Length);

            for (int i = 0, count = points.Length; i < count; ++i)
            {
                if (fpmath.cmax(fpmath.abs(points[i] - hashedPoints[i])) > tolerance)
                    return math.hash(points.GetUnsafePtr(), UnsafeUtility.SizeOf<fp3>() * points.Length);
            }
            return math.hash(hashedPoints.GetUnsafePtr(), UnsafeUtility.SizeOf<fp3>() * hashedPoints.Length);
        }

        internal static CollisionFilter GetFilter(this PhysicsShapeAuthoring shape)
        {
            // TODO: determine optimal workflow for specifying group index
            return new CollisionFilter
            {
                BelongsTo = shape.BelongsTo.Value,
                CollidesWith = shape.CollidesWith.Value
            };
        }

        internal static Material GetMaterial(this PhysicsShapeAuthoring shape)
        {
            // TODO: TBD how we will author editor content for other shape flags
            return new Material
            {
                Friction = shape.Friction.Value,
                FrictionCombinePolicy = shape.Friction.CombineMode,
                Restitution = shape.Restitution.Value,
                RestitutionCombinePolicy = shape.Restitution.CombineMode,
                CollisionResponse = shape.CollisionResponse,
                CustomTags = shape.CustomTags.Value
            };
        }

        static bool HasNonUniformScale(this fp4x4 m)
        {
            var s = new fp3(fpmath.lengthsq(m.c0.xyz), fpmath.lengthsq(m.c1.xyz), fpmath.lengthsq(m.c2.xyz));
            return fpmath.cmin(s) != fpmath.cmax(s);
        }

        internal static bool HasShear(this fp4x4 m)
        {
            // scale each axis by abs of its max component in order to work with very large/small scales
            var rs0 = m.c0.xyz / fpmath.max(fpmath.cmax(fpmath.abs(m.c0.xyz)), fp.epsilon);
            var rs1 = m.c1.xyz / fpmath.max(fpmath.cmax(fpmath.abs(m.c1.xyz)), fp.epsilon);
            var rs2 = m.c2.xyz / fpmath.max(fpmath.cmax(fpmath.abs(m.c2.xyz)), fp.epsilon);
            // verify all axes are orthogonal
            //TODO
            fp k_Zero = (fp)1e-6f;
            return
                fpmath.abs(fpmath.dot(rs0, rs1)) > k_Zero ||
                fpmath.abs(fpmath.dot(rs0, rs2)) > k_Zero ||
                fpmath.abs(fpmath.dot(rs1, rs2)) > k_Zero;
        }

        // TODO: revisit readable requirement when conversion is editor-only
        internal static bool IsValidForConversion(this UnityEngine.Mesh mesh, GameObject host)
        {
#if UNITY_EDITOR
            // anything in a sub-scene is fine because it is converted at edit time, but run-time ConvertToEntity will fail
            if (
                host.gameObject.scene.isSubScene
                // isSubScene is false in AssetImportWorker during sub-scene import
#if UNITY_2020_2_OR_NEWER
                || UnityEditor.AssetDatabase.IsAssetImportWorkerProcess()
#else
                || UnityEditor.Experimental.AssetDatabaseExperimental.IsAssetImportWorkerProcess()
#endif
            )
                return true;
#endif
            return mesh.isReadable;
        }

#if LEGACY_PHYSICS
        internal static GameObject GetPrimaryBody(this LegacyCollider collider) => GetPrimaryBody(collider.gameObject);
#endif
        internal static GameObject GetPrimaryBody(this PhysicsShapeAuthoring shape) => GetPrimaryBody(shape.gameObject);

        internal static GameObject GetPrimaryBody(GameObject shape)
        {
            var pb = FindFirstEnabledAncestor(shape, PhysicsShapeExtensions_NonBursted.s_PhysicsBodiesBuffer);
#if LEGACY_PHYSICS
            var rb = FindFirstEnabledAncestor(shape, PhysicsShapeExtensions_NonBursted.s_RigidbodiesBuffer);
#else
            GameObject rb = null;
#endif

            if (pb != null)
            {
                return rb == null ? pb.gameObject :
                    pb.transform.IsChildOf(rb.transform) ? pb.gameObject : rb.gameObject;
            }

            if (rb != null)
                return rb.gameObject;

            // for implicit static shape, first see if it is part of static optimized hierarchy
            var topStatic = FindTopmostEnabledAncestor(shape, PhysicsShapeExtensions_NonBursted.s_StaticOptimizeEntitiesBuffer);
            if (topStatic != null)
                return topStatic;

            // otherwise, find topmost enabled Collider or PhysicsShapeAuthoring
#if LEGACY_PHYSICS
            var topCollider = FindTopmostEnabledAncestor(shape, PhysicsShapeExtensions_NonBursted.s_CollidersBuffer);
#else
            GameObject topCollider = null;
#endif
            var topShape = FindTopmostEnabledAncestor(shape, PhysicsShapeExtensions_NonBursted.s_ShapesBuffer);

            return topCollider == null
                ? topShape == null ? shape.gameObject : topShape
                : topShape == null
                ? topCollider
                : topShape.transform.IsChildOf(topCollider.transform)
                ? topCollider
                : topShape;
        }

        static GameObject FindFirstEnabledAncestor<T>(GameObject shape, List<T> buffer) where T : Component
        {
            // include inactive in case the supplied shape GameObject is a prefab that has not been instantiated
            shape.GetComponentsInParent(true, buffer);
            GameObject result = null;
            for (int i = 0, count = buffer.Count; i < count; ++i)
            {
                if (
#if LEGACY_PHYSICS
                    (buffer[i] as LegacyCollider)?.enabled ??
#endif
                    (buffer[i] as MonoBehaviour)?.enabled ?? true)
                {
                    result = buffer[i].gameObject;
                    break;
                }
            }
            buffer.Clear();
            return result;
        }

        static GameObject FindTopmostEnabledAncestor<T>(GameObject shape, List<T> buffer) where T : Component
        {
            // include inactive in case the supplied shape GameObject is a prefab that has not been instantiated
            shape.GetComponentsInParent(true, buffer);
            GameObject result = null;
            for (var i = buffer.Count - 1; i >= 0; --i)
            {
                if (
#if LEGACY_PHYSICS
                    (buffer[i] as LegacyCollider)?.enabled ??
#endif
                    (buffer[i] as MonoBehaviour)?.enabled ?? true
                )
                {
                    result = buffer[i].gameObject;
                    break;
                }
            }
            buffer.Clear();
            return result;
        }

        // matrix to transform point from shape's local basis into world space
        static fp4x4 GetBasisToWorldMatrix(
            fp4x4 localToWorld, fp3 center, fpquaternion orientation, fp3 size
        ) =>
            fpmath.mul(localToWorld, fp4x4.TRS(center, orientation, size));

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        static void CheckBasisPriorityAndThrow(int3 basisPriority)
        {
            if (
                basisPriority.x == basisPriority.y
                || basisPriority.x == basisPriority.z
                || basisPriority.y == basisPriority.z
            )
                throw new ArgumentException(nameof(basisPriority));
        }

        // matrix to transform point on a primitive from bake space into space of the shape
        static fp4x4 GetPrimitiveBakeToShapeMatrix(
            fp4x4 localToWorld, fp4x4 shapeToWorld, ref fp3 center, ref EulerAngles orientation, fp3 scale, int3 basisPriority
        )
        {
            CheckBasisPriorityAndThrow(basisPriority);

            var localToBasis = fp4x4.TRS(center, orientation, scale);
            // correct for imprecision in cases of no scale to prevent e.g., convex radius from being altered
            if (scale.Equals(new fp3(fp.one)))
            {
                localToBasis.c0 = fpmath.normalizesafe(localToBasis.c0);
                localToBasis.c1 = fpmath.normalizesafe(localToBasis.c1);
                localToBasis.c2 = fpmath.normalizesafe(localToBasis.c2);
            }
            var localToBake = fpmath.mul(localToWorld, localToBasis);

            if (localToBake.HasNonUniformScale() || localToBake.HasShear())
            {
                // deskew second longest axis with respect to longest axis
                localToBake[basisPriority[1]] =
                    DeskewSecondaryAxis(localToBake[basisPriority[0]], localToBake[basisPriority[1]]);

                // recompute third axes from first two
                var n2 = fpmath.normalizesafe(
                    new fp4(fpmath.cross(localToBake[basisPriority[0]].xyz, localToBake[basisPriority[1]].xyz), fp.zero)
                );
                localToBake[basisPriority[2]] = n2 * fpmath.dot(localToBake[basisPriority[2]], n2);
            }

            var bakeToShape = fpmath.mul(fpmath.inverse(shapeToWorld), localToBake);
            // transform baked center/orientation (i.e. primitive basis) into shape space
            orientation.SetValue(
                fpquaternion.LookRotationSafe(bakeToShape[basisPriority[0]].xyz, bakeToShape[basisPriority[1]].xyz)
            );
            center = bakeToShape.c3.xyz;

            return bakeToShape;
        }

        static fp4 DeskewSecondaryAxis(fp4 primaryAxis, fp4 secondaryAxis)
        {
            var n0 = fpmath.normalizesafe(primaryAxis);
            var dot = fpmath.dot(secondaryAxis, n0);
            return secondaryAxis - n0 * dot;
        }

        static readonly int[] k_NextAxis = { 1, 2, 0 };
        static readonly int[] k_PrevAxis = { 2, 0, 1 };

        // used for de-skewing basis vectors; default priority assumes primary axis is z, secondary axis is y
        static readonly int3 k_DefaultAxisPriority = new int3(2, 1, 0);

        // priority is determined by length of each size dimension in the shape's basis after applying localToWorld transformation
        static int3 GetBasisAxisPriority(fp4x4 basisToWorld)
        {
            var basisAxisLengths = basisToWorld.DecomposeScale();
            var max = fpmath.cmax(basisAxisLengths);
            var min = fpmath.cmin(basisAxisLengths);
            if (max == min)
                return k_DefaultAxisPriority;

            var imax = max == basisAxisLengths.x ? 0 : max == basisAxisLengths.y ? 1 : 2;

            basisToWorld[k_NextAxis[imax]] = DeskewSecondaryAxis(basisToWorld[imax], basisToWorld[k_NextAxis[imax]]);
            basisToWorld[k_PrevAxis[imax]] = DeskewSecondaryAxis(basisToWorld[imax], basisToWorld[k_PrevAxis[imax]]);

            basisAxisLengths = basisToWorld.DecomposeScale();
            min = fpmath.cmin(basisAxisLengths);
            var imin = min == basisAxisLengths.x ? 0 : min == basisAxisLengths.y ? 1 : 2;
            if (imin == imax)
                imin = k_NextAxis[imax];
            var imid = k_NextAxis[imax] == imin ? k_PrevAxis[imax] : k_NextAxis[imax];

            return new int3(imax, imid, imin);
        }

        internal static BoxGeometry GetBakedBoxProperties(this PhysicsShapeAuthoring shape)
        {
            var box = shape.GetBoxProperties(out var orientation);
            return box.BakeToBodySpace(shape.transform.localToWorldMatrix, shape.GetShapeToWorldMatrix(), orientation);
        }

        internal static BoxGeometry BakeToBodySpace(
            this BoxGeometry box, fp4x4 localToWorld, fp4x4 shapeToWorld, EulerAngles orientation
        )
        {
            using (var geometry = new NativeArray<BoxGeometry>(1, Allocator.TempJob) { [0] = box })
            {
                var job = new BakeBoxJob
                {
                    Box = geometry,
                    localToWorld = localToWorld,
                    shapeToWorld = shapeToWorld,
                    orientation = orientation
                };
                job.Run();
                return geometry[0];
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        struct BakeBoxJob : IJob
        {
            public NativeArray<BoxGeometry> Box;
            // TODO: make members PascalCase after merging static query fixes
            public fp4x4 localToWorld;
            public fp4x4 shapeToWorld;
            public EulerAngles orientation;

            public static fp4x4 GetBakeToShape(PhysicsShapeAuthoring shape, fp3 center, EulerAngles orientation)
            {
                var transform = shape.transform;
                var localToWorld = (fp4x4)transform.localToWorldMatrix;
                var shapeToWorld = shape.GetShapeToWorldMatrix();
                return GetBakeToShape(localToWorld, shapeToWorld, ref center, ref orientation);
            }

            static fp4x4 GetBakeToShape(fp4x4 localToWorld, fp4x4 shapeToWorld, ref fp3 center, ref EulerAngles orientation)
            {
                fp4x4 bakeToShape;
                fp4x4 rotationMatrix = fp4x4.identity;
                var basisPriority = k_DefaultAxisPriority;
                var sheared = localToWorld.HasShear();
                if (localToWorld.HasNonUniformScale() || sheared)
                {
                    if (sheared)
                    {
                        var transformScale = localToWorld.DecomposeScale();
                        var basisToWorld = GetBasisToWorldMatrix(localToWorld, center, orientation, transformScale);
                        basisPriority = GetBasisAxisPriority(basisToWorld);
                    }

                    rotationMatrix = new fp4x4(
                        new fp4 { [basisPriority[2]] = fp.one },
                        new fp4 { [basisPriority[1]] = fp.one },
                        new fp4 { [basisPriority[0]] = fp.one },
                        new fp4 { [3] = fp.one }
                    );
                }

                bakeToShape =
                    GetPrimitiveBakeToShapeMatrix(localToWorld, shapeToWorld, ref center, ref orientation, fp.one, basisPriority);

                bakeToShape = fpmath.mul(bakeToShape, rotationMatrix);
                return bakeToShape;
            }

            public void Execute()
            {
                var center = Box[0].Center;
                var size = Box[0].Size;
                var bevelRadius = Box[0].BevelRadius;

                var bakeToShape = GetBakeToShape(localToWorld, shapeToWorld, ref center, ref orientation);
                bakeToShape = fpmath.mul(bakeToShape, fp4x4.Scale(size));

                var scale = bakeToShape.DecomposeScale();

                size = scale;

                Box[0] = new BoxGeometry
                {
                    Center = center,
                    Orientation = orientation,
                    Size = size,
                    BevelRadius = fpmath.clamp(bevelRadius, fp.zero, fp.half * fpmath.cmin(size))
                };
            }
        }

        // avoids drift in axes we're not actually changing
        static readonly fp kMinimumChange = HashableShapeInputs.k_DefaultLinearPrecision;

        internal static void SetBakedBoxSize(this PhysicsShapeAuthoring shape, fp3 size, fp bevelRadius)
        {
            var box         = shape.GetBoxProperties(out var orientation);
            var center      = box.Center;
            var prevSize    = fpmath.abs(box.Size);
            size = fpmath.abs(size);

            var bakeToShape = BakeBoxJob.GetBakeToShape(shape, center, orientation);
            var scale = bakeToShape.DecomposeScale();

            size /= scale;

            if (fpmath.abs(size[0] - prevSize[0]) < kMinimumChange) size[0] = prevSize[0];
            if (fpmath.abs(size[1] - prevSize[1]) < kMinimumChange) size[1] = prevSize[1];
            if (fpmath.abs(size[2] - prevSize[2]) < kMinimumChange) size[2] = prevSize[2];

            box.BevelRadius = bevelRadius;
            box.Size = size;

            shape.SetBox(box, orientation);
        }

        static void MakeZAxisPrimaryBasis(ref int3 basisPriority)
        {
            if (basisPriority[1] == 2)
                basisPriority = basisPriority.yxz;
            else if (basisPriority[2] == 2)
                basisPriority = basisPriority.zxy;
        }

        internal static fp3 GetCenter(this CapsuleGeometry geometry) =>
            fpmath.lerp(geometry.Vertex0, geometry.Vertex1, fp.half);

        internal static fp GetHeight(this CapsuleGeometry geometry) =>
            (fp)2f * geometry.Radius + fpmath.length(geometry.Vertex1 - geometry.Vertex0);

        internal static CapsuleGeometryAuthoring GetBakedCapsuleProperties(this PhysicsShapeAuthoring shape)
        {
            var capsule = shape.GetCapsuleProperties();
            return capsule.BakeToBodySpace(shape.transform.localToWorldMatrix, shape.GetShapeToWorldMatrix());
        }

        internal static CapsuleGeometryAuthoring BakeToBodySpace(
            this CapsuleGeometryAuthoring capsule, fp4x4 localToWorld, fp4x4 shapeToWorld
        )
        {
            using (var geometry = new NativeArray<CapsuleGeometryAuthoring>(1, Allocator.TempJob) { [0] = capsule })
            {
                var job = new BakeCapsuleJob
                {
                    Capsule = geometry,
                    localToWorld = localToWorld,
                    shapeToWorld = shapeToWorld
                };
                job.Run();
                return geometry[0];
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        struct BakeCapsuleJob : IJob
        {
            public NativeArray<CapsuleGeometryAuthoring> Capsule;
            // TODO: make members PascalCase after merging static query fixes
            public fp4x4 localToWorld;
            public fp4x4 shapeToWorld;

            public static fp4x4 GetBakeToShape(PhysicsShapeAuthoring shape, fp3 center, EulerAngles orientation)
            {
                var transform = shape.transform;
                var localToWorld = (fp4x4)transform.localToWorldMatrix;
                var shapeToWorld = shape.GetShapeToWorldMatrix();
                return GetBakeToShape(localToWorld, shapeToWorld, ref center, ref orientation);
            }

            static fp4x4 GetBakeToShape(fp4x4 localToWorld, fp4x4 shapeToWorld, ref fp3 center, ref EulerAngles orientation)
            {
                var basisPriority = k_DefaultAxisPriority;
                var sheared = localToWorld.HasShear();
                if (localToWorld.HasNonUniformScale() || sheared)
                {
                    if (sheared)
                    {
                        var transformScale = localToWorld.DecomposeScale();
                        var basisToWorld = GetBasisToWorldMatrix(localToWorld, center, orientation, transformScale);
                        basisPriority = GetBasisAxisPriority(basisToWorld);
                    }
                    MakeZAxisPrimaryBasis(ref basisPriority);
                }
                return GetPrimitiveBakeToShapeMatrix(localToWorld, shapeToWorld, ref center, ref orientation, fp.one, basisPriority);
            }

            public void Execute()
            {
                var radius = Capsule[0].Radius;
                var center = Capsule[0].Center;
                var height = Capsule[0].Height;
                var orientationEuler = Capsule[0].OrientationEuler;

                var bakeToShape = GetBakeToShape(localToWorld, shapeToWorld, ref center, ref orientationEuler);
                var scale = bakeToShape.DecomposeScale();

                radius *= fpmath.cmax(scale.xy);
                height = fpmath.max(fp.zero, height * scale.z);

                Capsule[0] = new CapsuleGeometryAuthoring
                {
                    OrientationEuler = orientationEuler,
                    Center = center,
                    Height = height,
                    Radius = radius
                };
            }
        }

        internal static void SetBakedCapsuleSize(this PhysicsShapeAuthoring shape, fp height, fp radius)
        {
            var capsule = shape.GetCapsuleProperties();
            var center  = capsule.Center;

            var bakeToShape = BakeCapsuleJob.GetBakeToShape(shape, center, capsule.OrientationEuler);
            var scale = bakeToShape.DecomposeScale();

            var newRadius = radius / fpmath.cmax(scale.xy);
            if (fpmath.abs(capsule.Radius - newRadius) > kMinimumChange) capsule.Radius = newRadius;

            height /= scale.z;

            if (fpmath.abs(fpmath.length(capsule.Height - height)) > kMinimumChange) capsule.Height = height;

            shape.SetCapsule(capsule);
        }

        internal static CylinderGeometry GetBakedCylinderProperties(this PhysicsShapeAuthoring shape)
        {
            var cylinder = shape.GetCylinderProperties(out var orientation);
            return cylinder.BakeToBodySpace(shape.transform.localToWorldMatrix, shape.GetShapeToWorldMatrix(), orientation);
        }

        internal static CylinderGeometry BakeToBodySpace(
            this CylinderGeometry cylinder, fp4x4 localToWorld, fp4x4 shapeToWorld, EulerAngles orientation
        )
        {
            using (var geometry = new NativeArray<CylinderGeometry>(1, Allocator.TempJob) { [0] = cylinder })
            {
                var job = new BakeCylinderJob
                {
                    Cylinder = geometry,
                    localToWorld = localToWorld,
                    shapeToWorld = shapeToWorld,
                    orientation = orientation
                };
                job.Run();
                return geometry[0];
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        struct BakeCylinderJob : IJob
        {
            public NativeArray<CylinderGeometry> Cylinder;
            // TODO: make members PascalCase after merging static query fixes
            public fp4x4 localToWorld;
            public fp4x4 shapeToWorld;
            public EulerAngles orientation;

            public static fp4x4 GetBakeToShape(PhysicsShapeAuthoring shape, fp3 center, EulerAngles orientation)
            {
                var transform = shape.transform;
                var localToWorld = (fp4x4)transform.localToWorldMatrix;
                var shapeToWorld = shape.GetShapeToWorldMatrix();
                return GetBakeToShape(localToWorld, shapeToWorld, ref center, ref orientation);
            }

            static fp4x4 GetBakeToShape(fp4x4 localToWorld, fp4x4 shapeToWorld, ref fp3 center, ref EulerAngles orientation)
            {
                var basisPriority = k_DefaultAxisPriority;
                var sheared = localToWorld.HasShear();
                if (localToWorld.HasNonUniformScale() || sheared)
                {
                    if (sheared)
                    {
                        var transformScale = localToWorld.DecomposeScale();
                        var basisToWorld = GetBasisToWorldMatrix(localToWorld, center, orientation, transformScale);
                        basisPriority = GetBasisAxisPriority(basisToWorld);
                    }
                    MakeZAxisPrimaryBasis(ref basisPriority);
                }
                return GetPrimitiveBakeToShapeMatrix(localToWorld, shapeToWorld, ref center, ref orientation, fp.one, basisPriority);
            }

            public void Execute()
            {
                var center = Cylinder[0].Center;
                var height = Cylinder[0].Height;
                var radius = Cylinder[0].Radius;
                var bevelRadius = Cylinder[0].BevelRadius;

                var bakeToShape = GetBakeToShape(localToWorld, shapeToWorld, ref center, ref orientation);
                var scale = bakeToShape.DecomposeScale();

                height *= scale.z;
                radius *= fpmath.cmax(scale.xy);

                Cylinder[0] = new CylinderGeometry
                {
                    Center = center,
                    Orientation = orientation,
                    Height = height,
                    Radius = radius,
                    BevelRadius = fpmath.min(bevelRadius, fpmath.min(height * fp.half, radius)),
                    SideCount = Cylinder[0].SideCount
                };
            }
        }

        internal static void SetBakedCylinderSize(this PhysicsShapeAuthoring shape, fp height, fp radius, fp bevelRadius)
        {
            var cylinder    = shape.GetCylinderProperties(out EulerAngles orientation);
            var center      = cylinder.Center;

            var bakeToShape = BakeCylinderJob.GetBakeToShape(shape, center, orientation);
            var scale = bakeToShape.DecomposeScale();

            var newRadius = radius / fpmath.cmax(scale.xy);
            if (fpmath.abs(cylinder.Radius - newRadius) > kMinimumChange) cylinder.Radius = newRadius;
            if (fpmath.abs(cylinder.BevelRadius - bevelRadius) > kMinimumChange) cylinder.BevelRadius = bevelRadius;


            var newHeight = fpmath.max(fp.zero, height / scale.z);
            if (fpmath.abs(cylinder.Height - newHeight) > kMinimumChange) cylinder.Height = newHeight;
            shape.SetCylinder(cylinder, orientation);
        }

        internal static SphereGeometry GetBakedSphereProperties(this PhysicsShapeAuthoring shape, out EulerAngles orientation)
        {
            var sphere = shape.GetSphereProperties(out orientation);
            return sphere.BakeToBodySpace(shape.transform.localToWorldMatrix, shape.GetShapeToWorldMatrix(), ref orientation);
        }

        internal static SphereGeometry BakeToBodySpace(
            this SphereGeometry sphere, fp4x4 localToWorld, fp4x4 shapeToWorld, ref EulerAngles orientation
        )
        {
            using (var geometry = new NativeArray<SphereGeometry>(1, Allocator.TempJob) { [0] = sphere })
            using (var outOrientation = new NativeArray<EulerAngles>(1, Allocator.TempJob) { [0] = orientation })
            {
                var job = new BakeSphereJob
                {
                    Sphere = geometry,
                    Orientation = outOrientation,
                    localToWorld = localToWorld,
                    shapeToWorld = shapeToWorld
                };
                job.Run();
                orientation = outOrientation[0];
                return geometry[0];
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        struct BakeSphereJob : IJob
        {
            public NativeArray<SphereGeometry> Sphere;
            public NativeArray<EulerAngles> Orientation;
            // TODO: make members PascalCase after merging static query fixes
            public fp4x4 localToWorld;
            public fp4x4 shapeToWorld;

            public void Execute()
            {
                var center = Sphere[0].Center;
                var radius = Sphere[0].Radius;
                var orientation = Orientation[0];

                var basisToWorld = GetBasisToWorldMatrix(localToWorld, center, orientation, fp.one);
                var basisPriority = basisToWorld.HasShear() ? GetBasisAxisPriority(basisToWorld) : k_DefaultAxisPriority;
                var bakeToShape = GetPrimitiveBakeToShapeMatrix(localToWorld, shapeToWorld, ref center, ref orientation, fp.one, basisPriority);

                radius *= fpmath.cmax(bakeToShape.DecomposeScale());

                Sphere[0] = new SphereGeometry
                {
                    Center = center,
                    Radius = radius
                };
                Orientation[0] = orientation;
            }
        }

        internal static void SetBakedSphereRadius(this PhysicsShapeAuthoring shape, fp radius)
        {
            var sphere = shape.GetSphereProperties(out EulerAngles eulerAngles);
            var center = sphere.Center;
            radius = fpmath.abs(radius);

            var basisToWorld    = GetBasisToWorldMatrix(shape.transform.localToWorldMatrix, center, eulerAngles, fp.one);
            var basisPriority   = basisToWorld.HasShear() ? GetBasisAxisPriority(basisToWorld) : k_DefaultAxisPriority;
            var bakeToShape     = GetPrimitiveBakeToShapeMatrix(shape.transform.localToWorldMatrix, shape.GetShapeToWorldMatrix(), ref center, ref eulerAngles, fp.one, basisPriority);

            var scale = fpmath.cmax(bakeToShape.DecomposeScale());

            var newRadius = radius / scale;
            sphere.Radius = newRadius;
            shape.SetSphere(sphere);
        }

        internal static void GetPlanePoints(
            fp3 center, fp2 size, EulerAngles orientation,
            out fp3 vertex0, out fp3 vertex1, out fp3 vertex2, out fp3 vertex3
        )
        {
            var sizeYUp = fpmath.fp3(size.x, fp.zero, size.y);

            vertex0 = center + fpmath.mul(orientation, sizeYUp * fpmath.fp3(-fp.half, fp.zero,  fp.half));
            vertex1 = center + fpmath.mul(orientation, sizeYUp * fpmath.fp3(fp.half, fp.zero,  fp.half));
            vertex2 = center + fpmath.mul(orientation, sizeYUp * fpmath.fp3(fp.half, fp.zero, -fp.half));
            vertex3 = center + fpmath.mul(orientation, sizeYUp * fpmath.fp3(-fp.half, fp.zero, -fp.half));
        }

        internal static void GetBakedPlaneProperties(
            this PhysicsShapeAuthoring shape, out fp3 vertex0, out fp3 vertex1, out fp3 vertex2, out fp3 vertex3
        )
        {
            shape.GetPlaneProperties(out var center, out var size, out EulerAngles orientation);
            BakeToBodySpace(
                center, size, orientation, shape.transform.localToWorldMatrix, shape.GetShapeToWorldMatrix(),
                out vertex0, out vertex1, out vertex2, out vertex3
            );
        }

        internal static void BakeToBodySpace(
            fp3 center, fp2 size, EulerAngles orientation, fp4x4 localToWorld, fp4x4 shapeToWorld,
            out fp3 vertex0, out fp3 vertex1, out fp3 vertex2, out fp3 vertex3
        )
        {
            using (var geometry = new NativeArray<fp3x4>(1, Allocator.TempJob))
            {
                var job = new BakePlaneJob
                {
                    Vertices = geometry,
                    center = center,
                    size = size,
                    orientation = orientation,
                    localToWorld = localToWorld,
                    shapeToWorld = shapeToWorld
                };
                job.Run();
                vertex0 = geometry[0].c0;
                vertex1 = geometry[0].c1;
                vertex2 = geometry[0].c2;
                vertex3 = geometry[0].c3;
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        struct BakePlaneJob : IJob
        {
            public NativeArray<fp3x4> Vertices;
            // TODO: make members PascalCase after merging static query fixes
            public fp3 center;
            public fp2 size;
            public EulerAngles orientation;
            public fp4x4 localToWorld;
            public fp4x4 shapeToWorld;

            public void Execute()
            {
                var v = Vertices[0];
                GetPlanePoints(center, size, orientation, out v.c0, out v.c1, out v.c2, out v.c3);
                var localToShape = fpmath.mul(fpmath.inverse(shapeToWorld), localToWorld);
                v.c0 = fpmath.mul(localToShape, new fp4(v.c0, fp.one)).xyz;
                v.c1 = fpmath.mul(localToShape, new fp4(v.c1, fp.one)).xyz;
                v.c2 = fpmath.mul(localToShape, new fp4(v.c2, fp.one)).xyz;
                v.c3 = fpmath.mul(localToShape, new fp4(v.c3, fp.one)).xyz;
                Vertices[0] = v;
            }
        }

        internal static void SetBakedPlaneSize(this PhysicsShapeAuthoring shape, fp2 size)
        {
            shape.GetPlaneProperties(out var center, out var planeSize, out EulerAngles orientation);

            var prevSize = fpmath.abs(planeSize);
            size = fpmath.abs(size);

            if (fpmath.abs(size[0] - prevSize[0]) < kMinimumChange) size[0] = prevSize[0];
            if (fpmath.abs(size[1] - prevSize[1]) < kMinimumChange) size[1] = prevSize[1];

            planeSize = size;

            shape.SetPlane(center, planeSize, orientation);
        }

        internal static Hash128 GetBakedConvexInputs(this PhysicsShapeAuthoring shape, HashSet<UnityEngine.Mesh> meshAssets)
        {
            using (var inputs = new NativeList<HashableShapeInputs>(8, Allocator.TempJob))
            using (var allSkinIndices = new NativeList<int>(4096, Allocator.TempJob))
            using (var allBlendShapeWeights = new NativeList<fp>(64, Allocator.TempJob))
            {
                shape.GetConvexHullProperties(default, true, inputs, allSkinIndices, allBlendShapeWeights, meshAssets);

                using (var hash = new NativeArray<Hash128>(1, Allocator.TempJob))
                {
                    var job = new GetShapeInputsHashJob
                    {
                        Result = hash,
                        ForceUniqueIdentifier = (uint)(shape.ForceUnique ? shape.GetInstanceID() : 0),
                        GenerationParameters = shape.ConvexHullGenerationParameters,
                        Material = shape.GetMaterial(),
                        CollisionFilter = shape.GetFilter(),
                        BakeFromShape = shape.GetLocalToShapeMatrix(),
                        Inputs = inputs,
                        AllSkinIndices = allSkinIndices,
                        AllBlendShapeWeights = allBlendShapeWeights
                    };
                    job.Run();
                    return hash[0];
                }
            }
        }

        internal static void GetBakedConvexProperties(this PhysicsShapeAuthoring shape, NativeList<fp3> pointCloud)
        {
            shape.GetConvexHullProperties(pointCloud, true, default, default, default, default);
            shape.BakePoints(pointCloud);
        }

#if !(UNITY_ANDROID && !UNITY_64) // !Android32
        // Getting memory alignment errors from HashUtility.Hash128 on Android32
        [BurstCompile]
#endif
        struct GetShapeInputsHashJob : IJob
        {
            public NativeArray<Hash128> Result;

            public uint ForceUniqueIdentifier;
            public ConvexHullGenerationParameters GenerationParameters;
            public Material Material;
            public CollisionFilter CollisionFilter;
            public fp4x4 BakeFromShape;

            [ReadOnly] public NativeArray<HashableShapeInputs> Inputs;
            [ReadOnly] public NativeArray<int> AllSkinIndices;
            [ReadOnly] public NativeArray<fp> AllBlendShapeWeights;

            public void Execute()
            {
                Result[0] = HashableShapeInputs.GetHash128(
                    ForceUniqueIdentifier, GenerationParameters, Material, CollisionFilter, BakeFromShape,
                    Inputs, AllSkinIndices, AllBlendShapeWeights, kMinimumChange
                );
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        internal struct GetAabbJob : IJob
        {
            [ReadOnly] public NativeArray<fp3> Points;
            public NativeArray<Aabb> Aabb;

            public void Execute()
            {
                var aabb = new Aabb { Min = fp.max_value, Max = fp.min_value };
                for (var i = 0; i < Points.Length; ++i)
                    aabb.Include(Points[i]);
                Aabb[0] = aabb;
            }
        }

        internal static Hash128 GetBakedMeshInputs(this PhysicsShapeAuthoring shape)
        {
            using (var inputs = new NativeList<HashableShapeInputs>(8, Allocator.TempJob))
            {
                shape.GetMeshProperties(default, default, true, inputs);
                using (var hash = new NativeArray<Hash128>(1, Allocator.TempJob))
                using (var allSkinIndices = new NativeArray<int>(0, Allocator.TempJob))
                using (var allBlendShapeWeights = new NativeArray<fp>(0, Allocator.TempJob))
                {
                    var job = new GetShapeInputsHashJob
                    {
                        Result = hash,
                        ForceUniqueIdentifier = (uint)(shape.ForceUnique ? shape.GetInstanceID() : 0),
                        Material = shape.GetMaterial(),
                        CollisionFilter = shape.GetFilter(),
                        BakeFromShape = shape.GetLocalToShapeMatrix(),
                        Inputs = inputs,
                        AllSkinIndices = allSkinIndices,
                        AllBlendShapeWeights = allBlendShapeWeights
                    };
                    job.Run();
                    return hash[0];
                }
            }
        }

        internal static void GetBakedMeshProperties(
            this PhysicsShapeAuthoring shape, NativeList<fp3> vertices, NativeList<int3> triangles,
            HashSet<UnityEngine.Mesh> meshAssets = null
        )
        {
            shape.GetMeshProperties(vertices, triangles, true, default, meshAssets);
            shape.BakePoints(vertices);
        }
    }
}

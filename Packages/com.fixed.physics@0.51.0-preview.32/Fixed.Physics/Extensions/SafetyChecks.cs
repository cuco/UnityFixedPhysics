using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;

[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.Collider>))]
[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.BoxCollider>))]
[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.CapsuleCollider>))]
[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.CylinderCollider>))]
[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.SphereCollider>))]
[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.PolygonCollider>))]
[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.ConvexCollider>))]
[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.MeshCollider>))]
[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.CompoundCollider>))]
[assembly: RegisterGenericComponentType(typeof(Fixed.Physics.DummyColliderFakesComponent<Fixed.Physics.TerrainCollider>))]

namespace Fixed.Physics
{
    //TODO: collect all other defines in the codebase here and move to it's own file
    static class CompilationSymbols
    {
        //TODO: change this to UNITY_DOTS_DEBUG, as that is the defacto debuging define in above entities .17

        public const string SafetyChecksSymbol = "ENABLE_UNITY_COLLECTIONS_CHECKS";
    }

    //Empty data container used to register mappings between collider headers and actual C# collider types
    struct DummyColliderFakesComponent<T> : IComponentData {}

    static class ColliderHeaderFakes
    {
        public const ColliderType k_AbstractType = unchecked((ColliderType)0xFF);

        public static ColliderHeader GetHeaderForColliderType<T>() where T : ICollider
        {
            var index = TypeManager.GetTypeIndex<DummyColliderFakesComponent<T>>();

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<BoxCollider>>())
                return new ColliderHeader() { Type = ColliderType.Box };

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<SphereCollider>>())
                return new ColliderHeader() { Type = ColliderType.Sphere };

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<CylinderCollider>>())
                return new ColliderHeader() { Type = ColliderType.Cylinder };

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<CapsuleCollider>>())
                return new ColliderHeader() { Type = ColliderType.Capsule };

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<ConvexCollider>>())
                return new ColliderHeader() { Type = ColliderType.Convex };

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<MeshCollider>>())
                return new ColliderHeader() { Type = ColliderType.Mesh };

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<CompoundCollider>>())
                return new ColliderHeader() { Type = ColliderType.Compound };

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<TerrainCollider>>())
                return new ColliderHeader() { Type = ColliderType.Terrain };

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<PolygonCollider>>())
                return new ColliderHeader() { Type = ColliderType.Triangle };

            if (index == TypeManager.GetTypeIndex<DummyColliderFakesComponent<Collider>>())
                return new ColliderHeader() { Type = k_AbstractType };

            throw new Exception($"The Collider typeIndex {index}, does not map to a supported ColliderType, please update the checks above.");
        }

        public static bool IsConvex(ColliderType type)
        {
            return type < ColliderType.Mesh;
        }
    }

    static class SafetyChecks
    {
        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static unsafe void CheckColliderTypeAndThrow<ExpectedType>(ColliderType type)
            where ExpectedType : ICollider
        {
            var dummyHeader = ColliderHeaderFakes.GetHeaderForColliderType<ExpectedType>();
            if (ColliderHeaderFakes.IsConvex(type) && dummyHeader.Type == ColliderType.Convex)
                return; //Box,Capsule,Sphere,Cylinder etc conversion to ConvexCollider

            if (dummyHeader.Type == ColliderHeaderFakes.k_AbstractType)
                return; //Collider to Collider type conversion

            if (dummyHeader.Type == ColliderType.Triangle && type == ColliderType.Quad)
                dummyHeader.Type = ColliderType.Quad; //Triangle/Quad share the same type, PolygonCollider

            if (dummyHeader.Type != type)
                throw new Exception($"Collider types do not match. Expected {dummyHeader.Type}, but was {type}.");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static unsafe void Check4ByteAlignmentAndThrow(void* data, in FixedString32Bytes paramName)
        {
            if (((long)data & 0x3) != 0)
                throw new InvalidOperationException($"{paramName} must be 4-byte aligned.");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckAreEqualAndThrow(SimulationType expected, SimulationType actual)
        {
            if (actual != expected)
                throw new ArgumentException($"Simulation type {actual} is not supported. This method should only be called when using {expected}.");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckFiniteAndThrow(fp3 value, FixedString32Bytes paramName)
        {
            if (math.any(!fpmath.isfinite(value)))
                throw new ArgumentException($"{value} was not finite.", $"{paramName}");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckFiniteAndPositiveAndThrow(fp3 value, in FixedString32Bytes paramName)
        {
            if (math.any(value < fp.zero) || math.any(!fpmath.isfinite(value)))
                throw new ArgumentOutOfRangeException($"{paramName}", $"{value} is not positive and finite.");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckIndexAndThrow(int index, int length, int min = 0)
        {
            if (index < min || index >= length)
                throw new IndexOutOfRangeException($"Index {index} is out of range [{min}, {length}].");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckInRangeAndThrow(int value, int2 range, in FixedString32Bytes paramName)
        {
            if (value < range.x || value > range.y)
                throw new ArgumentOutOfRangeException($"{paramName}", $"{value} is out of range [{range.x}, {range.y}].");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckNotEmptyAndThrow<T>(NativeArray<T> array, in FixedString32Bytes paramName) where T : struct
        {
            if (!array.IsCreated || array.Length == 0)
                throw new ArgumentException("Array is empty.", $"{paramName}");
        }

        #region Geometry Validation

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckCoplanarAndThrow(fp3 vertex0, fp3 vertex1, fp3 vertex2, fp3 vertex3, in FixedString32Bytes paramName)
        {
            var normal = fpmath.normalize(fpmath.cross(vertex1 - vertex0, vertex2 - vertex0));
            if (fpmath.abs(fpmath.dot(normal, vertex3 - vertex0)) > fp.FromRaw(0x3a83126f))
                throw new ArgumentException("Vertices are not co-planar", $"{paramName}");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckTriangleIndicesInRangeAndThrow(NativeArray<int3> triangles, int numVertices, in FixedString32Bytes paramName)
        {
            for (var i = 0; i < triangles.Length; ++i)
            {
                if (math.any(triangles[i] < 0) || math.any(triangles[i] >= numVertices))
                    throw new ArgumentException($"{paramName}", $"Triangle {triangles[i]} contained index out of range [0, {numVertices - 1}]");
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Geometry_CheckFiniteAndThrow(fp3 value, in FixedString32Bytes paramName, in FixedString32Bytes propertyName)
        {
            if (math.any(!fpmath.isfinite(value)))
                throw new ArgumentException($"{propertyName} {value} was not finite.", $"{paramName}");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Geometry_CheckFiniteAndPositiveAndThrow(fp value, in FixedString32Bytes paramName, in FixedString32Bytes propertyName)
        {
            if (value < fp.zero || !fpmath.isfinite(value))
                throw new ArgumentException($"{propertyName} {value} is not positive.", $"{paramName}");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Geometry_CheckFiniteAndPositiveAndThrow(fp3 value, in FixedString32Bytes paramName, in FixedString32Bytes propertyName)
        {
            if (math.any(value < fp.zero) || math.any(!fpmath.isfinite(value)))
                throw new ArgumentException($"{paramName}", $"{propertyName} {value} is not positive.");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Geometry_CheckValidAndThrow(fpquaternion q, in FixedString32Bytes paramName, in FixedString32Bytes propertyName)
        {
            if (q.Equals(default) || math.any(!fpmath.isfinite(q.value)))
                throw new ArgumentException($"{propertyName} {q} is not valid.", $"{paramName}");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckValidAndThrow(NativeArray<fp3> points, in FixedString32Bytes pointsName, in ConvexHullGenerationParameters generationParameters, in FixedString32Bytes paramName)
        {
            Geometry_CheckFiniteAndPositiveAndThrow(generationParameters.BevelRadius, paramName, nameof(ConvexHullGenerationParameters.BevelRadius));

            for (int i = 0, count = points.Length; i < count; ++i)
                CheckFiniteAndThrow(points[i], pointsName);
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckValidAndThrow(in BoxGeometry geometry, in FixedString32Bytes paramName)
        {
            Geometry_CheckFiniteAndThrow(geometry.Center, paramName, nameof(BoxGeometry.Center));
            Geometry_CheckValidAndThrow(geometry.Orientation, paramName, nameof(BoxGeometry.Orientation));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Size, paramName, nameof(BoxGeometry.Size));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.BevelRadius, paramName, nameof(BoxGeometry.BevelRadius));
            if (geometry.BevelRadius < fp.zero || geometry.BevelRadius > fpmath.cmin(geometry.Size) * fp.half)
                throw new ArgumentException($"{paramName}", $"{nameof(BoxGeometry.BevelRadius)} must be greater than or equal to and);less than or equal to half the smallest size dimension.");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckValidAndThrow(in CapsuleGeometry geometry, in FixedString32Bytes paramName)
        {
            Geometry_CheckFiniteAndThrow(geometry.Vertex0, paramName, nameof(CapsuleGeometry.Vertex0));
            Geometry_CheckFiniteAndThrow(geometry.Vertex1, paramName, nameof(CapsuleGeometry.Vertex1));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Radius, paramName, nameof(CapsuleGeometry.Radius));
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckValidAndThrow(in CylinderGeometry geometry, in FixedString32Bytes paramName)
        {
            Geometry_CheckFiniteAndThrow(geometry.Center, paramName, nameof(CylinderGeometry.Center));
            Geometry_CheckValidAndThrow(geometry.Orientation, paramName, nameof(CylinderGeometry.Orientation));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Height, paramName, nameof(CylinderGeometry.Height));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Radius, paramName, nameof(CylinderGeometry.Radius));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.BevelRadius, paramName, nameof(CylinderGeometry.BevelRadius));
            if (geometry.BevelRadius < fp.zero || geometry.BevelRadius > fpmath.min(geometry.Height * fp.half, geometry.Radius))
                throw new ArgumentException($"{paramName}", $"{nameof(CylinderGeometry.BevelRadius)} must be greater than or equal to 0 and less than or equal to half the smallest size dimension.");
            if (geometry.SideCount < CylinderGeometry.MinSideCount || geometry.SideCount > CylinderGeometry.MaxSideCount)
                throw new ArgumentException($"{paramName}", $"{nameof(CylinderGeometry.SideCount)} must be greater than or equal to {CylinderGeometry.MinSideCount} and less than or equal to {CylinderGeometry.MaxSideCount}.");
        }

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void CheckValidAndThrow(in SphereGeometry geometry, in FixedString32Bytes paramName)
        {
            Geometry_CheckFiniteAndThrow(geometry.Center, paramName, nameof(SphereGeometry.Center));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Radius, paramName, nameof(SphereGeometry.Radius));
        }

        #endregion

        #region Throw Exceptions

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void ThrowInvalidOperationException(FixedString128Bytes message = default) => throw new InvalidOperationException($"{message}");

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void ThrowNotImplementedException() => throw new NotImplementedException();

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void ThrowNotSupportedException(FixedString64Bytes message = default) => throw new NotSupportedException($"{message}");

        [Conditional(CompilationSymbols.SafetyChecksSymbol)]
        public static void ThrowArgumentException(in FixedString32Bytes paramName, FixedString64Bytes message = default) =>
            throw new ArgumentException($"{message}", $"{paramName}");

        #endregion
    }
}

using System;
using Unity.Entities;
using Fixed.Mathematics;

namespace Fixed.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToWorld))]
    [WriteGroup(typeof(LocalToParent))]
    [WriteGroup(typeof(CompositeScale))]
    [WriteGroup(typeof(ParentScaleInverse))]
    public struct NonUniformScale : IComponentData
    {
        public float3 Value;
    }
}

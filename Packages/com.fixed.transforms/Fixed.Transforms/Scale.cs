using System;
using Unity.Entities;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToWorld))]
    [WriteGroup(typeof(LocalToParent))]
    [WriteGroup(typeof(CompositeScale))]
    [WriteGroup(typeof(ParentScaleInverse))]
    public struct Scale : IComponentData
    {
        public fp Value;
    }
}

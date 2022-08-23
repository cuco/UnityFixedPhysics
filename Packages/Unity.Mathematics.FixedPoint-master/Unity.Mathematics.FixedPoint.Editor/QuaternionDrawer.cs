using UnityEditor;
using Unity.Mathematics.Editor;

namespace Unity.Mathematics.FixedPoint.Editor
{
    // [CustomPropertyDrawer(typeof(quaternion))]
    // class QuaternionDrawer : PostNormalizedVectorDrawer
    // {
    //     protected override SerializedProperty GetVectorProperty(SerializedProperty property)
    //     {
    //         return property.FindPropertyRelative("value");
    //     }
    //
    //     protected override double4 Normalize(double4 value)
    //     {
    //         return math.normalizesafe(new quaternion((float4)value)).value;
    //     }
    // }
}

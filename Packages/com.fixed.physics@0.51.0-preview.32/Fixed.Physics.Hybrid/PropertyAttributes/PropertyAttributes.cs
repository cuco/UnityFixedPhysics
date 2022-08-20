using UnityEngine;

namespace Fixed.Physics.Authoring
{
    sealed class EnumFlagsAttribute : PropertyAttribute {}
    sealed class ExpandChildrenAttribute : PropertyAttribute {}
    sealed class SoftRangeAttribute : PropertyAttribute
    {
        public readonly sfloat SliderMin;
        public readonly sfloat SliderMax;
        public sfloat TextFieldMin { get; set; }
        public sfloat TextFieldMax { get; set; }

        public SoftRangeAttribute(sfloat min, sfloat max)
        {
            SliderMin = TextFieldMin = min;
            SliderMax = TextFieldMax = max;
        }
    }
}

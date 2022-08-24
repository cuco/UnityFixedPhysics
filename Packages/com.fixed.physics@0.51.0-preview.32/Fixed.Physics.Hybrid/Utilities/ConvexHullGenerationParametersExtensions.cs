using System;
using Unity.Collections;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics.Authoring
{
    public static class ConvexHullGenerationParametersExtensions
    {
        // recommended simplification tolerance is at least 1 centimeter
        //TODO 0.01f
        internal static readonly fp k_MinRecommendedSimplificationTolerance = fp.one / (fp)100;

        internal static void InitializeToRecommendedAuthoringValues(
            ref this ConvexHullGenerationParameters generationParameters, NativeArray<fp3> points
        )
        {
            generationParameters = ConvexHullGenerationParameters.Default.ToAuthoring();

            if (points.Length <= 1)
                return;

            var bounds = new Aabb { Min = points[0], Max = points[0] };
            for (var i = 1; i < points.Length; ++i)
                bounds.Include(points[i]);
            generationParameters.SimplificationTolerance = fpmath.max(
                k_MinRecommendedSimplificationTolerance,
                ConvexHullGenerationParameters.Default.SimplificationTolerance * fpmath.cmax(bounds.Extents)
            );
            // TODO: initialize other properties based on input points?
        }

        internal static void OnValidate(ref this ConvexHullGenerationParameters generationParameters, fp maxAngle /* = 180f*/)
        {
            generationParameters.SimplificationTolerance = fpmath.max(fp.zero, generationParameters.SimplificationTolerance);
            generationParameters.BevelRadius = fpmath.max(fp.zero, generationParameters.BevelRadius);
            generationParameters.MinimumAngle = fpmath.clamp(generationParameters.MinimumAngle, fp.zero, maxAngle);
        }

        public static ConvexHullGenerationParameters ToAuthoring(this ConvexHullGenerationParameters generationParameters)
        {
            generationParameters.MinimumAngle = fpmath.degrees(generationParameters.MinimumAngle);
            return generationParameters;
        }

        public static ConvexHullGenerationParameters ToRunTime(this ConvexHullGenerationParameters generationParameters)
        {
            generationParameters.MinimumAngle = fpmath.radians(generationParameters.MinimumAngle);
            return generationParameters;
        }
    }
}

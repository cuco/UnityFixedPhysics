using Unity.Entities;
using Fixed.Physics.Systems;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics.GraphicsIntegration
{
    /// <summary>
    /// A system to keep track of the time values in the most recent tick of the <c>FixedStepSimulationSystemGroup</c>.
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(ExportPhysicsWorld))]
    [AlwaysUpdateSystem]
    public partial class RecordMostRecentFixedTime : SystemBase
    {
        /// <summary>
        /// The value of <c>Time.ElapsedTime</c> in the most recent tick of the <c>FixedStepSimulationSystemGroup</c>.
        /// </summary>
        public fp MostRecentElapsedTime { get; private set; }

        /// <summary>
        /// The value of <c>Time.DeltaTime</c> in the most recent tick of the <c>FixedStepSimulationSystemGroup</c>.
        /// </summary>
        public fp MostRecentDeltaTime { get; private set; }

        protected override void OnUpdate()
        {
            MostRecentElapsedTime = (fp)Time.ElapsedTime;
            MostRecentDeltaTime = (fp)Time.DeltaTime;
        }
    }
}

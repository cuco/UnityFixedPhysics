using Unity.Entities;
using Fixed.Physics.Systems;

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
        public sfloat MostRecentElapsedTime { get; private set; }

        /// <summary>
        /// The value of <c>Time.DeltaTime</c> in the most recent tick of the <c>FixedStepSimulationSystemGroup</c>.
        /// </summary>
        public sfloat MostRecentDeltaTime { get; private set; }

        protected override void OnUpdate()
        {
            MostRecentElapsedTime = (sfloat)Time.ElapsedTime;
            MostRecentDeltaTime = (sfloat)Time.DeltaTime;
        }
    }
}

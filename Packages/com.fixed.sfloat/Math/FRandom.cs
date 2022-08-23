using System;

namespace Morefun.LockStep
{
	public class FRandom
    {
        // Simple Linear congruential generator
        // with glibc's parameters for 31 bit integers
        // https://en.wikipedia.org/wiki/Linear_congruential_generator#Parameters_in_common_use
        //TODO 改为了public
        public int seed;
        private const uint a = 1103515245;
        private const uint c = 12345;
        private const uint mask = 0x7FFFFFFF;

        public FRandom(int seed)
        {
            LockstepDebug.Assert(seed >= 0);
            this.seed = seed;
        }

        /// <summary>
        /// Gets a random int between zero and int.maxValue inclusively.
        /// </summary>
        public int Next()
        {
            seed = (int)((a * (uint)seed + c) & mask);
            return seed;
        }

        /// <summary>
        /// Gets a random int between zero to maximum value inclusively.
        /// </summary>
        /// <param name="maxValue">Maximum value. Must >= 0.</param>
        public int Next(int maxValue)
        {
            //LockstepDebug.Assert(maxValue >= 0);
            if (maxValue == 0) return 0;
            return Next() % (maxValue);
        }

        public int Current()
        {
            return seed;
        }

        /// <summary>
        /// Gets a random int between minimum and maximum value inclusively.
        /// </summary>
        /// <param name="minValue">Minimum value.</param>
        /// <param name="maxValue">Maximum value.</param>
        public int Next(int minValue, int maxValue)
        {
            LockstepDebug.Assert(minValue <= maxValue);
            return minValue + Next(maxValue - minValue);
        }

        /// <summary>
        /// Gets a random FScalar value in [0, 1).
        /// </summary>
        /// <remarks>It has only limited precision. For generating a larger range of output, use NextScalar(FScalar) or NextScalar(FScalar, FScalar)</remarks>
        public FScalar NextFScalar()
        {
            return FScalar.FromRaw((int)((uint)Next() & FScalar.fractionMask));
        }

        /// <summary>
        /// Gets a random FScalar value in [-1,1) by Gaussian Rand
        /// 模拟正态分布获取一个-1~1的随机数
        /// </summary>
        /// <returns></returns>
        public FScalar NextGaussianRand()
        {
            int holdseed = seed;
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            int r = (holdseed + seed);
            return FScalar.FromRaw(r) / FScalar.maxValue;
        }
        
        /**
         * 随机[0, 1)，与0.5比较
         */
        public bool NextBool()
        {
            return NextFScalar() > FScalar.half;
        }

        /// <summary>
        /// Gets a random int between zero to maximum value inclusively.
        /// </summary>
        /// <param name="maxValue">Maximum value. Must >= 0.</param>
        public FScalar NextFScalar(FScalar maxValue)
        {
            LockstepDebug.Assert(maxValue >= FScalar.zero);
            return FScalar.FromRaw((int)(((long)Next() * (maxValue.rawValue + 1)) >> 31));
        }

        /// <summary>
        /// Gets a random FScalar between minimum and maximum value inclusively.
        /// </summary>
        /// <param name="minValue">Minimum value.</param>
        /// <param name="maxValue">Maximum value.</param>
        public FScalar NextFScalar(FScalar minValue, FScalar maxValue)
        {
            LockstepDebug.Assert(minValue <= maxValue);
            return minValue + NextFScalar(maxValue - minValue);
        }
        
// 		public void WriteGameState(IGameStateWriter writer)
// 		{
// //			writer.StartObject();
// 			writer.Key("seed");
// 			writer.Int (seed);
// //			writer.EndObject();
// 		}
    }
}

using System;
using System.Collections.Generic;

namespace Morefun.LockStep
{
    [Serializable]
    public struct FKeyframe
    {
        public FScalar time;
        public FScalar value;
        public FScalar inTangent;
        public FScalar outTangent;

        public FKeyframe(FScalar time, FScalar value)
        {
            this.time = time;
            this.value = value;
            this.inTangent = FScalar.zero;
            this.outTangent = FScalar.zero;
        }

        public FKeyframe(FScalar time, FScalar value, FScalar inTangent, FScalar outTangent)
        {
            this.time = time;
            this.value = value;
            this.inTangent = inTangent;
            this.outTangent = outTangent;
        }
    }

    [Serializable]
    public enum FWrapMode
    {
        ClampForever,
        Loop,
        PingPong
    }

    /// <summary>
    /// 对应Unity的AnimationCurve
    /// </summary>
    [Serializable]
	public class FAnimationCurve
    {
        // keys are not sorted automatically
        public List<FKeyframe> keyList = new List<FKeyframe>();
        public FWrapMode preWrapMode = FWrapMode.ClampForever;
        public FWrapMode postWrapMode = FWrapMode.ClampForever;

        public int length => keyList.Count;

        // keys are not sorted automatically
        public FKeyframe this[int index]
        {
            get => keyList[index];
            set => keyList[index] = value;
        }

        public FKeyframe[] keys
        {
            get => keyList.ToArray();
            set => keyList = new List<FKeyframe>(value);
        }

        public FAnimationCurve()
        {
            keyList = new List<FKeyframe>();
        }

        // keys are not sorted automatically
        public FAnimationCurve(params FKeyframe[] keys)
        {
            keyList = new List<FKeyframe>(keys);
        }

        // "Smooth tangents are automatically computed for the key" is not implemented.
        // If no key could be added because there is already another keyframe at the same time -1 will be returned.
        public int AddKey(FScalar time, FScalar value)
        {
            return AddKey(new FKeyframe(time, value));
        }

        // If no key could be added because there is already another keyframe at the same time -1 will be returned.
        public int AddKey(FKeyframe key)
        {
            for (int i = 0; i < keyList.Count; i++)
            {
                if (key.time == keyList[i].time)
                    return -1;
                
                if (key.time < keyList[i].time)
                {
                    keyList.Insert(i, key);
                    return i;
                }
            }

            keyList.Add(key);
            return keyList.Count - 1;
        }

        public int MoveKey(int index, FKeyframe key)
        {
            if ((index > 0 && key.time < keyList[index - 1].time) || 
                (index < keyList.Count - 1 && key.time > keyList[index - 1].time))
            {
                RemoveKey(index);
                return AddKey(key);
            }

            keyList[index] = key;
            return index;
        }

        public void RemoveKey(int index)
        {
            ((IList<FKeyframe>)keyList).RemoveAt(index);
        }

        /// <summary>
        /// 增加一个线型
        /// </summary>
        /// <param name="timeStart"></param>
        /// <param name="valueStart"></param>
        /// <param name="timeEnd"></param>
        /// <param name="valueEnd"></param>
        /// <returns></returns>
        public static FAnimationCurve Linear(FScalar timeStart, FScalar valueStart, FScalar timeEnd, FScalar valueEnd)
        {
            FScalar outTangent = (valueEnd - valueStart) / (timeEnd - timeStart);
            FKeyframe[] keys = new FKeyframe[] { new FKeyframe(timeStart, valueStart, 0, outTangent), new FKeyframe(timeEnd, valueEnd, outTangent, 0) };
            return new FAnimationCurve(keys);
        }

        public FScalar Evaluate(FScalar time)
        {
            if (keyList == null || keyList.Count == 0)
                return FScalar.zero;

            if (keyList.Count == 1)
                return keyList[0].value;

            // Shift time to the first key
            time -= keyList[0].time;
            FScalar period = keyList[keyList.Count - 1].time - keyList[0].time;

            if (time <= FScalar.zero)
            {
                switch (preWrapMode)
                {
                    case FWrapMode.Loop:
                        time = period + time % period;
                        if (time >= period)
                            time = FScalar.zero;
                        break;

                    case FWrapMode.PingPong:
                        time = period - FScalar.Abs((-time) % (period * 2) - period);
                        break;

                    case FWrapMode.ClampForever:
                        return keyList[0].value;
                }
            }
            else if (time >= period)
            {
                switch (postWrapMode)
                {
                    case FWrapMode.Loop:
                        time = time % period;
                        break;

                    case FWrapMode.PingPong:
                        time = period - FScalar.Abs(time % (period * 2) - period);
                        break;

                    case FWrapMode.ClampForever:
                        return keyList[keyList.Count - 1].value;
                }
            }

            // Shift back
            time += keyList[0].time;

            if (time == keyList[0].time)
                return keyList[0].value;
            if (time == keyList[keyList.Count - 1].time)
                return keyList[keyList.Count - 1].value;

            // Find in-between keyframes index - 1, index
            // TODO: binary search
            int index = 1;
            for (; index < keyList.Count; index++)
                if (time <= keyList[index].time)
                    break;

            // http://answers.unity3d.com/answers/508835/view.html
            FKeyframe f0 = keyList[index - 1];
            FKeyframe f1 = keyList[index];

            FScalar dt = f1.time - f0.time;
            FScalar m0 = f0.outTangent * dt;
            FScalar m1 = f1.inTangent * dt;

            FScalar t = (time - f0.time) / dt;
            FScalar t2 = t * t;
            FScalar t3 = t2 * t;

            FScalar a = 2 * t3 - 3 * t2 + 1;
            FScalar b = t3 - 2 * t2 + t;
            FScalar c = t3 - t2;
            FScalar d = -2 * t3 + 3 * t2;

            return a * f0.value + b * m0 + c * m1 + d * f1.value;
        }
    }
}

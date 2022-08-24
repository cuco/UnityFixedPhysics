using System;
using Unity.Mathematics.FixedPoint;

namespace Fixed.DebugDisplay
{
    internal struct Data
    {
        internal int offset;
        internal int length;

        internal void Validate()
        {
            if ((length & (length - 1)) != 0)
                throw new ArgumentException(
                    $"Length of data in graph is {length} which must be a power of two.");
        }

        public static implicit operator Data(Unit unit)
        {
            return new Data { offset = unit.m_Begin, length = unit.Length };
        }
    }

    internal struct GraphBuffer : IDisposable
    {
        internal Unit ReserveAllData()
        {
            return new Unit(0, 1, 0, m_Data.Length);
        }

        internal struct Sample
        {
            internal Data data; // offset of first datum, count of data
            internal ColorIndex ColorIndex; // color of data
            internal fp xMin; // first data to display
            internal fp xMax; // last data to display
            internal fp yMin; // first Y value to display
            internal fp yMax; // last Y value to display
        }

        internal struct InstanceSample
        {
            internal int color; // color of the sample
            internal int firstIndex; // first sample index in range to display
            internal int indexMask; // AND sample index with this to make it wrap around
            internal fp indexMul; // multiply the pixel.x by this,
            internal fp indexAdd; // and then by this to get the sample index.
            internal fp sampleMul; // multiply the sample by this,
            internal fp sampleAdd; // and then add this to get the pixel.y
        }

        internal struct Instance
        {
            internal fp2 screenPosition;
            internal fp2 cellSize;
            internal int frameColor;
            internal int samples;
            internal InstanceSample sample0;
            internal InstanceSample sample1;
        };

        internal const int kMaxInstances = 16;
        internal const int kMaxValues = 4096;
        internal const int kMaxColors = 16;
        internal UnsafeArray<Instance> m_Instance;
        internal UnsafeArray<fp> m_Data;

        internal void Initialize()
        {
            m_Instance = new UnsafeArray<Instance>(kMaxInstances);
            m_Data = new UnsafeArray<fp>(kMaxValues);
        }

        fp recip(fp f)
        {
            return (f == fp.zero) ? fp.one : fp.one / f;
        }

        internal void ClearGraph(int index)
        {
            m_Instance[index] = new Instance();
        }

        internal void SetGraph(int x, int y, int w, int h, Sample a, int index)
        {
            if (index >= m_Instance.Length)
                return;
            a.data.Validate();

            a.xMax += fp.one;

            fp axScale = (a.xMax - a.xMin) / ((fp)(w * 8) - fp.one);
            fp ayScale = (fp)(h * 16 - 2) * recip(a.yMin - a.yMax);

            m_Instance[index] = new Instance
            {
                screenPosition = new fp2((fp)(x * Cell.kPixelsWide), (fp)(y * Cell.kPixelsTall)),
                cellSize = new fp2((fp)w, (fp)h),
                frameColor = ColorIndex.White.value,
                samples = 1,
                sample0 = new InstanceSample
                {
                    color = a.ColorIndex.value,
                    firstIndex = a.data.offset,
                    indexMask = a.data.length - 1,
                    indexMul = axScale,
                    indexAdd = a.xMin,
                    sampleMul = ayScale,
                    sampleAdd = ayScale * -a.yMax + fp.one,
                }
            };
        }

        internal unsafe void SetGraph(int x, int y, int w, int h, Sample a, Sample b, int index)
        {
            if (index >= m_Instance.Length)
                return;
            a.data.Validate();
            b.data.Validate();

            a.xMax += fp.one;
            b.xMax += fp.one;

            fp axScale = (a.xMax - a.xMin) / (fp)(w * 8 - 1);
            fp ayScale = (fp)(h * 16 - 2) * recip(a.yMin - a.yMax);
            fp bxScale = (b.xMax - b.xMin) / (fp)(w * 8 - 1);
            fp byScale = (fp)(h * 16 - 2) * recip(b.yMin - b.yMax);

            m_Instance[index] = new Instance
            {
                screenPosition = new fp2((fp)(x * Cell.kPixelsWide), (fp)(y * Cell.kPixelsTall)),
                cellSize = new fp2((fp)w, (fp)h),
                frameColor = ColorIndex.White.value,
                samples = 2,
                sample0 = new InstanceSample
                {
                    color = a.ColorIndex.value,
                    firstIndex = a.data.offset,
                    indexMask = a.data.length - 1,
                    indexMul = axScale,
                    indexAdd = a.xMin,
                    sampleMul = ayScale,
                    sampleAdd = ayScale * -a.yMax + fp.one,
                },
                sample1 = new InstanceSample
                {
                    color = b.ColorIndex.value,
                    firstIndex = b.data.offset,
                    indexMask = b.data.length - 1,
                    indexMul = bxScale,
                    indexAdd = b.xMin,
                    sampleMul = byScale,
                    sampleAdd = byScale * -b.yMax + fp.one,
                }
            };
        }

        public void Dispose()
        {
            m_Instance.Dispose();
            m_Data.Dispose();
        }

        internal Unit AllocateAll()
        {
            return new Unit(0, 1, 0, m_Instance.Length);
        }
    }
}

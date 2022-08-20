using System;
using Fixed.Mathematics;

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
            internal sfloat xMin; // first data to display
            internal sfloat xMax; // last data to display
            internal sfloat yMin; // first Y value to display
            internal sfloat yMax; // last Y value to display
        }

        internal struct InstanceSample
        {
            internal int color; // color of the sample
            internal int firstIndex; // first sample index in range to display
            internal int indexMask; // AND sample index with this to make it wrap around
            internal sfloat indexMul; // multiply the pixel.x by this,
            internal sfloat indexAdd; // and then by this to get the sample index.
            internal sfloat sampleMul; // multiply the sample by this,
            internal sfloat sampleAdd; // and then add this to get the pixel.y
        }

        internal struct Instance
        {
            internal float2 screenPosition;
            internal float2 cellSize;
            internal int frameColor;
            internal int samples;
            internal InstanceSample sample0;
            internal InstanceSample sample1;
        };

        internal const int kMaxInstances = 16;
        internal const int kMaxValues = 4096;
        internal const int kMaxColors = 16;
        internal UnsafeArray<Instance> m_Instance;
        internal UnsafeArray<sfloat> m_Data;

        internal void Initialize()
        {
            m_Instance = new UnsafeArray<Instance>(kMaxInstances);
            m_Data = new UnsafeArray<sfloat>(kMaxValues);
        }

        sfloat recip(sfloat f)
        {
            return (f == sfloat.Zero) ? sfloat.One : sfloat.One / f;
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

            a.xMax += sfloat.One;

            sfloat axScale = (a.xMax - a.xMin) / ((sfloat)(w * 8) - sfloat.One);
            sfloat ayScale = (sfloat)(h * 16 - 2) * recip(a.yMin - a.yMax);

            m_Instance[index] = new Instance
            {
                screenPosition = new float2((sfloat) (x * Cell.kPixelsWide), (sfloat) (y * Cell.kPixelsTall)),
                cellSize = new float2((sfloat)w, (sfloat)h),
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
                    sampleAdd = ayScale * -a.yMax + sfloat.One,
                }
            };
        }

        internal unsafe void SetGraph(int x, int y, int w, int h, Sample a, Sample b, int index)
        {
            if (index >= m_Instance.Length)
                return;
            a.data.Validate();
            b.data.Validate();

            a.xMax += sfloat.One;
            b.xMax += sfloat.One;

            sfloat axScale = (a.xMax - a.xMin) / (sfloat)(w * 8 - 1);
            sfloat ayScale = (sfloat)(h * 16 - 2) * recip(a.yMin - a.yMax);
            sfloat bxScale = (b.xMax - b.xMin) / (sfloat)(w * 8 - 1);
            sfloat byScale = (sfloat)(h * 16 - 2) * recip(b.yMin - b.yMax);

            m_Instance[index] = new Instance
            {
                screenPosition = new float2((sfloat) (x * Cell.kPixelsWide), (sfloat) (y * Cell.kPixelsTall)),
                cellSize = new float2((sfloat)w, (sfloat)h),
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
                    sampleAdd = ayScale * -a.yMax + sfloat.One,
                },
                sample1 = new InstanceSample
                {
                    color = b.ColorIndex.value,
                    firstIndex = b.data.offset,
                    indexMask = b.data.length - 1,
                    indexMul = bxScale,
                    indexAdd = b.xMin,
                    sampleMul = byScale,
                    sampleAdd = byScale * -b.yMax + sfloat.One,
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

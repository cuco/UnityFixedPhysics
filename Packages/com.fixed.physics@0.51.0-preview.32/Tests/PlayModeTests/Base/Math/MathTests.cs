using NUnit.Framework;
using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics.FixedPoint;
using static Fixed.Physics.Math;

namespace Fixed.Physics.Tests.Base.Math
{
    class MathTests
    {
        [BurstCompile(CompileSynchronously = true)]
        private struct IndexOfMinComponent2Job : IJob, IDisposable
        {
            [ReadOnly]
            public NativeArray<fp2> In;
            [WriteOnly]
            public NativeArray<int> Out;

            public static IndexOfMinComponent2Job Init()
            {
                var length = 4;
                var job = new IndexOfMinComponent2Job
                {
                    In = new NativeArray<fp2>(length, Allocator.TempJob),
                    Out = new NativeArray<int>(length, Allocator.TempJob)
                };

                job.In[0] = new fp2(-(fp)1, (fp)1);
                job.In[1] = new fp2((fp)1, -(fp)1);
                job.In[2] = new fp2(fp.NaN, (fp)1);
                job.In[3] = new fp2(fp.PositiveInfinity, fp.NegativeInfinity);

                return job;
            }

            public void Dispose()
            {
                Assert.AreEqual(0, Out[0]);
                Assert.AreEqual(1, Out[1]);
                Assert.AreEqual(1, Out[2]);
                Assert.AreEqual(1, Out[3]);

                In.Dispose();
                Out.Dispose();
            }

            public void Execute()
            {
                for (int i = 0; i < In.Length; i++)
                {
                    Out[i] = IndexOfMinComponent(In[i]);
                }
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct IndexOfMinComponent3Job : IJob, IDisposable
        {
            [ReadOnly]
            public NativeArray<fp3> In;
            [WriteOnly]
            public NativeArray<int> Out;

            public static IndexOfMinComponent3Job Init()
            {
                var length = 4;
                var job = new IndexOfMinComponent3Job
                {
                    In = new NativeArray<fp3>(length, Allocator.TempJob),
                    Out = new NativeArray<int>(length, Allocator.TempJob)
                };

                job.In[0] = new fp3(-(fp)1, (fp)1, (fp)1);
                job.In[1] = new fp3((fp)1, -(fp)1, (fp)1);
                job.In[2] = new fp3((fp)1, (fp)1, -(fp)1);
                job.In[3] = new fp3(fp.NaN, fp.PositiveInfinity, fp.NegativeInfinity);

                return job;
            }

            public void Dispose()
            {
                Assert.AreEqual(0, Out[0]);
                Assert.AreEqual(1, Out[1]);
                Assert.AreEqual(2, Out[2]);
                Assert.AreEqual(2, Out[3]);

                In.Dispose();
                Out.Dispose();
            }

            public void Execute()
            {
                for (int i = 0; i < In.Length; i++)
                {
                    Out[i] = IndexOfMinComponent(In[i]);
                }
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct IndexOfMinComponent4Job : IJob, IDisposable
        {
            [ReadOnly]
            public NativeArray<fp4> In;
            [WriteOnly]
            public NativeArray<int> Out;

            public static IndexOfMinComponent4Job Init()
            {
                var length = 5;
                var job = new IndexOfMinComponent4Job
                {
                    In = new NativeArray<fp4>(length, Allocator.TempJob),
                    Out = new NativeArray<int>(length, Allocator.TempJob)
                };

                job.In[0] = new fp4(-(fp)1, (fp)1, (fp)1, (fp)1);
                job.In[1] = new fp4((fp)1, -(fp)1, (fp)1, (fp)1);
                job.In[2] = new fp4((fp)1, (fp)1, -(fp)1, (fp)1);
                job.In[3] = new fp4((fp)1, (fp)1, (fp)1, -(fp)1);
                job.In[4] = new fp4(fp.NaN, fp.PositiveInfinity, fp.NegativeInfinity, -fp.NaN);

                return job;
            }

            public void Dispose()
            {
                Assert.AreEqual(0, Out[0]);
                Assert.AreEqual(1, Out[1]);
                Assert.AreEqual(2, Out[2]);
                Assert.AreEqual(3, Out[3]);
                Assert.AreEqual(3, Out[4]); // NaNs throw the result!

                In.Dispose();
                Out.Dispose();
            }

            public void Execute()
            {
                for (int i = 0; i < In.Length; i++)
                {
                    Out[i] = IndexOfMinComponent(In[i]);
                }
            }
        }

        [Test]
        public void TestIndexOfMinComponent()
        {
            var indexOfMinComponent2Job = IndexOfMinComponent2Job.Init();
            indexOfMinComponent2Job.Run();
            indexOfMinComponent2Job.Dispose();

            var indexOfMinComponent3Job = IndexOfMinComponent3Job.Init();
            indexOfMinComponent3Job.Run();
            indexOfMinComponent3Job.Dispose();

            var indexOfMinComponent4Job = IndexOfMinComponent4Job.Init();
            indexOfMinComponent4Job.Run();
            indexOfMinComponent4Job.Dispose();
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct IndexOfMaxComponent2Job : IJob, IDisposable
        {
            [ReadOnly]
            public NativeArray<fp2> In;
            [WriteOnly]
            public NativeArray<int> Out;

            public static IndexOfMaxComponent2Job Init()
            {
                var length = 4;
                var job = new IndexOfMaxComponent2Job
                {
                    In = new NativeArray<fp2>(length, Allocator.TempJob),
                    Out = new NativeArray<int>(length, Allocator.TempJob)
                };

                job.In[0] = new fp2(-(fp)1, (fp)1);
                job.In[1] = new fp2((fp)1, -(fp)1);
                job.In[2] = new fp2(fp.NaN, (fp)1);
                job.In[3] = new fp2(fp.PositiveInfinity, fp.NegativeInfinity);

                return job;
            }

            public void Dispose()
            {
                Assert.AreEqual(1, Out[0]);
                Assert.AreEqual(0, Out[1]);
                Assert.AreEqual(1, Out[2]);
                Assert.AreEqual(0, Out[3]);

                In.Dispose();
                Out.Dispose();
            }

            public void Execute()
            {
                for (int i = 0; i < In.Length; i++)
                {
                    Out[i] = IndexOfMaxComponent(In[i]);
                }
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct IndexOfMaxComponent3Job : IJob, IDisposable
        {
            [ReadOnly]
            public NativeArray<fp3> In;
            [WriteOnly]
            public NativeArray<int> Out;

            public static IndexOfMaxComponent3Job Init()
            {
                var length = 4;
                var job = new IndexOfMaxComponent3Job
                {
                    In = new NativeArray<fp3>(length, Allocator.TempJob),
                    Out = new NativeArray<int>(length, Allocator.TempJob)
                };

                job.In[0] = new fp3(-(fp)1, -(fp)1, (fp)1);
                job.In[1] = new fp3((fp)1, -(fp)1, -(fp)1);
                job.In[2] = new fp3(-(fp)1, (fp)1, -(fp)1);
                job.In[3] = new fp3(fp.NaN, fp.PositiveInfinity, fp.NegativeInfinity);

                return job;
            }

            public void Dispose()
            {
                Assert.AreEqual(2, Out[0]);
                Assert.AreEqual(0, Out[1]);
                Assert.AreEqual(1, Out[2]);
                Assert.AreEqual(1, Out[3]);

                In.Dispose();
                Out.Dispose();
            }

            public void Execute()
            {
                for (int i = 0; i < In.Length; i++)
                {
                    Out[i] = IndexOfMaxComponent(In[i]);
                }
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct IndexOfMaxComponent4Job : IJob, IDisposable
        {
            [ReadOnly]
            public NativeArray<fp4> In;
            [WriteOnly]
            public NativeArray<int> Out;

            public static IndexOfMaxComponent4Job Init()
            {
                var length = 5;
                var job = new IndexOfMaxComponent4Job
                {
                    In = new NativeArray<fp4>(length, Allocator.TempJob),
                    Out = new NativeArray<int>(length, Allocator.TempJob)
                };

                job.In[0] = new fp4(-(fp)1, -(fp)1, -(fp)1, (fp)1);
                job.In[1] = new fp4((fp)1, -(fp)1, -(fp)1, -(fp)1);
                job.In[2] = new fp4(-(fp)1, (fp)1, -(fp)1, -(fp)1);
                job.In[3] = new fp4(-(fp)1, -(fp)1, (fp)1, -(fp)1);
                job.In[4] = new fp4(fp.NaN, fp.PositiveInfinity, fp.NegativeInfinity, -fp.NaN);

                return job;
            }

            public void Dispose()
            {
                Assert.AreEqual(3, Out[0]);
                Assert.AreEqual(0, Out[1]);
                Assert.AreEqual(1, Out[2]);
                Assert.AreEqual(2, Out[3]);
                Assert.AreEqual(3, Out[4]); // NaNs throw the result!

                In.Dispose();
                Out.Dispose();
            }

            public void Execute()
            {
                for (int i = 0; i < In.Length; i++)
                {
                    Out[i] = IndexOfMaxComponent(In[i]);
                }
            }
        }

        [Test]
        public void TestIndexOfMaxComponent()
        {
            var indexOfMaxComponent2Job = IndexOfMaxComponent2Job.Init();
            indexOfMaxComponent2Job.Run();
            indexOfMaxComponent2Job.Dispose();

            var indexOfMaxComponent3Job = IndexOfMaxComponent3Job.Init();
            indexOfMaxComponent3Job.Run();
            indexOfMaxComponent3Job.Dispose();

            var indexOfMaxComponent4Job = IndexOfMaxComponent4Job.Init();
            indexOfMaxComponent4Job.Run();
            indexOfMaxComponent4Job.Dispose();
        }
    }
}

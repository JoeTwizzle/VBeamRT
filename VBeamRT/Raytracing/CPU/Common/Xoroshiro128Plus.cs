using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT.Raytracing.CPU.Common;
public sealed class Xoroshiro128Plus
{
    private ulong s0, s1;

    public Xoroshiro128Plus(ulong seed)
    {
        s0 = SplitMix64(ref seed);
        s1 = SplitMix64(ref seed);
    }

    private static ulong SplitMix64(ref ulong seed)
    {
        ulong z = (seed += 0x9E3779B97F4A7C15UL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private ulong NextULong()
    {
        ulong result = s0 + s1;
        ulong s1Copy = s1 ^ s0;
        s0 = (s0 << 55 | s0 >> (64 - 55)) ^ s1Copy ^ (s1Copy << 14);
        s1 = s1Copy << 36 | s1Copy >> (64 - 36);
        return result;
    }

    // 24-bit precision float in [0,1)
    public float NextFloat()
    {
        return (NextULong() >> 40) * (1.0f / (1UL << 24)); 
    }

    public Vec2 NextVec2()
    {
        return new Vec2(NextFloat(), NextFloat());
    }
}

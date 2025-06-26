using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT.Raytracing.CPU.Common;
public class PCG32
{
    private ulong state;
    private ulong inc;

    public PCG32(ulong seed, ulong sequence = 1)
    {
        state = 0;
        inc = (sequence << 1) | 1;
        NextUInt(); // Warm up
        state += seed;
        NextUInt();
    }

    public uint NextUInt()
    {
        ulong oldState = state;
        state = oldState * 6364136223846793005UL + inc;
        uint xorshifted = (uint)(((oldState >> 18) ^ oldState) >> 27);
        uint rot = (uint)(oldState >> 59);
        return (xorshifted >> (int)rot) | (xorshifted << ((int)(-rot) & 31));
    }

    public float NextFloat()
    {
        return NextUInt() * (1.0f / 4294967296.0f); // [0, 1)
    }

    public Vec2 NextVec2()
    {
        return new Vec2(NextFloat(), NextFloat());
    }

    public Vec3 NextVec3()
    {
        return new Vec3(NextFloat(), NextFloat(), NextFloat());
    }

    public float NextFloat(float min, float max)
    {
        return min + (max - min) * NextFloat();
    }
}

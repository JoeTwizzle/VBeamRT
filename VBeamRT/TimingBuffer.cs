using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT;

public sealed class TimingBuffer
{
    readonly double[] _values;
    int samples;
    int head = 0;

    public TimingBuffer(int maxSampleCount)
    {
        _values = new double[maxSampleCount];
        samples = 0;
    }

    public void Add(double value)
    {
        if (samples < _values.Length)
        {
            samples++;
        }
        _values[head++] = value;
        if (head >= _values.Length)
        {
            head = 0;
        }
    }

    public double Average()
    {
        double average = 0;
        for (int i = 0; i < samples; i++)
        {
            average = (average * i + _values[i]) / (i + 1);
        }
        return average;
    }
}

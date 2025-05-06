using OpenTK.Platform;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT;
public sealed class Metrics
{
    public double DeltaTimeFull;
    public float DeltaTime;
    public readonly TimingBuffer TimingBuffer;
    private readonly System.Timers.Timer _timer;

    public Metrics()
    {
        TimingBuffer = new(30);
        _timer = new(TimeSpan.FromSeconds(1));
        _timer.Start();
        _timer.Elapsed += TimerElapsed;
    }

    private void TimerElapsed(object? sender, System.Timers.ElapsedEventArgs e)
    {
        Toolkit.Window.Logger?.LogInfo($"Frame time Avg: {(TimingBuffer.Average() * 1000):###.###}ms");
    }
}

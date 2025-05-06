using OpenTK.Platform;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace VBeamRT;
internal sealed class Metrics
{
    public double DeltaTimeFull;
    public float DeltaTime;
    public readonly TimingBuffer TimingBuffer;
    private readonly System.Timers.Timer _timer;
    private readonly Game _game;

    public Metrics(Game game)
    {
        TimingBuffer = new(30);
        _game = game;
        _timer = new(TimeSpan.FromSeconds(1));
        _timer.Start();
        _timer.Elapsed += TimerElapsed;
    }

    private void TimerElapsed(object? sender, System.Timers.ElapsedEventArgs e)
    {
        string log = $"Frame time Avg: {(TimingBuffer.Average() * 1000):###.###}ms";
        Toolkit.Window.Logger?.LogInfo(log);
        Toolkit.Window.SetTitle(_game.MainWindowInfo.Handle, log);
    }
}

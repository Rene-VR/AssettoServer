using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using AssettoServer.Server.Ai.Splines;
using AssettoServer.Server.Configuration;
using Serilog;
using YamlDotNet.Serialization;

namespace AssettoServer.Server.Ai;

// Seacrest 2026-10-09: AI traffic rules - stop signs + give way.
// Reads <cfg folder>/traffic_rules.yml (made by D:\SEACREST\ai_splines\rules\make_rules.py).
// AI stops at a stop point, waits StopSeconds, then goes when the junction box is empty, no car on a
// priority (Watch) lane reaches the box within GapSeconds, and no other car has been waiting longer.
// After MaxWaitSeconds it goes anyway. No file = rules off (server behaves as before).

public class TrafficRulesFile
{
    public float StopSeconds { get; set; } = 2.5f;
    public float GapSeconds { get; set; } = 4.0f;
    public float MaxWaitSeconds { get; set; } = 25.0f;
    public float StopDecelFactor { get; set; } = 0.5f;   // fraction of AiDeceleration used to brake for a stop
    public float PlayerWatchMeters { get; set; } = 120f; // players closer than Radius + this are checked
    public bool Debug { get; set; } = false;
    public List<TrafficRuleJunction> Junctions { get; set; } = new();
}

public class TrafficRuleJunction
{
    public string Name { get; set; } = "";
    public List<float> Center { get; set; } = new();
    public float Radius { get; set; } = 15;
    public bool AllWayStop { get; set; }
    public float GapExtraSeconds { get; set; } = 0;   // merge stops: time the stopped car needs to reach the merge point
    public bool PlayersOnly { get; set; } = false;    // Seacrest 2026-10-26: stop, then wait only for players (not for AI traffic)
    public List<TrafficRuleStop> Stops { get; set; } = new();
    public List<TrafficRuleWatch> Watch { get; set; } = new();
}

public class TrafficRuleStop
{
    public string? Lane { get; set; }
    public List<float> Pos { get; set; } = new();
}

public class TrafficRuleWatch
{
    public string? Lane { get; set; }
    public List<float> From { get; set; } = new();
    public List<float> Entry { get; set; } = new();
    public List<float> To { get; set; } = new();
}

public class TrafficJunction
{
    public string Name = "";
    public Vector3 Center;
    public float Radius;
    public float GapExtra;
    public bool PlayersOnly;
    public readonly HashSet<int> StopIds = new();
    public readonly Dictionary<int, float> WatchDistance = new(); // point id -> metres to the box entry
    public readonly List<(AiState Ai, long Since)> Waiting = new();
}

public class TrafficRules
{
    public bool Enabled { get; }
    public float StopDecelFactor => _file.StopDecelFactor;

    private readonly TrafficRulesFile _file = new();
    private readonly Dictionary<int, TrafficJunction> _stops = new();
    private readonly List<TrafficJunction> _junctions = new();
    private readonly ConcurrentDictionary<AiState, byte> _aiStates = new();
    private readonly Lazy<EntryCarManager> _entryCarManager; // lazy: EntryCarManager creates the AI states that need this class
    private readonly object _lock = new();

    public TrafficRules(ACServerConfiguration configuration, AiSpline spline, Lazy<EntryCarManager> entryCarManager)
    {
        _entryCarManager = entryCarManager;
        var path = Path.Join(configuration.BaseFolder, "traffic_rules.yml");
        if (!File.Exists(path))
        {
            Log.Information("Traffic rules: {Path} not found, AI ignores stop signs", path);
            return;
        }

        try
        {
            var deserializer = new DeserializerBuilder().IgnoreUnmatchedProperties().Build();
            _file = deserializer.Deserialize<TrafficRulesFile>(File.ReadAllText(path)) ?? new TrafficRulesFile();
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Traffic rules: cannot read {Path}, rules off", path);
            return;
        }

        var points = spline.Points;
        int stopCount = 0, watchCount = 0, skipped = 0, n = 0;
        foreach (var j in _file.Junctions)
        {
            n++;
            var tj = new TrafficJunction { Name = j.Name, Center = V(j.Center), Radius = j.Radius, GapExtra = j.GapExtraSeconds, PlayersOnly = j.PlayersOnly };

            foreach (var s in j.Stops)
            {
                int id = Find(spline, s.Pos);
                if (id < 0) { skipped++; Log.Warning("Traffic rules: {Junction} stop {Lane} not on the spline, skipped", j.Name, s.Lane); continue; }
                tj.StopIds.Add(id);
                _stops[id] = tj;
                stopCount++;
            }

            foreach (var w in j.Watch)
            {
                int from = Find(spline, w.From), entry = Find(spline, w.Entry), to = Find(spline, w.To);
                if (from < 0 || entry < 0 || to < 0) { skipped++; Log.Warning("Traffic rules: {Junction} watch {Lane} not on the spline, skipped", j.Name, w.Lane); continue; }

                // walk from -> to along the lane, remember the distance to the box entry
                var ids = new List<int>();
                var cum = new List<float>();
                float dist = 0, entryDist = -1;
                int id = from;
                for (int step = 0; step < 2000 && id >= 0; step++)
                {
                    ids.Add(id);
                    cum.Add(dist);
                    if (id == entry) entryDist = dist;
                    if (id == to) break;
                    dist += points[id].Length;
                    id = points[id].NextId;
                }
                if (entryDist < 0 || ids[^1] != to) { skipped++; Log.Warning("Traffic rules: {Junction} watch {Lane} walk failed, skipped", j.Name, w.Lane); continue; }
                for (int k = 0; k < ids.Count; k++)
                    tj.WatchDistance[ids[k]] = MathF.Max(0, entryDist - cum[k]);
                watchCount++;
            }

            _junctions.Add(tj);
            Log.Debug("Traffic rules: junction {Count}/{Total} {Name} loaded", n, _file.Junctions.Count, j.Name);
        }

        Enabled = _stops.Count > 0;
        Log.Information("Traffic rules: {Junctions} junctions, {Stops} stop points, {Watch} watch lanes, {Skipped} skipped ({Path})",
            _junctions.Count, stopCount, watchCount, skipped, path);
    }

    private static Vector3 V(List<float> p) => p.Count >= 3 ? new Vector3(p[0], p[1], p[2]) : Vector3.Zero;

    private static int Find(AiSpline spline, List<float> p)
    {
        if (p.Count < 3) return -1;
        var (id, distSq) = spline.WorldToSpline(V(p));
        return distSq < 0.5f * 0.5f ? id : -1;
    }

    public void Register(AiState ai) => _aiStates.TryAdd(ai, 0);

    public void DebugLog(string message, params object[] args)
    {
        if (_file.Debug) Log.Information("Traffic rules: " + message, args);
    }

    public TrafficJunction? GetStop(int pointId) => _stops.TryGetValue(pointId, out var j) ? j : null;

    public bool IsWaitingCar(AiState ai)
    {
        lock (_lock)
        {
            foreach (var j in _junctions)
                foreach (var w in j.Waiting)
                    if (w.Ai == ai) return true;
        }
        return false;
    }

    public void Arrive(TrafficJunction j, AiState ai, long now)
    {
        lock (_lock)
        {
            foreach (var w in j.Waiting) if (w.Ai == ai) return;
            j.Waiting.Add((ai, now));
        }
        if (_file.Debug) Log.Information("Traffic rules: AI {Id} stopped at {Junction}", ai.EntryCar.SessionId, j.Name);
    }

    public void Leave(TrafficJunction j, AiState ai)
    {
        lock (_lock) j.Waiting.RemoveAll(w => w.Ai == ai);
    }

    public void LeaveAll(AiState ai)
    {
        lock (_lock)
            foreach (var j in _junctions) j.Waiting.RemoveAll(w => w.Ai == ai);
    }

    public bool CanGo(TrafficJunction j, AiState ai, long now, long since)
    {
        float waited = (now - since) / 1000f;
        if (waited < _file.StopSeconds) return false;
        if (waited > _file.MaxWaitSeconds)
        {
            if (_file.Debug) Log.Information("Traffic rules: AI {Id} max wait at {Junction}, going", ai.EntryCar.SessionId, j.Name);
            return true;
        }

        float r2 = j.Radius * j.Radius;
        float gap = _file.GapSeconds + j.GapExtra;

        // first come, first go (cars that stopped earlier at this junction go first)
        lock (_lock)
        {
            j.Waiting.RemoveAll(w => !w.Ai.Initialized);
            foreach (var w in j.Waiting)
                if (w.Ai != ai && w.Since < since) return false;
        }

        // other AI: in the box, or on a priority lane arriving within GapSeconds (skipped for PlayersOnly junctions)
        foreach (var other in _aiStates.Keys)
        {
            if (j.PlayersOnly) break;
            if (other == ai || !other.Initialized) continue;
            bool waitingHere;
            lock (_lock) waitingHere = j.Waiting.Exists(w => w.Ai == other);
            if (waitingHere) continue;

            if (Vector3.DistanceSquared(other.Status.Position, j.Center) < r2 && other.CurrentSpeed > 0.3f) return false;

            if (j.WatchDistance.TryGetValue(other.CurrentSplinePointId, out var d) && other.CurrentSpeed > 0.5f
                && d / other.CurrentSpeed < gap) return false;
        }

        // players: in the box, or driving towards it and arriving within GapSeconds
        foreach (var car in _entryCarManager.Value.EntryCars)
        {
            if (car.Client?.HasSentFirstUpdate != true) continue;
            var pos = car.Status.Position;
            var toC = j.Center - pos;
            if (j.PlayersOnly && MathF.Abs(toC.Y) > 4f) continue; // other level (bridge / road below)
            float dist = toC.Length();
            if (dist < j.Radius) return false;
            if (dist > j.Radius + _file.PlayerWatchMeters) continue;
            float closing = Vector3.Dot(car.Status.Velocity, toC / dist);
            if (closing > 1.0f && (dist - j.Radius) / closing < gap) return false;
        }

        if (_file.Debug) Log.Information("Traffic rules: AI {Id} goes at {Junction} after {Waited:0.0} s", ai.EntryCar.SessionId, j.Name, waited);
        return true;
    }
}

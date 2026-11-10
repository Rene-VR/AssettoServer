using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using AssettoServer.Server.Ai.Splines;
using AssettoServer.Server.Configuration;
using Serilog;
using YamlDotNet.Serialization;

namespace AssettoServer.Server.Ai;

// Seacrest 2026-10-25: AI drift zones (visual only).
// Reads <cfg folder>/drift_zones.yml. No file = drift off (server behaves as before).
// Per zone (centre = middle of the chevrons, game x / y / z) the server finds at startup every lane near the
// centre, takes the part of that lane where the spline radius <= MaxCornerRadius as the corner, and lets ONLY
// lanes on the OUTSIDE of the corner drift (no adjacent lane on the outside, incl. oncoming lanes).
// A drifting car gets extra yaw (nose into the corner), counter-steer on the front wheels, a small outward
// offset, optional extra corner speed and faster spinning rear wheels.

public class DriftZonesFile
{
    public float AngleDeg { get; set; } = 25f;           // extra body yaw at full drift
    public float CounterSteerRatio { get; set; } = 0.6f; // front wheel angle = -(drift angle x this), limited by the wheel lock
    public float OutwardMeters { get; set; } = 0.5f;     // car slides this far to the outside at full drift
    public float SpeedFactor { get; set; } = 1.0f;       // corner speed x this for drifting cars (1 = unchanged)
    public float RearWheelSpin { get; set; } = 2.0f;     // rear tyre angular speed x this at full drift (smoke test; 1 = off)
    public float BlendMeters { get; set; } = 15f;        // ramp in before / out after the corner part
    public float BlendRate { get; set; } = 2.5f;         // time smoothing (per second)
    public float Chance { get; set; } = 1.0f;            // share of AI cars that drift (decided per spawn)
    public float SearchRadius { get; set; } = 120f;      // lanes within this distance (horizontal) of the centre
    public float HeightTolerance { get; set; } = 20f;    // and within this height difference
    public float MaxCornerRadius { get; set; } = 150f;   // spline points with radius <= this are "corner"
    public float MaxLaneGapMeters { get; set; } = 12f;   // another lane closer than this on the outside = not the outside lane
    public float MinCornerMeters { get; set; } = 10f;    // corner parts shorter than this are ignored (junction stubs)
    public float MinCornerRadius { get; set; } = 25f;    // lane parts with a tighter point than this never drift (junction turns), 2026-10-26
    public float MinSpeedKmh { get; set; } = 60f;        // no drift below this speed (blends out), 2026-10-26
    public bool Debug { get; set; } = true;              // log every car that starts drifting
    public List<DriftZoneEntry> Zones { get; set; } = new();
}

public class DriftZoneEntry
{
    public string Name { get; set; } = "";
    public List<float> Center { get; set; } = new();     // [x, y, z] game coords (y = height)
    public float? SearchRadius { get; set; }
    public float? MaxCornerRadius { get; set; }
    public bool Enabled { get; set; } = true;
}

public class DriftZones
{
    public bool Enabled { get; }
    public DriftZonesFile Settings { get; } = new();

    private readonly float[] _weight = Array.Empty<float>();   // 0..1 per spline point
    private readonly sbyte[] _side = Array.Empty<sbyte>();     // +1 corner turns toward the lane's LeftId side, -1 toward RightId side
    private readonly short[] _zone = Array.Empty<short>();     // zone index per point (-1 = none)
    private readonly List<string> _zoneNames = new();

    public DriftZones(ACServerConfiguration configuration, AiSpline spline)
    {
        var path = Path.Join(configuration.BaseFolder, "drift_zones.yml");
        if (!File.Exists(path))
        {
            Log.Debug("Drift zones: {Path} not found, AI drift off", path);
            return;
        }

        try
        {
            var deserializer = new DeserializerBuilder().IgnoreUnmatchedProperties().Build();
            Settings = deserializer.Deserialize<DriftZonesFile>(File.ReadAllText(path)) ?? new DriftZonesFile();
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Drift zones: cannot read {Path}, drift off", path);
            return;
        }

        var points = spline.Points;
        int n = points.Length;
        _weight = new float[n];
        _side = new sbyte[n];
        _zone = new short[n];
        Array.Fill(_zone, (short)-1);

        int driftLanes = 0, zoneNo = 0;
        foreach (var z in Settings.Zones)
        {
            zoneNo++;
            Log.Debug("Drift zones: zone {No}/{Count} {Name}", zoneNo, Settings.Zones.Count, z.Name);
            if (!z.Enabled) { Log.Debug("Drift zones: {Name} disabled", z.Name); continue; }
            if (z.Center.Count < 3) { Log.Debug("Drift zones: {Name} needs Center [x, y, z], skipped", z.Name); continue; }

            short zoneIndex = (short)_zoneNames.Count;
            _zoneNames.Add(z.Name);
            var c = new Vector3(z.Center[0], z.Center[1], z.Center[2]);
            float r = z.SearchRadius ?? Settings.SearchRadius;
            float maxCornerRadius = z.MaxCornerRadius ?? Settings.MaxCornerRadius;

            // 1. all points near the centre
            var cand = new HashSet<int>();
            for (int i = 0; i < n; i++)
            {
                var p = points[i].Position;
                float dx = p.X - c.X, dz = p.Z - c.Z;
                if (dx * dx + dz * dz <= r * r && MathF.Abs(p.Y - c.Y) <= Settings.HeightTolerance)
                    cand.Add(i);
            }
            if (cand.Count == 0) { Log.Debug("Drift zones: {Name} no spline points within {R} m, check the centre", z.Name, r); continue; }

            // 2. split into lane chains (prev/next links inside the candidate set)
            var visited = new HashSet<int>();
            var chains = new List<List<int>>();
            int lanes = 0;
            foreach (int start0 in cand)
            {
                if (visited.Contains(start0)) continue;
                int start = start0;
                for (int s = 0; s < 5000; s++)
                {
                    int prev = points[start].PreviousId;
                    if (prev < 0 || !cand.Contains(prev) || prev == start0) break;
                    start = prev;
                }
                var chain = new List<int>();
                int id = start;
                for (int s = 0; s < 5000 && id >= 0 && cand.Contains(id) && !visited.Contains(id); s++)
                {
                    chain.Add(id);
                    visited.Add(id);
                    id = points[id].NextId;
                }
                if (chain.Count >= 3) chains.Add(chain);
            }

            foreach (var chain in chains)
            {
                lanes++;

                // 3. corner part = first..last point with radius <= maxCornerRadius
                int first = -1, last = -1;
                for (int k = 0; k < chain.Count; k++)
                {
                    float rad = points[chain[k]].Radius;
                    if (rad > 0.01f && rad <= maxCornerRadius) { if (first < 0) first = k; last = k; }
                }
                if (first < 0)
                {
                    Log.Debug("Drift zones: {Name} lane {Lane} (start point {Id}, {Pts} pts): no corner (radius > {Max} m)", z.Name, lanes, chain[0], chain.Count, maxCornerRadius);
                    continue;
                }

                // 4. turn direction (same 'left' vector as AdjacentLaneDetector uses for LeftId) and outside-lane check
                float turnSum = 0;
                for (int k = first; k <= last; k++)
                    turnSum += TurnSide(points, chain[k]);
                sbyte side = (sbyte)(turnSum >= 0 ? 1 : -1);

                // outside lane = on most corner points there is NO lane on the outside:
                // no adjacent-lane link (RightId for a left-side turn, LeftId for a right-side turn) AND no other
                // spline point (any lane, any direction) within MaxLaneGapMeters on the outside side
                var chainSet = new HashSet<int>(chain);
                int outsideFree = 0, cnt = 0;
                for (int k = first; k <= last; k++)
                {
                    ref readonly var p = ref points[chain[k]];
                    cnt++;
                    if ((side > 0 ? p.RightId : p.LeftId) >= 0) continue;
                    int nxt = p.NextId >= 0 ? p.NextId : p.PreviousId;
                    if (nxt < 0) continue;
                    var dir = new Vector2(points[nxt].Position.X - p.Position.X, points[nxt].Position.Z - p.Position.Z);
                    if (p.NextId < 0) dir = -dir;
                    if (dir.LengthSquared() < 1e-6f) continue;
                    dir = Vector2.Normalize(dir);
                    var left = new Vector2(dir.Y, -dir.X);
                    bool blocked = false;
                    foreach (int q in cand)
                    {
                        if (chainSet.Contains(q)) continue;
                        var dq = new Vector2(points[q].Position.X - p.Position.X, points[q].Position.Z - p.Position.Z);
                        if (MathF.Abs(Vector2.Dot(dq, dir)) > 4f) continue;
                        if (MathF.Abs(points[q].Position.Y - p.Position.Y) > 5f) continue;
                        float lat = Vector2.Dot(dq, left) * side; // > 0 = inside of the corner, < 0 = outside
                        if (lat < -1f && lat > -Settings.MaxLaneGapMeters) { blocked = true; break; }
                    }
                    if (!blocked) outsideFree++;
                }
                bool outside = outsideFree * 2 > cnt;

                // cumulative distance along the chain
                var dist = new float[chain.Count];
                for (int k = 1; k < chain.Count; k++)
                    dist[k] = dist[k - 1] + Vector3.Distance(points[chain[k - 1]].Position, points[chain[k]].Position);
                float coreLen = dist[last] - dist[first];
                float minR = float.MaxValue;
                for (int k = first; k <= last; k++)
                {
                    float rk = points[chain[k]].Radius;
                    if (rk > 0.01f) minR = MathF.Min(minR, rk);
                }
                // Seacrest 2026-10-26: no drift on tight lane parts (junction turns, R < MinCornerRadius)
                bool tooTight = minR < Settings.MinCornerRadius;

                Log.Debug("Drift zones: {Name} lane {Lane} (start point {Id}, {Pts} pts): corner {Len:0} m, min radius {MinR:0} m, turns {Turn}, outside free {Free}/{Cnt} -> {Result}",
                    z.Name, lanes, chain[0], chain.Count, coreLen, minR, side > 0 ? "toward LEFT lane side" : "toward RIGHT lane side",
                    outsideFree, cnt, coreLen < Settings.MinCornerMeters ? "too short, no drift" : tooTight ? "too tight (junction turn), no drift" : outside ? "OUTSIDE lane, DRIFTS" : "inside lane, no drift");
                if (!outside || coreLen < Settings.MinCornerMeters || tooTight) continue;
                driftLanes++;

                // 5. weights: 1 in the corner part, linear ramp over BlendMeters before and after
                float blend = MathF.Max(0.1f, Settings.BlendMeters);
                for (int k = 0; k < chain.Count; k++)
                {
                    float d = dist[k];
                    float w;
                    if (d < dist[first]) w = 1f - (dist[first] - d) / blend;
                    else if (d > dist[last]) w = 1f - (d - dist[last]) / blend;
                    else w = 1f;
                    if (w <= 0) continue;
                    int pid = chain[k];
                    if (w > _weight[pid])
                    {
                        _weight[pid] = w;
                        _side[pid] = side;
                        _zone[pid] = zoneIndex;
                    }
                }
            }
            Log.Debug("Drift zones: {Name} done, {Lanes} lane(s) near the centre", z.Name, lanes);
        }

        Enabled = driftLanes > 0;
        Log.Debug("Drift zones: {Zones} zone(s), {Lanes} drifting lane part(s), angle {Angle} deg, chance {Chance}",
            _zoneNames.Count, driftLanes, Settings.AngleDeg, Settings.Chance);
    }

    // +1 = the spline bends toward the point's LeftId side, -1 = toward RightId side, 0 = unknown
    private static float TurnSide(ReadOnlySpan<SplinePoint> points, int id)
    {
        ref readonly var p = ref points[id];
        if (p.PreviousId < 0 || p.NextId < 0) return 0;
        var a = points[p.PreviousId].Position;
        var b = p.Position;
        var c = points[p.NextId].Position;
        var d1 = new Vector2(b.X - a.X, b.Z - a.Z);
        var d2 = new Vector2(c.X - b.X, c.Z - b.Z);
        if (d1.LengthSquared() < 1e-6f || d2.LengthSquared() < 1e-6f) return 0;
        d1 = Vector2.Normalize(d1);
        d2 = Vector2.Normalize(d2);
        var left = new Vector2(d2.Y, -d2.X); // AdjacentLaneDetector: LeftId is found at (dz, -dx)
        return Vector2.Dot(d2 - d1, left);
    }

    public float Weight(int pointId) => Enabled && pointId >= 0 && pointId < _weight.Length ? _weight[pointId] : 0f;
    public int Side(int pointId) => Enabled && pointId >= 0 && pointId < _side.Length ? _side[pointId] : 0;
    public string ZoneName(int pointId) => Enabled && pointId >= 0 && pointId < _zone.Length && _zone[pointId] >= 0 ? _zoneNames[_zone[pointId]] : "";
}

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using AssettoServer.Server.Ai.Splines;
using AssettoServer.Server.Configuration;
using AssettoServer.Server.Weather;
using AssettoServer.Shared.Model;
using AssettoServer.Shared.Network.Packets.Outgoing;
using AssettoServer.Utils;
using JPBotelho;
using Serilog;

namespace AssettoServer.Server.Ai;

public class AiState
{
    public CarStatus Status { get; } = new();
    public bool Initialized { get; private set; }

    public int CurrentSplinePointId
    {
        get => _currentSplinePointId;
        private set
        {
            _spline.SlowestAiStates.Enter(value, this);
            _spline.SlowestAiStates.Leave(_currentSplinePointId, this);
            _currentSplinePointId = value;
        }
    }

    private int _currentSplinePointId;
    
    public long SpawnProtectionEnds { get; set; }
    public float SafetyDistanceSquared { get; set; } = 20 * 20;
    public float Acceleration { get; set; }
    public float CurrentSpeed { get; private set; }
    public float TargetSpeed { get; private set; }
    public float InitialMaxSpeed { get; private set; }
    public float MaxSpeed { get; private set; }
    public Color Color { get; private set; }
    public byte SpawnCounter { get; private set; }
    public float ClosestAiObstacleDistance { get; private set; }
    public EntryCar EntryCar { get; }

    private const float WalkingSpeed = 10 / 3.6f;

    private Vector3 _startTangent;
    private Vector3 _endTangent;

    private float _currentVecLength;
    private float _currentVecProgress;
    private long _lastTick;
    private bool _stoppedForObstacle;
    private long _stoppedForObstacleSince;
    private long _ignoreObstaclesUntil;
    private long _stoppedForCollisionUntil;
    private long _obstacleHonkStart;
    private long _obstacleHonkEnd;
    private CarStatusFlags _indicator = 0;
    private int _nextJunctionId;
    private bool _junctionPassed;
    private float _endIndicatorDistance;
    private float _minObstacleDistance;

    // front wheel steering for AI traffic
    private float _lastYaw;
    private bool _hasLastYaw;
    private float _smoothedWheelAngleDeg;
    private const float WheelbaseMeters = 2.6f;
    private const float MaxWheelAngleDeg = 35f;
    // WheelAngleScale, RollDegPerG, PitchDegPerG: now read from extra_cfg.yml (AiParams), 2026-10-10
    private float WheelAngleScale => _configuration.Extra.AiParams.WheelAngleScale;
    private float RollDegPerG => _configuration.Extra.AiParams.RollDegPerG;
    private float PitchDegPerG => _configuration.Extra.AiParams.PitchDegPerG;
    private const float WheelAngleSign = 1.0f;  // set to -1 if wheels turn the wrong way

    // body roll for AI traffic
    private float _smoothedRollRad;
    private const float RollSign = 1.0f;       // set to -1 if cars lean into the corner
    private const float MaxRollDeg = 6.0f;

    // body pitch for AI traffic (nose dives under braking, squats under acceleration)
    private float _lastSpeed;
    private float _smoothedLongAccel;
    private float _smoothedPitchRad;
    private const float PitchSign = 1.0f;      // set to -1 if the nose goes UP under braking
    private const float MaxPitchDeg = 3.0f;

    private readonly ACServerConfiguration _configuration;
    private readonly SessionManager _sessionManager;
    private readonly EntryCarManager _entryCarManager;
    private readonly WeatherManager _weatherManager;
    private readonly AiSpline _spline;
    private readonly JunctionEvaluator _junctionEvaluator;

    // Seacrest 2026-10-09: traffic rules (stop signs + give way), see trafficrules.cs
    private readonly TrafficRules _rules;
    private TrafficJunction? _stopJunction;
    private int _stopPointId = -1;
    private float _stopDistance = float.MaxValue;
    private int _clearedStopId = -1;
    private bool _waitingAtStop;
    private long _waitingSince;
    public bool InStopQueue { get; private set; }

    // Seacrest 2026-10-25: drift zones (visual drift in chevron corners), see driftzones.cs
    private readonly DriftZones _drift;
    private bool _driftCar;          // this spawn may drift (Chance)
    private float _driftFactor;      // 0..1, smoothed
    private int _driftSide;          // +1 / -1, last zone side
    private bool _driftLogged;

    private static readonly List<Color> CarColors =
    [
        Color.FromArgb(13, 17, 22),
        Color.FromArgb(19, 24, 31),
        Color.FromArgb(28, 29, 33),
        Color.FromArgb(12, 13, 24),
        Color.FromArgb(11, 20, 33),
        Color.FromArgb(151, 154, 151),
        Color.FromArgb(153, 157, 160),
        Color.FromArgb(194, 196, 198),
        Color.FromArgb(234, 234, 234),
        Color.FromArgb(255, 255, 255),
        Color.FromArgb(182, 17, 27),
        Color.FromArgb(218, 25, 24),
        Color.FromArgb(73, 17, 29),
        Color.FromArgb(35, 49, 85),
        Color.FromArgb(28, 53, 81),
        Color.FromArgb(37, 58, 167),
        Color.FromArgb(21, 92, 45),
        Color.FromArgb(18, 46, 43)
    ];

    public AiState(EntryCar entryCar, SessionManager sessionManager, WeatherManager weatherManager, ACServerConfiguration configuration, EntryCarManager entryCarManager, AiSpline spline, TrafficRules rules, DriftZones drift)
    {
        _drift = drift;
        EntryCar = entryCar;
        _sessionManager = sessionManager;
        _weatherManager = weatherManager;
        _configuration = configuration;
        _entryCarManager = entryCarManager;
        _spline = spline;
        _junctionEvaluator = new JunctionEvaluator(spline);
        _rules = rules;
        _rules.Register(this);

        _lastTick = _sessionManager.ServerTimeMilliseconds;
    }

    public void Despawn()
    {
        Initialized = false;
        _spline.SlowestAiStates.Leave(CurrentSplinePointId, this);
        ResetStopState();
    }

    private void SetRandomSpeed()
    {
        float variation = _configuration.Extra.AiParams.MaxSpeedMs * _configuration.Extra.AiParams.MaxSpeedVariationPercent;

        float fastLaneOffset = 0;
        if (_spline.Points[CurrentSplinePointId].LeftId >= 0)
        {
            fastLaneOffset = _configuration.Extra.AiParams.RightLaneOffsetMs;
        }
        InitialMaxSpeed = _configuration.Extra.AiParams.MaxSpeedMs + fastLaneOffset - (variation / 2) + (float)Random.Shared.NextDouble() * variation;
        CurrentSpeed = InitialMaxSpeed;
        TargetSpeed = InitialMaxSpeed;
        MaxSpeed = InitialMaxSpeed;
    }

    private void SetRandomColor()
    {
        Color = CarColors[Random.Shared.Next(CarColors.Count)];
    }

    public void Teleport(int pointId)
    {
        _junctionEvaluator.Clear();
        CurrentSplinePointId = pointId;
        if (!_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextPointId))
            throw new InvalidOperationException($"Cannot get next spline point for {CurrentSplinePointId}");
        _currentVecLength = (_spline.Points[nextPointId].Position - _spline.Points[CurrentSplinePointId].Position).Length();
        _currentVecProgress = 0;
            
        CalculateTangents();
        
        SetRandomSpeed();
        SetRandomColor();

        var minDist = _configuration.Extra.AiParams.MinAiSafetyDistanceSquared;
        var maxDist = _configuration.Extra.AiParams.MaxAiSafetyDistanceSquared;
        if (_configuration.Extra.AiParams.LaneCountSpecificOverrides.TryGetValue(_spline.GetLanes(CurrentSplinePointId).Length, out var overrides))
        {
            minDist = overrides.MinAiSafetyDistanceSquared;
            maxDist = overrides.MaxAiSafetyDistanceSquared;
        }
        
        if (EntryCar.MinAiSafetyDistanceMetersSquared.HasValue)
            minDist = EntryCar.MinAiSafetyDistanceMetersSquared.Value;
        if (EntryCar.MaxAiSafetyDistanceMetersSquared.HasValue)
            maxDist = EntryCar.MaxAiSafetyDistanceMetersSquared.Value;

        SpawnProtectionEnds = _sessionManager.ServerTimeMilliseconds + Random.Shared.Next(EntryCar.AiMinSpawnProtectionTimeMilliseconds, EntryCar.AiMaxSpawnProtectionTimeMilliseconds);
        SafetyDistanceSquared = Random.Shared.Next((int)Math.Round(minDist * (1.0f / _configuration.Extra.AiParams.TrafficDensity)),
            (int)Math.Round(maxDist * (1.0f / _configuration.Extra.AiParams.TrafficDensity)));
        _stoppedForCollisionUntil = 0;
        _ignoreObstaclesUntil = 0;
        _obstacleHonkEnd = 0;
        _obstacleHonkStart = 0;
        _indicator = 0;
        _nextJunctionId = -1;
        _junctionPassed = false;
        _endIndicatorDistance = 0;
        _lastTick = _sessionManager.ServerTimeMilliseconds;
        _minObstacleDistance = Random.Shared.Next(8, 13);
        _hasLastYaw = false;
        _smoothedWheelAngleDeg = 0;
        _smoothedRollRad = 0;
        _smoothedLongAccel = 0;
        _smoothedPitchRad = 0;
        _lastSpeed = CurrentSpeed;
        _driftCar = _drift.Enabled && Random.Shared.NextDouble() < _drift.Settings.Chance;
        _driftFactor = 0;
        _driftSide = 0;
        _driftLogged = false;
        ResetStopState();
        if (_rules.Enabled)
        {
            // spawned just past a stop line: do not stop for it
            int backId = CurrentSplinePointId;
            for (int b = 0; b < 6 && backId >= 0; b++)
            {
                if (_rules.GetStop(backId) != null) { _clearedStopId = backId; break; }
                if (!_junctionEvaluator.TryPrevious(backId, out var prevId)) break;
                backId = prevId;
            }
        }
        SpawnCounter++;
        Initialized = true;
        Update();
    }

    private void ResetStopState()
    {
        if (_waitingAtStop || InStopQueue) _rules.LeaveAll(this);
        _stopJunction = null;
        _stopPointId = -1;
        _stopDistance = float.MaxValue;
        _clearedStopId = -1;
        _waitingAtStop = false;
        InStopQueue = false;
    }

    // Seacrest 2026-10-26: front wheel angle from the spline geometry
    private const float WheelSmoothMeters = 3.0f; // wheel follows the target over about this distance driven

    private static float VertexCurvature(Vector3 a, Vector3 b, Vector3 c)
    {
        float h1 = MathF.Atan2(b.Z - a.Z, b.X - a.X);
        float h2 = MathF.Atan2(c.Z - b.Z, c.X - b.X);
        float d = h2 - h1;
        if (d > MathF.PI) d -= 2 * MathF.PI;
        else if (d < -MathF.PI) d += 2 * MathF.PI;
        float len = 0.5f * (new Vector2(b.X - a.X, b.Z - a.Z).Length() + new Vector2(c.X - b.X, c.Z - b.Z).Length());
        return len > 0.05f ? d / len : 0f;
    }

    // signed curvature around the car (same sign as the yaw rate), blended along the segment -> wheel angle in degrees
    private float GeometricWheelAngleDeg(int nextPointId, float t)
    {
        var pts = _spline.Points;
        int p0 = CurrentSplinePointId;
        Vector3 P0 = pts[p0].Position;
        Vector3 P1 = pts[nextPointId].Position;
        bool has2 = _junctionEvaluator.TryNext(p0, out var p2, 2);
        bool has3 = has2 && _junctionEvaluator.TryNext(p0, out var p3, 3) && p3 >= 0;
        bool hasM = _junctionEvaluator.TryPrevious(p0, out var pm);

        float k1 = has2 ? VertexCurvature(P0, P1, pts[p2].Position) : 0f;
        float k0 = hasM ? VertexCurvature(pts[pm].Position, P0, P1) : k1;
        float k2 = k1;
        if (has3 && _junctionEvaluator.TryNext(p0, out var p3b, 3))
            k2 = VertexCurvature(P1, pts[p2].Position, pts[p3b].Position);

        t = Math.Clamp(t, 0f, 1f);
        float kappa = (1f - t) * 0.5f * (k0 + k1) + t * 0.5f * (k1 + k2);
        float deg = MathF.Atan(WheelbaseMeters * kappa) * 180f / MathF.PI;
        return Math.Clamp(deg, -MaxWheelAngleDeg, MaxWheelAngleDeg);
    }

    private void CalculateTangents()
    {
        if (!_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextPointId))
            throw new InvalidOperationException("Cannot get next spline point");

        var points = _spline.Points;
        
        if (_junctionEvaluator.TryPrevious(CurrentSplinePointId, out var previousPointId))
        {
            _startTangent = (points[nextPointId].Position - points[previousPointId].Position) * 0.5f;
        }
        else
        {
            _startTangent = (points[nextPointId].Position - points[CurrentSplinePointId].Position) * 0.5f;
        }

        if (_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextNextPointId, 2))
        {
            _endTangent = (points[nextNextPointId].Position - points[CurrentSplinePointId].Position) * 0.5f;
        }
        else
        {
            _endTangent = (points[nextPointId].Position - points[CurrentSplinePointId].Position) * 0.5f;
        }
    }

    private bool Move(float progress)
    {
        var points = _spline.Points;
        var junctions = _spline.Junctions;
        
        bool recalculateTangents = false;
        while (progress > _currentVecLength)
        {
            progress -= _currentVecLength;
                
            if (!_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextPointId)
                || !_junctionEvaluator.TryNext(nextPointId, out var nextNextPointId))
            {
                return false;
            }

            CurrentSplinePointId = nextPointId;
            _currentVecLength = (points[nextNextPointId].Position - points[CurrentSplinePointId].Position).Length();
            recalculateTangents = true;

            if (_junctionPassed)
            {
                _endIndicatorDistance -= _currentVecLength;

                if (_endIndicatorDistance < 0)
                {
                    _indicator = 0;
                    _junctionPassed = false;
                    _endIndicatorDistance = 0;
                }
            }
                
            if (_nextJunctionId >= 0 && points[CurrentSplinePointId].JunctionEndId == _nextJunctionId)
            {
                _junctionPassed = true;
                _endIndicatorDistance = junctions[_nextJunctionId].IndicateDistancePost;
                _nextJunctionId = -1;
            }
        }

        if (recalculateTangents)
        {
            CalculateTangents();
        }

        _currentVecProgress = progress;

        return true;
    }

    public bool CanSpawn(int spawnPointId, AiState? previousAi, AiState? nextAi)
    {
        var ops = _spline.Operations;
        ref readonly var spawnPoint = ref ops.Points[spawnPointId];

        if (!IsAllowedLaneCount(spawnPointId))
            return false;
        if (!IsAllowedLane(in spawnPoint))
            return false;
        if (!IsKeepingSafetyDistances(in spawnPoint, previousAi, nextAi))
            return false;

        return EntryCar.CanSpawnAiState(spawnPoint.Position, this);
    }

    private bool IsKeepingSafetyDistances(in SplinePoint spawnPoint, AiState? previousAi, AiState? nextAi)
    {
        if (previousAi != null)
        {
            var distance = MathF.Max(0, Vector3.Distance(spawnPoint.Position, previousAi.Status.Position)
                           - previousAi.EntryCar.VehicleLengthPreMeters
                           - EntryCar.VehicleLengthPostMeters);

            var distanceSquared = distance * distance;
            if (distanceSquared < previousAi.SafetyDistanceSquared || distanceSquared < SafetyDistanceSquared)
                return false;
        }
        
        if (nextAi != null)
        {
            var distance = MathF.Max(0, Vector3.Distance(spawnPoint.Position, nextAi.Status.Position)
                                        - nextAi.EntryCar.VehicleLengthPostMeters
                                        - EntryCar.VehicleLengthPreMeters);

            var distanceSquared = distance * distance;
            if (distanceSquared < nextAi.SafetyDistanceSquared || distanceSquared < SafetyDistanceSquared)
                return false;
        }

        return true;
    }

    private bool IsAllowedLaneCount(int spawnPointId)
    {
        var laneCount = _spline.GetLanes(spawnPointId).Length;
        if (EntryCar.MinLaneCount.HasValue && laneCount < EntryCar.MinLaneCount.Value)
            return false;
        if (EntryCar.MaxLaneCount.HasValue && laneCount > EntryCar.MaxLaneCount.Value)
            return false;
        
        return true;
    }

    private bool IsAllowedLane(in SplinePoint spawnPoint)
    {
        var isAllowedLane = true;
        if (EntryCar.AiAllowedLanes != null)
        {
            isAllowedLane = (EntryCar.AiAllowedLanes.Contains(LaneSpawnBehavior.Middle) && spawnPoint.LeftId >= 0 && spawnPoint.RightId >= 0)
                            || (EntryCar.AiAllowedLanes.Contains(LaneSpawnBehavior.Left) && spawnPoint.LeftId < 0)
                            || (EntryCar.AiAllowedLanes.Contains(LaneSpawnBehavior.Right) && spawnPoint.RightId < 0);
        }

        return isAllowedLane;
    }

    // Seacrest 2026-10-10: max corner speed^2 = lateral accel * radius, lateral accel goes from
    // CornerTightLatAccel (tight corners) to CornerWideLatAccel (wide corners), interpolated by radius
    private float MaxCorneringSpeedSquared(float radius)
    {
        var p = _configuration.Extra.AiParams;
        float span = p.CornerWideRadiusMeters - p.CornerTightRadiusMeters;
        float t = span > 0.01f ? Math.Clamp((radius - p.CornerTightRadiusMeters) / span, 0f, 1f) : 1f;
        float latAccel = p.CornerTightLatAccel + (p.CornerWideLatAccel - p.CornerTightLatAccel) * t;
        return MathF.Max(0.1f, latAccel) * MathF.Max(1f, radius);
    }

    // Seacrest 2026-10-25: drifting cars may take the drift corner faster (SpeedFactor in drift_zones.yml)
    private float DriftSpeedSquaredFactor(int pointId)
    {
        if (!_driftCar) return 1f;
        float w = _drift.Weight(pointId);
        if (w <= 0) return 1f;
        float f = 1f + (_drift.Settings.SpeedFactor - 1f) * w;
        return f * f;
    }

    // distance needed to slow from v (speed^2) to vc (speed^2) at the given deceleration, times CorneringBrakeDistanceFactor
    private float CornerBrakingDistance(float currentSpeedSquared, float targetSpeedSquared, float deceleration)
    {
        return MathF.Max(0f, currentSpeedSquared - targetSpeedSquared) / (2 * MathF.Max(0.1f, deceleration))
               * EntryCar.AiCorneringBrakeDistanceFactor;
    }

    private (AiState? ClosestAiState, float ClosestAiStateDistance, float MaxSpeed) SplineLookahead()
    {
        var points = _spline.Points;
        var junctions = _spline.Junctions;
        
        // Seacrest 2026-10-10: corner braking uses (v^2 - vc^2) / 2a; look far enough ahead for that
        float cornerDecel = EntryCar.AiDeceleration * EntryCar.AiCorneringBrakeForceFactor;
        float maxBrakingDistance = MathF.Max(PhysicsUtils.CalculateBrakingDistance(CurrentSpeed, EntryCar.AiDeceleration) * 2,
                                             CornerBrakingDistance(CurrentSpeed * CurrentSpeed, 0, cornerDecel)) + 20;
        AiState? closestAiState = null;
        float closestAiStateDistance = float.MaxValue;
        bool junctionFound = false;
        float distanceTravelled = 0;
        var pointId = CurrentSplinePointId;
        ref readonly var point = ref points[pointId];
        float maxSpeed = float.MaxValue;
        float currentSpeedSquared = CurrentSpeed * CurrentSpeed;

        // the point we are on now counts too (we are already in that part of the corner)
        {
            float vc2 = MaxCorneringSpeedSquared(point.Radius) * DriftSpeedSquaredFactor(pointId);
            if (vc2 < currentSpeedSquared)
                maxSpeed = MathF.Sqrt(vc2);
        }

        // traffic rules: first uncleared stop point ahead (or the one we are standing on / just passed)
        int stopId = -1;
        TrafficJunction? stopJct = null;
        float stopDist = float.MaxValue;
        if (_rules.Enabled)
        {
            // the point we are on, or up to ~10 m behind (rolled past the line): still stop there
            int backId = pointId;
            float back = -_currentVecProgress;
            for (int b = 0; b < 6 && backId >= 0; b++)
            {
                if (backId != _clearedStopId && (stopJct = _rules.GetStop(backId)) != null)
                {
                    stopId = backId;
                    stopDist = back;
                    break;
                }
                if (!_junctionEvaluator.TryPrevious(backId, out var prevId)) break;
                back -= points[prevId].Length;
                backId = prevId;
                if (back < -10f) break;
            }
            if (stopId < 0) stopJct = null;
        }

        while (distanceTravelled < maxBrakingDistance)
        {
            distanceTravelled += point.Length;
            pointId = _junctionEvaluator.Next(pointId);
            if (pointId < 0)
                break;

            point = ref points[pointId];

            if (!junctionFound && point.JunctionStartId >= 0 && distanceTravelled < junctions[point.JunctionStartId].IndicateDistancePre)
            {
                ref readonly var jct = ref junctions[point.JunctionStartId];
                
                var indicator = _junctionEvaluator.WillTakeJunction(point.JunctionStartId) ? jct.IndicateWhenTaken : jct.IndicateWhenNotTaken;
                if (indicator != 0)
                {
                    _indicator = indicator;
                    _nextJunctionId = point.JunctionStartId;
                    junctionFound = true;
                }
            }

            if (closestAiState == null)
            {
                var slowest = _spline.SlowestAiStates[pointId];

                if (slowest != null)
                {
                    closestAiState = slowest;
                    closestAiStateDistance = MathF.Max(0, Vector3.Distance(Status.Position, closestAiState.Status.Position)
                                                          - EntryCar.VehicleLengthPreMeters
                                                          - closestAiState.EntryCar.VehicleLengthPostMeters);
                }
            }

            // Seacrest 2026-10-10: radius dependent grip + correct braking distance, measured from the car itself
            float maxCorneringSpeedSquared = MaxCorneringSpeedSquared(point.Radius) * DriftSpeedSquaredFactor(pointId);
            if (maxCorneringSpeedSquared < currentSpeedSquared)
            {
                float maxCorneringSpeed = MathF.Sqrt(maxCorneringSpeedSquared);
                float brakingDistance = CornerBrakingDistance(currentSpeedSquared, maxCorneringSpeedSquared, cornerDecel);

                if (brakingDistance > distanceTravelled - _currentVecProgress)
                {
                    maxSpeed = Math.Min(maxCorneringSpeed, maxSpeed);
                }
            }

            if (_rules.Enabled && stopId < 0 && pointId != _clearedStopId)
            {
                var j = _rules.GetStop(pointId);
                if (j != null)
                {
                    stopId = pointId;
                    stopJct = j;
                    stopDist = distanceTravelled - _currentVecProgress;
                }
            }
        }

        if (_stopPointId >= 0 && stopId != _stopPointId && !_waitingAtStop && _stopPointId != _clearedStopId && _stopJunction != null)
        {
            _rules.DebugLog("AI {Id} RAN the stop at {Junction} (speed {Speed:0.0} m/s)", EntryCar.SessionId, _stopJunction.Name, CurrentSpeed);
        }
        if (stopId != _stopPointId && _waitingAtStop && _stopJunction != null)
        {
            _rules.Leave(_stopJunction, this);
            _waitingAtStop = false;
        }
        _stopPointId = stopId;
        _stopJunction = stopJct;
        _stopDistance = stopDist;
        if (stopId >= 0)
        {
            // brake smoothly so the car stops about 1 m before the stop point
            float d = MathF.Max(0, stopDist - 1.0f);
            float v = MathF.Sqrt(2 * EntryCar.AiDeceleration * _rules.StopDecelFactor * d);
            maxSpeed = Math.Min(maxSpeed, v);
        }

        return (closestAiState, closestAiStateDistance, maxSpeed);
    }

    private bool ShouldIgnorePlayerObstacles()
    {
        if (_configuration.Extra.AiParams.IgnorePlayerObstacleSpheres != null)
        {
            foreach (var sphere in _configuration.Extra.AiParams.IgnorePlayerObstacleSpheres)
            {
                if (Vector3.DistanceSquared(Status.Position, sphere.Center) < sphere.RadiusMeters * sphere.RadiusMeters)
                {
                    return true;
                }
            }
        }

        return false;
    }

    private (EntryCar? entryCar, float distance) FindClosestPlayerObstacle()
    {
        if (!ShouldIgnorePlayerObstacles())
        {
            EntryCar? closestCar = null;
            float minDistance = float.MaxValue;
            for (var i = 0; i < _entryCarManager.EntryCars.Length; i++)
            {
                var playerCar = _entryCarManager.EntryCars[i];
                if (playerCar.Client?.HasSentFirstUpdate == true)
                {
                    float distance = Vector3.DistanceSquared(playerCar.Status.Position, Status.Position);

                    if (distance < minDistance && GetAngleToCar(playerCar.Status) is > 166 and < 194)
                    {
                        minDistance = distance;
                        closestCar = playerCar;
                    }
                }
            }

            if (closestCar != null)
            {
                return (closestCar, MathF.Sqrt(minDistance));
            }
        }

        return (null, float.MaxValue);
    }

    private bool IsObstacle(EntryCar playerCar)
    {
        float aiRectWidth = 4; // Lane width
        float halfAiRectWidth = aiRectWidth / 2;
        float aiRectLength = 10; // length of rectangle infront of ai traffic
        float aiRectOffset = 1; // offset of the rectangle from ai position

        float obstacleRectWidth = 1; // width of obstacle car 
        float obstacleRectLength = 1; // length of obstacle car
        float halfObstacleRectWidth = obstacleRectWidth / 2;
        float halfObstanceRectLength = obstacleRectLength / 2;

        Vector3 forward = Vector3.Transform(-Vector3.UnitX, Matrix4x4.CreateRotationY(Status.Rotation.X));
        Matrix4x4 aiViewMatrix = Matrix4x4.CreateLookAt(Status.Position, Status.Position + forward, Vector3.UnitY);

        Matrix4x4 targetWorldViewMatrix = Matrix4x4.CreateRotationY(playerCar.Status.Rotation.X) * Matrix4x4.CreateTranslation(playerCar.Status.Position) * aiViewMatrix;

        Vector3 targetFrontLeft = Vector3.Transform(new Vector3(-halfObstanceRectLength, 0, halfObstacleRectWidth), targetWorldViewMatrix);
        Vector3 targetFrontRight = Vector3.Transform(new Vector3(-halfObstanceRectLength, 0, -halfObstacleRectWidth), targetWorldViewMatrix);
        Vector3 targetRearLeft = Vector3.Transform(new Vector3(halfObstanceRectLength, 0, halfObstacleRectWidth), targetWorldViewMatrix);
        Vector3 targetRearRight = Vector3.Transform(new Vector3(halfObstanceRectLength, 0, -halfObstacleRectWidth), targetWorldViewMatrix);

        static bool IsPointInside(Vector3 point, float width, float length, float offset)
            => MathF.Abs(point.X) >= width || (-point.Z >= offset && -point.Z <= offset + length);

        bool isObstacle = IsPointInside(targetFrontLeft, halfAiRectWidth, aiRectLength, aiRectOffset)
                          || IsPointInside(targetFrontRight, halfAiRectWidth, aiRectLength, aiRectOffset)
                          || IsPointInside(targetRearLeft, halfAiRectWidth, aiRectLength, aiRectOffset)
                          || IsPointInside(targetRearRight, halfAiRectWidth, aiRectLength, aiRectOffset);

        return isObstacle;
    }

    public void DetectObstacles()
    {
        if (!Initialized) return;
            
        if (_sessionManager.ServerTimeMilliseconds < _ignoreObstaclesUntil)
        {
            if (_rules.Enabled && _stopPointId >= 0 && _stopDistance < 80)
            {
                _ignoreObstaclesUntil = 0; // never run a stop sign while ignoring obstacles
            }
            else
            {
                SetTargetSpeed(MaxSpeed);
                return;
            }
        }

        if (_sessionManager.ServerTimeMilliseconds < _stoppedForCollisionUntil)
        {
            SetTargetSpeed(0);
            return;
        }
            
        float targetSpeed = InitialMaxSpeed;
        float maxSpeed = InitialMaxSpeed;
        bool hasObstacle = false;

        var splineLookahead = SplineLookahead();
        var playerObstacle = FindClosestPlayerObstacle();

        ClosestAiObstacleDistance = splineLookahead.ClosestAiState != null ? splineLookahead.ClosestAiStateDistance : -1;

        if (playerObstacle.distance < _minObstacleDistance || splineLookahead.ClosestAiStateDistance < _minObstacleDistance)
        {
            targetSpeed = 0;
            hasObstacle = true;
        }
        else if (playerObstacle.distance < splineLookahead.ClosestAiStateDistance && playerObstacle.entryCar != null)
        {
            float playerSpeed = playerObstacle.entryCar.Status.Velocity.Length();

            if (playerSpeed < 0.1f)
            {
                playerSpeed = 0;
            }

            if ((playerSpeed < CurrentSpeed || playerSpeed == 0)
                && playerObstacle.distance < PhysicsUtils.CalculateBrakingDistance(CurrentSpeed - playerSpeed, EntryCar.AiDeceleration) * 2 + 20)
            {
                targetSpeed = Math.Max(WalkingSpeed, playerSpeed);
                hasObstacle = true;
            }
        }
        else if (splineLookahead.ClosestAiState != null)
        {
            float closestTargetSpeed = Math.Min(splineLookahead.ClosestAiState.CurrentSpeed, splineLookahead.ClosestAiState.TargetSpeed);
            if ((closestTargetSpeed < CurrentSpeed || splineLookahead.ClosestAiState.CurrentSpeed == 0)
                && splineLookahead.ClosestAiStateDistance < PhysicsUtils.CalculateBrakingDistance(CurrentSpeed - closestTargetSpeed, EntryCar.AiDeceleration) * 2 + 20)
            {
                targetSpeed = Math.Max(WalkingSpeed, closestTargetSpeed);
                hasObstacle = true;
            }
        }

        targetSpeed = Math.Min(splineLookahead.MaxSpeed, targetSpeed);

        // traffic rules: wait at the stop point until the junction is clear
        long now = _sessionManager.ServerTimeMilliseconds;
        if (_stopPointId >= 0 && _stopJunction != null)
        {
            if (!_waitingAtStop && _stopDistance < 3.0f && CurrentSpeed < 0.3f)
            {
                _waitingAtStop = true;
                _waitingSince = now;
                _rules.Arrive(_stopJunction, this, now);
            }

            if (_waitingAtStop)
            {
                if (_rules.CanGo(_stopJunction, this, now, _waitingSince))
                {
                    _rules.Leave(_stopJunction, this);
                    _clearedStopId = _stopPointId;
                    _waitingAtStop = false;
                    _stopPointId = -1;
                    _stopJunction = null;
                    _stopDistance = float.MaxValue;
                }
                else
                {
                    targetSpeed = 0;
                }
            }
            else if (_stopDistance < 1.0f)
            {
                targetSpeed = 0;
            }
        }
        // in a stop queue = waiting at a stop, or stopped behind a car that is in a stop queue
        InStopQueue = _waitingAtStop || (hasObstacle && splineLookahead.ClosestAiState?.InStopQueue == true && CurrentSpeed < 0.5f);
        if (InStopQueue)
        {
            // no honking, no 'ignore obstacles' timeout while queueing at a stop sign
            _stoppedForObstacle = false;
        }
        else if (CurrentSpeed == 0 && !_stoppedForObstacle)
        {
            _stoppedForObstacle = true;
            _stoppedForObstacleSince = _sessionManager.ServerTimeMilliseconds;
            _obstacleHonkStart = _stoppedForObstacleSince + Random.Shared.Next(3000, 7000);
            _obstacleHonkEnd = _obstacleHonkStart + Random.Shared.Next(500, 1500);
            Log.Verbose("AI {SessionId} stopped for obstacle", EntryCar.SessionId);
        }
        else if (CurrentSpeed > 0 && _stoppedForObstacle)
        {
            _stoppedForObstacle = false;
            Log.Verbose("AI {SessionId} no longer stopped for obstacle", EntryCar.SessionId);
        }
        else if (_stoppedForObstacle && _sessionManager.ServerTimeMilliseconds - _stoppedForObstacleSince > _configuration.Extra.AiParams.IgnoreObstaclesAfterMilliseconds)
        {
            _ignoreObstaclesUntil = _sessionManager.ServerTimeMilliseconds + 10_000;
            Log.Verbose("AI {SessionId} ignoring obstacles until {IgnoreObstaclesUntil}", EntryCar.SessionId, _ignoreObstaclesUntil);
        }

        float deceleration = EntryCar.AiDeceleration;
        bool brakingForStop = _stopPointId >= 0 && _stopDistance < 80 && targetSpeed < CurrentSpeed;
        if (!hasObstacle && !brakingForStop)
        {
            deceleration *= EntryCar.AiCorneringBrakeForceFactor;
        }
        
        MaxSpeed = maxSpeed;
        SetTargetSpeed(targetSpeed, deceleration, EntryCar.AiAcceleration);
    }

    public void StopForCollision()
    {
        if (!ShouldIgnorePlayerObstacles())
        {
            _stoppedForCollisionUntil = _sessionManager.ServerTimeMilliseconds + Random.Shared.Next(EntryCar.AiMinCollisionStopTimeMilliseconds, EntryCar.AiMaxCollisionStopTimeMilliseconds);
        }
    }

    public float GetAngleToCar(CarStatus car)
    {
        float challengedAngle = (float) (Math.Atan2(Status.Position.X - car.Position.X, Status.Position.Z - car.Position.Z) * 180 / Math.PI);
        if (challengedAngle < 0)
            challengedAngle += 360;
        float challengedRot = Status.GetRotationAngle();

        challengedAngle += challengedRot;
        challengedAngle %= 360;

        return challengedAngle;
    }

    private void SetTargetSpeed(float speed, float deceleration, float acceleration)
    {
        TargetSpeed = speed;
        if (speed < CurrentSpeed)
        {
            Acceleration = -deceleration;
        }
        else if (speed > CurrentSpeed)
        {
            Acceleration = acceleration;
        }
        else
        {
            Acceleration = 0;
        }
    }

    private void SetTargetSpeed(float speed)
    {
        SetTargetSpeed(speed, EntryCar.AiDeceleration, EntryCar.AiAcceleration);
    }

    public void Update()
    {
        if (!Initialized)
            return;

        var ops = _spline.Operations;

        long currentTime = _sessionManager.ServerTimeMilliseconds;
        long dt = currentTime - _lastTick;
        _lastTick = currentTime;

        if (Acceleration != 0)
        {
            CurrentSpeed += Acceleration * (dt / 1000.0f);
                
            if ((Acceleration < 0 && CurrentSpeed < TargetSpeed) || (Acceleration > 0 && CurrentSpeed > TargetSpeed))
            {
                CurrentSpeed = TargetSpeed;
                Acceleration = 0;
            }
        }

        float moveMeters = (dt / 1000.0f) * CurrentSpeed;
        if (!Move(_currentVecProgress + moveMeters) || !_junctionEvaluator.TryNext(CurrentSplinePointId, out var nextPoint))
        {
            Log.Debug("Car {SessionId} reached spline end, despawning", EntryCar.SessionId);
            Despawn();
            return;
        }

        CatmullRom.CatmullRomPoint smoothPos = CatmullRom.Evaluate(ops.Points[CurrentSplinePointId].Position, 
            ops.Points[nextPoint].Position, 
            _startTangent, 
            _endTangent, 
            _currentVecProgress / _currentVecLength);
            
        Vector3 rotation = new Vector3
        {
            X = MathF.Atan2(smoothPos.Tangent.Z, smoothPos.Tangent.X) - MathF.PI / 2,
            Y = (MathF.Atan2(new Vector2(smoothPos.Tangent.Z, smoothPos.Tangent.X).Length(), smoothPos.Tangent.Y) - MathF.PI / 2) * -1f,
            Z = ops.GetCamber(CurrentSplinePointId, _currentVecProgress / _currentVecLength)
        };

        float dtSec = dt / 1000.0f;
        float yawRate = 0; // still used for body roll
        if (_hasLastYaw && dtSec > 0 && CurrentSpeed > 1.0f)
        {
            float yawDelta = rotation.X - _lastYaw;
            if (yawDelta > MathF.PI) yawDelta -= 2 * MathF.PI;
            else if (yawDelta < -MathF.PI) yawDelta += 2 * MathF.PI;
            yawRate = yawDelta / dtSec;
        }
        _lastYaw = rotation.X;

        // Seacrest 2026-10-26: wheel angle from the spline shape (curvature at the car), not from yaw rate per tick.
        // Same angle at any speed, kept while standing still (wheels stay turned at a STOP), smoothed over distance.
        float targetWheelAngleDeg = GeometricWheelAngleDeg(nextPoint, _currentVecProgress / _currentVecLength);
        if (!_hasLastYaw)
            _smoothedWheelAngleDeg = targetWheelAngleDeg;
        else
            _smoothedWheelAngleDeg += (targetWheelAngleDeg - _smoothedWheelAngleDeg) * MathF.Min(1f, moveMeters / WheelSmoothMeters);
        _hasLastYaw = true;

        float lateralG = CurrentSpeed * yawRate / 9.81f;
        float targetRollDeg = Math.Clamp(lateralG * RollDegPerG, -MaxRollDeg, MaxRollDeg);
        float targetRollRad = RollSign * targetRollDeg * MathF.PI / 180f;
        _smoothedRollRad += (targetRollRad - _smoothedRollRad) * MathF.Min(1f, dtSec * 5f);
        rotation.Z += _smoothedRollRad;

        // pitch from measured speed change (positive = accelerating)
        float longAccel = 0;
        if (dtSec > 0)
            longAccel = (CurrentSpeed - _lastSpeed) / dtSec;
        _smoothedLongAccel += (longAccel - _smoothedLongAccel) * MathF.Min(1f, dtSec * 6f);
        float longG = _smoothedLongAccel / 9.81f;
        float targetPitchDeg = Math.Clamp(longG * PitchDegPerG, -MaxPitchDeg, MaxPitchDeg);
        float targetPitchRad = PitchSign * targetPitchDeg * MathF.PI / 180f;
        _smoothedPitchRad += (targetPitchRad - _smoothedPitchRad) * MathF.Min(1f, dtSec * 4f);
        rotation.Y += _smoothedPitchRad;
        _lastSpeed = CurrentSpeed;

        // Seacrest 2026-10-25: drift (after wheel/roll/pitch, so the yaw rate above stays the spline's own)
        Vector3 position = smoothPos.Position;
        float wheelDeg = _smoothedWheelAngleDeg;
        float rearSpin = 1f;
        if (_driftCar)
        {
            float target = 0;
            float w = _drift.Weight(CurrentSplinePointId);
            if (w > 0 && CurrentSpeed > 3.0f && CurrentSpeed * 3.6f >= _drift.Settings.MinSpeedKmh)
            {
                target = w;
                _driftSide = _drift.Side(CurrentSplinePointId);
                if (!_driftLogged && _drift.Settings.Debug)
                {
                    Log.Debug("Drift: AI {SessionId} drifting in {Zone} (point {Point}, {Speed:0} km/h)",
                        EntryCar.SessionId, _drift.ZoneName(CurrentSplinePointId), CurrentSplinePointId, CurrentSpeed * 3.6f);
                    _driftLogged = true;
                }
            }
            else
            {
                _driftLogged = false;
            }
            _driftFactor += (target - _driftFactor) * MathF.Min(1f, dtSec * _drift.Settings.BlendRate);
            if (target == 0 && _driftFactor < 0.002f) _driftFactor = 0;

            var tan = new Vector2(smoothPos.Tangent.X, smoothPos.Tangent.Z);
            if (_driftFactor > 0 && _driftSide != 0 && tan.LengthSquared() > 1e-6f)
            {
                var s = _drift.Settings;
                tan = Vector2.Normalize(tan);
                var left = new Vector2(tan.Y, -tan.X); // same 'left' as AdjacentLaneDetector / DriftZones
                float a = s.AngleDeg * MathF.PI / 180f * _driftFactor * _driftSide; // nose toward the inside of the corner
                var nose = tan * MathF.Cos(a) + left * MathF.Sin(a);
                float newYaw = MathF.Atan2(nose.Y, nose.X) - MathF.PI / 2;
                float yawOffset = newYaw - rotation.X;
                if (yawOffset > MathF.PI) yawOffset -= 2 * MathF.PI;
                else if (yawOffset < -MathF.PI) yawOffset += 2 * MathF.PI;
                rotation.X = newYaw;

                // slide a bit to the outside of the corner
                float outward = s.OutwardMeters * _driftFactor;
                position.X -= _driftSide * left.X * outward;
                position.Z -= _driftSide * left.Y * outward;

                // counter-steer: front wheels against the body rotation
                float counterDeg = Math.Clamp(-yawOffset * 180f / MathF.PI * s.CounterSteerRatio, -MaxWheelAngleDeg, MaxWheelAngleDeg);
                wheelDeg = _smoothedWheelAngleDeg + (counterDeg - _smoothedWheelAngleDeg) * _driftFactor;

                rearSpin = 1f + (s.RearWheelSpin - 1f) * _driftFactor;
            }
        }

        byte encodedWheel = (byte)Math.Clamp(MathF.Round(127 + WheelAngleSign * wheelDeg * WheelAngleScale), 0, 255);

        float tyreAngularSpeed = GetTyreAngularSpeed(CurrentSpeed, EntryCar.TyreDiameterMeters);
        byte encodedTyreAngularSpeed = EncodeTyreAngularSpeed(tyreAngularSpeed);
        byte encodedRearTyreAngularSpeed = rearSpin != 1f ? EncodeTyreAngularSpeed(tyreAngularSpeed * rearSpin) : encodedTyreAngularSpeed;

        Status.Timestamp = _sessionManager.ServerTimeMilliseconds;
        Status.Position = position with { Y = position.Y + EntryCar.AiSplineHeightOffsetMeters };
        Status.Rotation = rotation;
        Status.Velocity = smoothPos.Tangent * CurrentSpeed;

        Status.SteerAngle = encodedWheel;
        Status.WheelAngle = encodedWheel;
        Status.TyreAngularSpeed[0] = encodedTyreAngularSpeed;
        Status.TyreAngularSpeed[1] = encodedTyreAngularSpeed;
        Status.TyreAngularSpeed[2] = encodedRearTyreAngularSpeed;
        Status.TyreAngularSpeed[3] = encodedRearTyreAngularSpeed;
        Status.EngineRpm = (ushort)MathUtils.Lerp(EntryCar.AiIdleEngineRpm, EntryCar.AiMaxEngineRpm, CurrentSpeed / _configuration.Extra.AiParams.MaxSpeedMs);
        Status.StatusFlag = CarStatusFlags.LightsOn
                            | CarStatusFlags.HighBeamsOff
                            | (_sessionManager.ServerTimeMilliseconds < _stoppedForCollisionUntil || (CurrentSpeed < 20 / 3.6f && !InStopQueue && !(_stopPointId >= 0 && _stopDistance < 60)) ? CarStatusFlags.HazardsOn : 0)
                            | (CurrentSpeed == 0 || Acceleration < 0 ? CarStatusFlags.BrakeLightsOn : 0)
                            | (_stoppedForObstacle && _sessionManager.ServerTimeMilliseconds > _obstacleHonkStart && _sessionManager.ServerTimeMilliseconds < _obstacleHonkEnd ? CarStatusFlags.Horn : 0)
                            | GetWiperSpeed(_weatherManager.CurrentWeather.RainIntensity)
                            | _indicator;
        Status.Gear = 2;
    }
        
    private static float GetTyreAngularSpeed(float speed, float wheelDiameter)
    {
        return speed / (MathF.PI * wheelDiameter) * 6;
    }

    private static byte EncodeTyreAngularSpeed(float tyreAngularSpeed)
    {
        return (byte) (Math.Clamp(MathF.Round(MathF.Log10(tyreAngularSpeed + 1.0f) * 20.0f) * Math.Sign(tyreAngularSpeed), -100.0f, 154.0f) + 100.0f);
    }

    private static CarStatusFlags GetWiperSpeed(float rainIntensity)
    {
        return rainIntensity switch
        {
            < 0.05f => 0,
            < 0.25f => CarStatusFlags.WiperLevel1,
            < 0.5f => CarStatusFlags.WiperLevel2,
            _ => CarStatusFlags.WiperLevel3
        };
    }
}

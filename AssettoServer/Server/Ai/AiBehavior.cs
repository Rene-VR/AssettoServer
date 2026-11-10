using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using AssettoServer.Network.Tcp;
using AssettoServer.Server.Ai.Splines;
using AssettoServer.Server.Configuration;
using AssettoServer.Server.Plugin;
using AssettoServer.Shared.Network.Packets.Outgoing;
using AssettoServer.Shared.Services;
using AssettoServer.Utils;
using Microsoft.Extensions.Hosting;
using Prometheus;
using Serilog;

namespace AssettoServer.Server.Ai;

public class AiBehavior : CriticalBackgroundService, IAssettoServerAutostart
{
    private readonly ACServerConfiguration _configuration;
    private readonly SessionManager _sessionManager;
    private readonly EntryCarManager _entryCarManager;
    private readonly AiSpline _spline;
    //private readonly EntryCar.Factory _entryCarFactory;

    private readonly JunctionEvaluator _junctionEvaluator;

    private readonly Gauge _aiStateCountMetric = Metrics.CreateGauge("assettoserver_aistatecount", "Number of AI states");

    private readonly Summary _updateDurationTimer;
    private readonly Summary _obstacleDetectionDurationTimer;

    public AiBehavior(SessionManager sessionManager,
        ACServerConfiguration configuration,
        EntryCarManager entryCarManager,
        IHostApplicationLifetime applicationLifetime,
        //EntryCar.Factory entryCarFactory,
        CSPServerScriptProvider serverScriptProvider, 
        AiSpline spline) : base(applicationLifetime)
    {
        _sessionManager = sessionManager;
        _configuration = configuration;
        _entryCarManager = entryCarManager;
        _spline = spline;
        _junctionEvaluator = new JunctionEvaluator(spline, false);
        //_entryCarFactory = entryCarFactory;

        if (_configuration.Extra.AiParams.Debug)
        {
            using var stream = Assembly.GetExecutingAssembly().GetManifestResourceStream("AssettoServer.Server.Ai.ai_debug.lua")!;
            serverScriptProvider.AddScript(stream, "ai_debug.lua");
        }

        _updateDurationTimer = Metrics.CreateSummary("assettoserver_aibehavior_update", "AiBehavior.Update Duration", MetricDefaults.DefaultQuantiles);
        _obstacleDetectionDurationTimer = Metrics.CreateSummary("assettoserver_aibehavior_obstacledetection", "AiBehavior.ObstacleDetection Duration", MetricDefaults.DefaultQuantiles);

        _entryCarManager.ClientConnected += (client, _) =>
        {
            client.ChecksumPassed += OnClientChecksumPassed;
            client.Collision += OnCollision;
        };

        _entryCarManager.ClientDisconnected += OnClientDisconnected;
        _configuration.Extra.AiParams.PropertyChanged += (_, _) => AdjustOverbooking();
    }

    private static void OnCollision(ACTcpClient sender, CollisionEventArgs args)
    {
        if (args.TargetCar?.AiControlled == true)
        {
            var targetAiState = args.TargetCar.GetClosestAiState(sender.EntryCar.Status.Position);
            if (targetAiState.AiState != null && targetAiState.DistanceSquared < 25 * 25)
            {
                Task.Delay(Random.Shared.Next(100, 500)).ContinueWith(_ => targetAiState.AiState.StopForCollision());
            }
        }
    }

    private void OnClientChecksumPassed(ACTcpClient sender, EventArgs args)
    {
        sender.EntryCar.SetAiControl(false);
        AdjustOverbooking();
    }

    private async Task ObstacleDetectionAsync(CancellationToken stoppingToken)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(100));

        while (await timer.WaitForNextTickAsync(stoppingToken))
        {
            try
            {
                using var context = _obstacleDetectionDurationTimer.NewTimer();

                for (int i = 0; i < _entryCarManager.EntryCars.Length; i++)
                {
                    var entryCar = _entryCarManager.EntryCars[i];
                    if (entryCar.AiControlled)
                    {
                        entryCar.AiObstacleDetection();
                    }
                }

                if (_configuration.Extra.AiParams.Debug)
                {
                    SendDebugPackets();
                }
            }
            catch (Exception ex)
            {
                Log.Error(ex, "Error in AI obstacle detection");
            }
        }
    }

    private void SendDebugPackets()
    {
        CountedArray<byte> sessionIds = new(_entryCarManager.EntryCars.Length);
        CountedArray<byte> currentSpeeds = new(_entryCarManager.EntryCars.Length);
        CountedArray<byte> targetSpeeds = new(_entryCarManager.EntryCars.Length);
        CountedArray<byte> maxSpeeds = new(_entryCarManager.EntryCars.Length);
        CountedArray<short> closestAiObstacles = new(_entryCarManager.EntryCars.Length);
        foreach (var player in _entryCarManager.ConnectedCars.Values)
        {
            if (player.Client?.HasSentFirstUpdate == false) continue;

            sessionIds.Clear();
            currentSpeeds.Clear();
            targetSpeeds.Clear();
            maxSpeeds.Clear();
            closestAiObstacles.Clear();

            foreach (var car in _entryCarManager.EntryCars)
            {
                if (!car.AiControlled) continue;

                var (aiState, _) = car.GetClosestAiState(player.Status.Position);
                if (aiState == null) continue;

                sessionIds.Add(car.SessionId);
                currentSpeeds.Add((byte)(aiState.CurrentSpeed * 3.6f));
                targetSpeeds.Add((byte)(aiState.TargetSpeed * 3.6f));
                maxSpeeds.Add((byte)(aiState.MaxSpeed * 3.6f));
                closestAiObstacles.Add((short)aiState.ClosestAiObstacleDistance);
            }

            for (int i = 0; i < sessionIds.Count; i += AiDebugPacket.Length)
            {
                var packet = new AiDebugPacket();
                Array.Fill(packet.SessionIds, (byte)255);

                new ArraySegment<byte>(sessionIds.Array, i, Math.Min(AiDebugPacket.Length, sessionIds.Count - i)).CopyTo(packet.SessionIds);
                new ArraySegment<short>(closestAiObstacles.Array, i, Math.Min(AiDebugPacket.Length, sessionIds.Count - i)).CopyTo(packet.ClosestAiObstacles);
                new ArraySegment<byte>(currentSpeeds.Array, i, Math.Min(AiDebugPacket.Length, sessionIds.Count - i)).CopyTo(packet.CurrentSpeeds);
                new ArraySegment<byte>(maxSpeeds.Array, i, Math.Min(AiDebugPacket.Length, sessionIds.Count - i)).CopyTo(packet.MaxSpeeds);
                new ArraySegment<byte>(targetSpeeds.Array, i, Math.Min(AiDebugPacket.Length, sessionIds.Count - i)).CopyTo(packet.TargetSpeeds);

                player.Client?.SendPacket(packet);
            }
        }
    }
    
    // Maximum number of AI cars on the road at the same time (set in AdjustOverbooking).
    // Every AI slot keeps at least one AI state, so the car model/skin of a new traffic car
    // is picked at random from ALL slots, not from the first slots of entry_list.ini.
    private volatile int _targetOnRoad = int.MaxValue;

    private static void Shuffle<T>(IList<T> list)
    {
        for (int i = list.Count - 1; i > 0; i--)
        {
            int j = Random.Shared.Next(i + 1);
            (list[i], list[j]) = (list[j], list[i]);
        }
    }

    private readonly List<EntryCar> _playerCars = new();
    private readonly List<AiState> _initializedAiStates = new();
    private readonly List<AiState> _uninitializedAiStates = new();
    private readonly List<Vector3> _playerOffsetPositions = new();
    private readonly List<KeyValuePair<AiState, float>> _aiMinDistanceToPlayer = new();
    private readonly List<KeyValuePair<EntryCar, float>> _playerMinDistanceToAi = new();
    private void Update()
    {
        using var context = _updateDurationTimer.NewTimer();

        _playerCars.Clear();
        _initializedAiStates.Clear();
        _uninitializedAiStates.Clear();
        _playerOffsetPositions.Clear();
        _aiMinDistanceToPlayer.Clear();
        _playerMinDistanceToAi.Clear();

        foreach (var entryCar in _entryCarManager.EntryCars)
        {
            var (currentSplinePointId, _) = _spline.WorldToSpline(entryCar.Status.Position);
            var drivingTheRightWay = Vector3.Dot(_spline.Operations.GetForwardVector(currentSplinePointId), entryCar.Status.Velocity) > 0;

            if (!entryCar.AiControlled
                && entryCar.Client?.HasSentFirstUpdate == true
                && _sessionManager.ServerTimeMilliseconds - entryCar.LastActiveTime < _configuration.Extra.AiParams.PlayerAfkTimeoutMilliseconds
                && (_configuration.Extra.AiParams.TwoWayTraffic || _configuration.Extra.AiParams.WrongWayTraffic || drivingTheRightWay))
            {
                _playerCars.Add(entryCar);
            }
            else if (entryCar.AiControlled)
            {
                entryCar.RemoveUnsafeStates();
                entryCar.GetInitializedStates(_initializedAiStates, _uninitializedAiStates);
            }
            
        }

        _aiStateCountMetric.Set(_initializedAiStates.Count);

        for (int i = 0; i < _initializedAiStates.Count; i++)
        {
            _aiMinDistanceToPlayer.Add(new KeyValuePair<AiState, float>(_initializedAiStates[i], float.MaxValue));
        }

        for (int i = 0; i < _playerCars.Count; i++)
        {
            _playerMinDistanceToAi.Add(new KeyValuePair<EntryCar, float>(_playerCars[i], float.MaxValue));
        }

        // Get minimum distance to a player for each AI
        // Get minimum distance to AI for each player
        for (int i = 0; i < _initializedAiStates.Count; i++)
        {
            for (int j = 0; j < _playerCars.Count; j++)
            {
                if (_playerOffsetPositions.Count <= j)
                {
                    var offsetPosition = _playerCars[j].Status.Position;
                    if (_playerCars[j].Status.Velocity != Vector3.Zero)
                    {
                        offsetPosition += Vector3.Normalize(_playerCars[j].Status.Velocity) * _configuration.Extra.AiParams.PlayerPositionOffsetMeters;
                    }

                    _playerOffsetPositions.Add(offsetPosition);
                }

                var distanceSquared = Vector3.DistanceSquared(_initializedAiStates[i].Status.Position, _playerOffsetPositions[j]);

                if (_aiMinDistanceToPlayer[i].Value > distanceSquared)
                {
                    _aiMinDistanceToPlayer[i] = new KeyValuePair<AiState, float>(_initializedAiStates[i], distanceSquared);
                }

                if (_playerMinDistanceToAi[j].Value > distanceSquared)
                {
                    _playerMinDistanceToAi[j] = new KeyValuePair<EntryCar, float>(_playerCars[j], distanceSquared);
                }
            }
        }
        
        // Order AI cars by their minimum distance to a player. Higher distance = higher chance for respawn
        _aiMinDistanceToPlayer.Sort((a, b) => b.Value.CompareTo(a.Value));

        foreach (var dist in _aiMinDistanceToPlayer)
        {
            if (dist.Value > _configuration.Extra.AiParams.PlayerRadiusSquared
                && _sessionManager.ServerTimeMilliseconds > dist.Key.SpawnProtectionEnds)
            {
                _uninitializedAiStates.Add(dist.Key);
            }
        }
        
        if (_initializedAiStates.Count > 0 && _playerCars.Count > 0)
        {
            _playerCars.Clear();
            // Order player cars by their minimum distance to an AI. Higher distance = higher chance for next AI spawn
            _playerMinDistanceToAi.Sort((a, b) => b.Value.CompareTo(a.Value));
            for (int i = 0; i < _playerMinDistanceToAi.Count; i++)
            {
                _playerCars.Add(_playerMinDistanceToAi[i].Key);
            }
        }

        int onRoad = _initializedAiStates.Count;
        int targetOnRoad = _targetOnRoad;

        // Too many cars on the road (e.g. a player left): remove far away cars until the target is reached
        while (onRoad > targetOnRoad)
        {
            var excess = _uninitializedAiStates.FirstOrDefault(state => state.Initialized);
            if (excess == null) break;

            excess.Despawn();
            onRoad--;
        }

        // Random order = random car model/skin, independent of the order in entry_list.ini
        Shuffle(_uninitializedAiStates);

        while (_playerCars.Count > 0 && _uninitializedAiStates.Count > 0)
        {
            int spawnPointId = -1;
            while (spawnPointId < 0 && _playerCars.Count > 0)
            {
                var targetPlayerCar = _playerCars.ElementAt(GetRandomWeighted(_playerCars.Count));
                _playerCars.Remove(targetPlayerCar);

                spawnPointId = GetSpawnPoint(targetPlayerCar);
            }

            if (spawnPointId < 0 || !_junctionEvaluator.TryNext(spawnPointId, out _))
                continue;

            var previousAi = FindClosestAiState(spawnPointId, false);
            var nextAi = FindClosestAiState(spawnPointId, true);

            foreach (var targetAiState in _uninitializedAiStates)
            {
                if (!targetAiState.CanSpawn(spawnPointId, previousAi, nextAi))
                    continue;

                if (!targetAiState.Initialized)
                {
                    if (onRoad >= targetOnRoad)
                    {
                        // Target reached: swap a far away car for this new one so the car models rotate
                        var retire = _uninitializedAiStates.FirstOrDefault(state => state.Initialized);
                        if (retire == null) continue;

                        retire.Despawn();
                        onRoad--;
                    }

                    onRoad++;
                }

                targetAiState.Teleport(spawnPointId);

                _uninitializedAiStates.Remove(targetAiState);
                break;
            }
        }
    }

    private AiState? FindClosestAiState(int pointId, bool forward)
    {
        var points = _spline.Points;
        float distanceTravelled = 0;
        float searchDistance = 50;
        ref readonly var point = ref points[pointId];
        
        AiState? closestAiState = null;
        while (distanceTravelled < searchDistance && closestAiState == null)
        {
            distanceTravelled += point.Length;
            // TODO reuse this junction evaluator for the newly spawned car
            pointId = forward ? _junctionEvaluator.Next(pointId) : _junctionEvaluator.Previous(pointId);
            if (pointId < 0)
                break;

            point = ref points[pointId];
            
            var slowest = _spline.SlowestAiStates[pointId];
            if (slowest != null)
            {
                closestAiState = slowest;
            }
        }

        return closestAiState;
    }

    private async Task UpdateAsync(CancellationToken stoppingToken)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(_configuration.Extra.AiParams.AiBehaviorUpdateIntervalMilliseconds));

        while (await timer.WaitForNextTickAsync(stoppingToken))
        {
            try
            {
                Update();
            }
            catch (Exception ex)
            {
                Log.Error(ex, "Error in AI update");
            }
        }
    }

    private void OnClientDisconnected(ACTcpClient sender, EventArgs args)
    {
        if (sender.EntryCar.AiMode != AiMode.None)
        {
            sender.EntryCar.SetAiControl(true);
            AdjustOverbooking();
        }
    }

    private int GetRandomWeighted(int max)
    {
        // Probabilities for max = 4
        // 0    4/10
        // 1    3/10
        // 2    2/10
        // 3    1/10
            
        int maxRand = max * (max + 1) / 2;
        int rand = Random.Shared.Next(maxRand);
        int target = 0;
        for (int i = max; i < maxRand; i += (i - 1))
        {
            if (rand < i) break;
            target++;
        }

        return target;
    }

    private bool IsPositionSafe(int pointId)
    {
        var ops = _spline.Operations;

        for (var i = 0; i < _entryCarManager.EntryCars.Length; i++)
        {
            var entryCar = _entryCarManager.EntryCars[i];
            if (entryCar.AiControlled && !entryCar.IsPositionSafe(pointId))
            {
                return false;
            }

            if (entryCar.Client?.HasSentFirstUpdate == true
                && Vector3.DistanceSquared(entryCar.Status.Position, ops.Points[pointId].Position) < _configuration.Extra.AiParams.SpawnSafetyDistanceToPlayerSquared)
            {
                return false;
            }
        }

        return true;
    }

    private int GetSpawnPoint(EntryCar playerCar)
    {
        var result = _spline.WorldToSpline(playerCar.Status.Position);
        var ops = _spline.Operations;


        if (result.PointId < 0 || ops.Points[result.PointId].NextId < 0) return -1;
        
        int direction = Vector3.Dot(ops.GetForwardVector(result.PointId), playerCar.Status.Velocity) > 0 ? 1 : -1;
        
        // Do not not spawn if a player is too far away from the AI spline, e.g. in pits or in a part of the map without traffic
        if (result.DistanceSquared > _configuration.Extra.AiParams.MaxPlayerDistanceToAiSplineSquared)
        {
            return -1;
        }
        
        // Seacrest 2026-11-01: separate spawn distances for cars driving the same way as the player (in front)
        // and ONCOMING cars. Oncoming range 0 = same as the normal range.
        var aiParams = _configuration.Extra.AiParams;
        int minFront = aiParams.MinSpawnDistancePoints;
        int maxFront = aiParams.MaxSpawnDistancePoints;
        int minOncoming = aiParams.MinOncomingSpawnDistancePoints > 0 ? aiParams.MinOncomingSpawnDistancePoints : minFront;
        int maxOncoming = aiParams.MaxOncomingSpawnDistancePoints > 0 ? aiParams.MaxOncomingSpawnDistancePoints : maxFront;
        bool splitDistances = (minOncoming != minFront || maxOncoming != maxFront) && playerCar.Status.Velocity.LengthSquared() > 1f;
        int minAny = Math.Min(minFront, minOncoming);
        int maxAny = Math.Max(maxFront, maxOncoming);
        if (maxAny <= minAny) maxAny = minAny + 1;

        int spawnPointId = -1;
        bool accepted = false;
        for (int attempt = 0; attempt < 10 && !accepted; attempt++)
        {
            int spawnDistance = Random.Shared.Next(minAny, maxAny);
            spawnPointId = _junctionEvaluator.Traverse(result.PointId, spawnDistance * direction);

            if (spawnPointId >= 0)
            {
                spawnPointId = _spline.RandomLane(spawnPointId);
            }

            if (!splitDistances || spawnPointId < 0 || ops.Points[spawnPointId].NextId < 0)
            {
                accepted = true;
                break;
            }

            // the lane picked above decides which distance range applies
            bool oncoming = Vector3.Dot(ops.GetForwardVector(spawnPointId), playerCar.Status.Velocity) < 0;
            int lo = oncoming ? minOncoming : minFront;
            int hi = oncoming ? maxOncoming : maxFront;
            accepted = spawnDistance >= lo && spawnDistance <= hi;
        }

        // no lane/distance combination fitted its range: skip this spawn, the next update tries again
        if (!accepted) return -1;
        
        if (spawnPointId >= 0 && ops.Points[spawnPointId].NextId >= 0)
        {
            direction = Vector3.Dot(ops.GetForwardVector(spawnPointId), playerCar.Status.Velocity) > 0 ? 1 : -1;
        }

        while (spawnPointId >= 0 && !IsPositionSafe(spawnPointId))
        {
            spawnPointId = _junctionEvaluator.Traverse(spawnPointId, direction * 5);
        }
        
        if (spawnPointId >= 0)
        {
            spawnPointId = _spline.RandomLane(spawnPointId);
        }

        return spawnPointId;
    }

    private void AdjustOverbooking()
    {
        int playerCount = _entryCarManager.EntryCars.Count(car => car.Client != null && car.Client.IsConnected);
        var aiSlots = _entryCarManager.EntryCars.Where(car => car.Client == null && car.AiControlled).ToList(); // client null check is necessary here so that slots where someone is connecting don't count

        if (aiSlots.Count == 0)
        {
            Log.Debug("AI Slot overbooking update - no AI slots available");
            return;
        }
            
        int targetAiCount = Math.Min(playerCount * Math.Min((int)Math.Round(_configuration.Extra.AiParams.AiPerPlayerTargetCount * _configuration.Extra.AiParams.TrafficDensity), aiSlots.Count), _configuration.Extra.AiParams.MaxAiTargetCount);

        _targetOnRoad = targetAiCount;

        int overbooking = targetAiCount / aiSlots.Count;
        int rest = targetAiCount % aiSlots.Count;

        // Random slots get the extra (overbooked) AI states, not always the first ones in the entry list
        Shuffle(aiSlots);
            
        Log.Debug("AI Slot overbooking update - No. players: {NumPlayers} - No. AI Slots: {NumAiSlots} - Target AI count: {TargetAiCount} - Overbooking: {Overbooking} - Rest: {Rest}", 
            playerCount, aiSlots.Count, targetAiCount, overbooking, rest);

        for (int i = 0; i < aiSlots.Count; i++)
        {
            // Every slot keeps at least one AI state; how many are on the road at once is limited by _targetOnRoad
            aiSlots[i].SetAiOverbooking(Math.Max(1, i < rest ? overbooking + 1 : overbooking));
        }
    }

    protected override Task ExecuteAsync(CancellationToken stoppingToken)
    {
        _ = UpdateAsync(stoppingToken);
        _ = ObstacleDetectionAsync(stoppingToken);

        return Task.CompletedTask;
    }
}

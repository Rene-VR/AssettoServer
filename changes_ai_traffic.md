# AI traffic changes (based on AssettoServer v0.0.54)

Modified AssettoServer, made for a converted NFS Hot Pursuit track (Seacrest County) with AI traffic,
running on an old x86 Windows machine. Branch `ai-traffic`, based on tag `v0.0.54`.

Example config files are in [`examples/`](examples). `drift_zones.yml` and `traffic_rules.yml` are made
for one specific track (generated from its `fast_lane.aip`), so they only show the format.

## AI traffic look

1. **Front wheels steer** – wheel angle from the curvature of the spline ahead (follows the car's own
   junction choice), smoothed over distance so a standing car keeps its angle.
   Setting: `WheelAngleScale` (AiParams, default 12). File: `Server/Ai/AiState.cs`
2. **Body roll in corners** – roll from lateral g (speed × yaw rate), on top of the spline camber.
   Setting: `RollDegPerG` (default 2.5), max 6° (constant). File: `AiState.cs`
3. **Body pitch** – nose dips when braking, lifts when accelerating, on top of the spline slope.
   Setting: `PitchDegPerG` (default 2.5), max 3° (constant). File: `AiState.cs`
4. **Drift zones** – part of the AI cars drift through chosen corners (extra yaw, counter-steer,
   rear wheel spin / smoke). All settings in `<cfg>/drift_zones.yml` (no file = off). Min corner radius
   and min speed so junction turns never drift.
   Files: `Server/Ai/driftzones.cs` (new), `AiState.cs`, `AiModule.cs`

## AI traffic behaviour

5. **Cornering speed** – new `MaxCorneringSpeedSquared(radius)`: grip depends on the corner radius,
   linear between a tight and a wide value. Cars slow down a lot in junction turns / tight corners but
   stay fast in wide sweepers. `CorneringSpeedFactor` is no longer used.
   Settings (AiParams): `CornerTightLatAccel` 3.0, `CornerTightRadiusMeters` 15,
   `CornerWideLatAccel` 9.0, `CornerWideRadiusMeters` 200.
   Files: `AiState.cs`, `Server/Configuration/ACExtraConfiguration.cs`
6. **Traffic rules** – AI stops at stop signs and gives way at junctions.
   Loaded from `<cfg>/traffic_rules.yml` (no file = off).
   Files: `Server/Ai/trafficrules.cs` (new), `AiState.cs`, `AiModule.cs`
7. **Random AI car pick** – AI car picked at random over all slots, so the `entry_list.ini` order no
   longer matters. File: `Server/Ai/AiBehavior.cs`
8. **Separate spawn distances** – own min/max spawn distance for cars in front and oncoming cars.
   Settings: `MinOncomingSpawnDistancePoints`, `MaxOncomingSpawnDistancePoints`.
   Files: `AiBehavior.cs`, `ACExtraConfiguration.cs`, `ACServerConfigurationValidator.cs`
9. **AI name prefix** – setting `NamePrefix`.

## Server / build

10. **Kunos lobby re-registration** – re-registers in the lobby on session change / restart only
    (no startup retries, no re-register after failed pings, to avoid flooding the lobby).
    File: `Server/KunosLobbyRegistration.cs`
11. **Quieter console** – console shows Information and up only; debug lines go to the log files.
    File: `Logging.cs`
12. **x86 build (without Steam)** – x86 `csp_xxhash3` dll + cdecl calling convention fix.
    Files: `AssettoServer.csproj`, plugin `.csproj` files, `Redist/csp_xxhash3_x86.dll` (new),
    `Vendor/CSPXxHash3/CSPXxHash3.cs`. Build with `build_win_x86.bat`.
13. **AI spline cache** – the cache key now includes the AiParams baked into the cache
    (`LaneWidthMeters`, `TwoWayTraffic`, `SmoothCamber`), so changing them rebuilds the cache
    automatically; old `*.aic` files are deleted after a new cache is generated.
    File: `Server/Ai/Splines/AiSplineLocator.cs`

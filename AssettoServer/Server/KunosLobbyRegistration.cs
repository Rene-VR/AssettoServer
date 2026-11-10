using System;
using System.Linq;
using System.Net.Http;
using System.Threading;
using System.Threading.Tasks;
using System.Web;
using AssettoServer.Server.Configuration;
using AssettoServer.Shared.Services;
using Microsoft.Extensions.Hosting;
using Polly;
using Serilog;

namespace AssettoServer.Server;

public class KunosLobbyRegistration : CriticalBackgroundService
{
    private readonly ACServerConfiguration _configuration;
    private readonly SessionManager _sessionManager;
    private readonly EntryCarManager _entryCarManager;
    private readonly HttpClient _httpClient;

    // Re-register on session change/restart (Seacrest patch)
    private const int MinReRegisterSeconds = 60;
    private readonly SemaphoreSlim _registerLock = new(1, 1);
    private volatile bool _initialRegistrationDone;
    private DateTime _lastRegistration = DateTime.MinValue;
    private CancellationToken _stoppingToken;

    public KunosLobbyRegistration(ACServerConfiguration configuration, SessionManager sessionManager, EntryCarManager entryCarManager, HttpClient httpClient, IHostApplicationLifetime applicationLifetime) : base(applicationLifetime)
    {
        _configuration = configuration;
        _sessionManager = sessionManager;
        _entryCarManager = entryCarManager;
        _httpClient = httpClient;
        _sessionManager.SessionChanged += OnSessionChanged;
    }

    private void OnSessionChanged(SessionManager sender, SessionChangedEventArgs args)
    {
        if (!_configuration.Server.RegisterToLobby || !_initialRegistrationDone)
            return;

        _ = Task.Run(async () =>
        {
            if (!await _registerLock.WaitAsync(0))
                return; // a registration is already running

            try
            {
                if ((DateTime.UtcNow - _lastRegistration).TotalSeconds < MinReRegisterSeconds)
                {
                    Log.Information("Session changed, lobby re-registration skipped (last one was less than {Seconds} s ago)", MinReRegisterSeconds);
                    return;
                }

                Log.Information("Session changed, re-registering to lobby...");
                await RegisterToLobbyAsync(_stoppingToken);
                Log.Information("Lobby re-registration successful");
            }
            catch (TaskCanceledException) { }
            catch (Exception ex)
            {
                Log.Error(ex, "Error during Kunos lobby re-registration");
            }
            finally
            {
                _registerLock.Release();
            }
        });
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        if (!_configuration.Server.RegisterToLobby)
            return;

        _stoppingToken = stoppingToken;
        
        try
        {
            await RegisterToLobbyAsync(stoppingToken);
            Log.Information("Lobby registration successful");
            _initialRegistrationDone = true;
        }
        catch (TaskCanceledException) { }
        catch (Exception ex)
        {
            Log.Error(ex, "Error during Kunos lobby registration");
            return;
        }

        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await Task.Delay(TimeSpan.FromMinutes(1), stoppingToken);
                await Policy
                    .Handle<KunosLobbyException>()
                    .Or<HttpRequestException>()
                    .WaitAndRetryAsync(3, attempt => TimeSpan.FromSeconds(attempt * 10))
                    .ExecuteAsync(PingLobbyAsync, stoppingToken);
            }
            catch (TaskCanceledException) { }
            catch (Exception ex)
            {
                Log.Error(ex, "Error during Kunos lobby update");
            }
        }
    }

    private async Task RegisterToLobbyAsync(CancellationToken token)
    {
        var cfg = _configuration.Server;
        var builder = new UriBuilder("http://93.57.10.21/lobby.ashx/register");
        var queryParams = HttpUtility.ParseQueryString(builder.Query);

        string cars = string.Join(',', _entryCarManager.EntryCars.Select(c => c.Model).Distinct());
        
        // Truncate cars list, Lobby will return 404 when the URL is too long
        const int maxLen = 1200;
        if (cars.Length > maxLen)
        {
            cars = cars[..maxLen];
            int last = cars.LastIndexOf(',');
            cars = cars[..last];
        }
        
        queryParams["name"] = cfg.Name + (_configuration.Extra.EnableServerDetails ? $" ℹ{_configuration.Server.HttpPort}" : "");
        queryParams["port"] = cfg.UdpPort.ToString();
        queryParams["tcp_port"] = cfg.TcpPort.ToString();
        queryParams["max_clients"] = cfg.MaxClients.ToString();
        queryParams["track"] = _configuration.FullTrackName;
        queryParams["cars"] = cars;
        queryParams["timeofday"] = ((int)cfg.SunAngle).ToString();
        queryParams["sessions"] = string.Join(',', _configuration.Sessions.Select(s => (int)s.Type));
        queryParams["durations"] = string.Join(',', _configuration.Sessions.Select(s => s.IsTimedRace ? s.Time * 60 : s.Laps));
        queryParams["password"] = string.IsNullOrEmpty(cfg.Password) ? "0" : "1";
        queryParams["version"] = "202";
        queryParams["pickup"] = "1";
        queryParams["autoclutch"] = cfg.AutoClutchAllowed ? "1" : "0";
        queryParams["abs"] = cfg.ABSAllowed.ToString();
        queryParams["tc"] = cfg.TractionControlAllowed.ToString();
        queryParams["stability"] = cfg.StabilityAllowed ? "1" : "0";
        queryParams["legal_tyres"] = cfg.LegalTyres;
        queryParams["fixed_setup"] = "0";
        queryParams["timed"] = "0";
        queryParams["extra"] = cfg.HasExtraLap ? "1" : "0";
        queryParams["pit"] = "0";
        queryParams["inverted"] = cfg.InvertedGridPositions.ToString();
        builder.Query = queryParams.ToString();

        Log.Information("Registering server to lobby...");
        HttpResponseMessage response = await _httpClient.GetAsync(builder.ToString(), token);
        string body = await response.Content.ReadAsStringAsync(token);

        if (!body.StartsWith("OK"))
        {
            throw new KunosLobbyException(body);
        }

        _lastRegistration = DateTime.UtcNow;
    }

    private async Task PingLobbyAsync(CancellationToken token)
    {
        var builder = new UriBuilder("http://93.57.10.21/lobby.ashx/ping");
        var queryParams = HttpUtility.ParseQueryString(builder.Query);
        
        queryParams["session"] = ((int)_sessionManager.CurrentSession.Configuration.Type).ToString();
        queryParams["timeleft"] = (_sessionManager.CurrentSession.TimeLeftMilliseconds / 1000).ToString();
        queryParams["port"] = _configuration.Server.UdpPort.ToString();
        queryParams["clients"] = _entryCarManager.ConnectedCars.Count.ToString();
        queryParams["track"] = _configuration.FullTrackName;
        queryParams["pickup"] = "1";
        builder.Query = queryParams.ToString();
        
        HttpResponseMessage response = await _httpClient.GetAsync(builder.ToString(), token);
        string body = await response.Content.ReadAsStringAsync(token);

        if (!body.StartsWith("OK"))
        {
            if (body is "ERROR - RESTART YOUR SERVER TO REGISTER WITH THE LOBBY" 
                or "ERROR,SERVER NOT REGISTERED WITH LOBBY - PLEASE RESTART")
            {
                await RegisterToLobbyAsync(token);
            }
            else
            {
                throw new KunosLobbyException(body);
            }
        }
    }
}

public class KunosLobbyException : Exception
{
    public KunosLobbyException()
    {
    }

    public KunosLobbyException(string message)
        : base(message)
    {
    }

    public KunosLobbyException(string message, Exception inner)
        : base(message, inner)
    {
    }
}

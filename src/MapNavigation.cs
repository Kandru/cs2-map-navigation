using CounterStrikeSharp.API;
using CounterStrikeSharp.API.Core;
using CounterStrikeSharp.API.Modules.Utils;

namespace MapNavigation
{
    public partial class MapNavigation : BasePlugin
    {
        public override string ModuleName => "CS2 MapNavigation";
        public override string ModuleAuthor => "Kalle <kalle@kandru.de>";

        private List<Vector> _playerPositions = [];
        private bool _isDuringRound = false;
        private bool _missingPaths = false;
        private string _currentMapName = string.Empty;

        public override void Load(bool hotReload)
        {
            RegisterEventHandler<EventRoundStart>(OnRoundStart);
            RegisterEventHandler<EventRoundEnd>(OnRoundEnd);
            RegisterListener<Listeners.OnMapStart>(OnMapStart);
            RegisterListener<Listeners.OnMapEnd>(OnMapEnd);
            if (hotReload)
            {
                _isDuringRound = true;
                _currentMapName = Server.MapName.ToLower();
                CreateOrLoadMapConfig();
                GetSpawnpoints();
                GetBombspots();
                GetHostages();
                HighlightBombspots();
                HighlightBombpaths();
                RegisterPlayerPositions();
            }
        }

        public override void Unload(bool hotReload)
        {
            DeregisterEventHandler<EventRoundStart>(OnRoundStart);
            DeregisterEventHandler<EventRoundEnd>(OnRoundEnd);
            RemoveListener<Listeners.OnMapStart>(OnMapStart);
            RemoveListener<Listeners.OnMapEnd>(OnMapEnd);
            RemoveListener<Listeners.OnTick>(OnTick);
        }

        private void RegisterPlayerPositions()
        {
            // check if bombspot paths are missing
            var bombSpots = Utilities.FindAllEntitiesByDesignerName<CBombTarget>("func_bomb_target");
            foreach (CBombTarget entry in bombSpots)
            {
                if (!entry.IsValid
                    || entry.AbsOrigin == null) continue;
                if (!entry.IsBombSiteB && _currentMapConfig.PathCTToABombspot.Count == 0
                    || entry.IsBombSiteB && _currentMapConfig.PathCTToBBombspot.Count == 0)
                {
                    DebugPrint($"CT Bombspot path(s) missing. Enabling pathfinding.");
                    _missingPaths = true;
                }
                if (entry.IsBombSiteB && _currentMapConfig.PathTToABombspot.Count == 0
                    || entry.IsBombSiteB && _currentMapConfig.PathTToBBombspot.Count == 0)
                {
                    DebugPrint($"T Bombspot path(s) missing. Enabling pathfinding.");
                    _missingPaths = true;
                }
            }
            // start listener for player positions if paths are missing
            if (_missingPaths)
            {
                DebugPrint($"Starting to record player positions.");
                RegisterListener<Listeners.OnTick>(OnTick);
            }
        }

        private HookResult OnRoundStart(EventRoundStart @event, GameEventInfo info)
        {
            _isDuringRound = true;
            GetSpawnpoints();
            GetBombspots();
            GetHostages();
            HighlightBombspots();
            HighlightBombpaths();
            return HookResult.Continue;
        }

        private HookResult OnRoundEnd(EventRoundEnd @event, GameEventInfo info)
        {
            _isDuringRound = false;
            if (_missingPaths) InitiateBombspotPathfinding();
            return HookResult.Continue;
        }

        private void OnMapStart(string mapName)
        {
            DebugPrint("OnMapStart " + mapName);
            _isDuringRound = false;
            _missingPaths = false;
            _currentMapName = mapName.ToLower();
            CreateOrLoadMapConfig();
            // delay the registration of player positions to avoid issues with map loading
            AddTimer(5f, () =>
            {
                RegisterPlayerPositions();
            });
        }

        private void OnMapEnd()
        {
            DebugPrint("OnMapEnd ");
            RemoveListener<Listeners.OnTick>(OnTick);
            SaveMapConfig();
            _isDuringRound = false;
            _missingPaths = false;
            _playerPositions.Clear();
        }

        private void OnTick()
        {
            if (!_isDuringRound) return;
            foreach (CCSPlayerController entry in Utilities.GetPlayers()
                .Where(p => p.IsValid && !p.IsHLTV && p.PawnIsAlive
                    // pawn exists
                    && p.PlayerPawn != null && p.PlayerPawn.IsValid && p.PlayerPawn.Value != null && p.PlayerPawn.Value.AbsOrigin != null
                    // player is on the ground
                    && p.PlayerPawn.Value!.GroundEntity.Value != null))
            {
                Vector newPos = new(
                    entry.PlayerPawn!.Value!.AbsOrigin!.X,
                    entry.PlayerPawn!.Value!.AbsOrigin!.Y,
                    entry.PlayerPawn!.Value!.AbsOrigin!.Z
                );
                float middleDistance = (Config.MinPathfindingDistance + Config.MaxPathfindingDistance) / 2.0f;
                if (_playerPositions.Count == 0 || _playerPositions.All(pos => GetVectorDistance(newPos, pos) > middleDistance))
                {
                    _playerPositions.Add(newPos);
                    DebugPrint($"Player {entry.PlayerName} position: {newPos}");
                }
            }
        }
    }
}

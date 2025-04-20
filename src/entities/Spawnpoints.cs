using CounterStrikeSharp.API;
using CounterStrikeSharp.API.Core;
using CounterStrikeSharp.API.Modules.Utils;

namespace MapNavigation
{
    public partial class MapNavigation
    {
        private readonly List<SpawnPoint> _TSpawnpoints = [];
        private readonly List<SpawnPoint> _CTSpawnpoints = [];

        private void GetSpawnpoints()
        {
            DebugPrint("GetSpawnpoints");
            _TSpawnpoints.Clear();
            _CTSpawnpoints.Clear();
            var tSpawns = Utilities.FindAllEntitiesByDesignerName<SpawnPoint>("info_player_terrorist");
            foreach (SpawnPoint entry in tSpawns)
            {
                if (!entry.IsValid) continue;
                _TSpawnpoints.Add(entry);
            }
            var ctSpawns = Utilities.FindAllEntitiesByDesignerName<SpawnPoint>("info_player_counterterrorist");
            foreach (SpawnPoint entry in ctSpawns)
            {
                if (!entry.IsValid) continue;
                _CTSpawnpoints.Add(entry);
            }
            DebugPrint($"Amount T-Spawns: {_TSpawnpoints.Count}, Amount CT-Spawns: {_CTSpawnpoints.Count}");
        }

        private Vector? GetSpawnPointsCenter(List<SpawnPoint> spawnpoints)
        {
            if (spawnpoints.Count == 0) return null;
            float sumX = 0, sumY = 0, sumZ = 0;
            int totalPoints = 0;
            foreach (var spawnpoint in spawnpoints)
            {
                if (!spawnpoint.IsValid
                    || spawnpoint.AbsOrigin == null) continue;

                sumX += spawnpoint.AbsOrigin.X;
                sumY += spawnpoint.AbsOrigin.Y;
                sumZ += spawnpoint.AbsOrigin.Z;
                totalPoints++;
            }
            // Calculate average (centroid)
            if (totalPoints > 0)
            {
                DebugPrint($"Spawn point center: {new Vector(sumX / totalPoints, sumY / totalPoints, sumZ / totalPoints)}");
                return new Vector(
                    sumX / totalPoints,
                    sumY / totalPoints,
                    sumZ / totalPoints
                );
            }
            // Return origin if no spawn points found
            DebugPrint("Warning: No spawn points found when calculating center");
            return null;
        }
    }
}
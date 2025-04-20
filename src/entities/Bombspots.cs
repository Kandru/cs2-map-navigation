using CounterStrikeSharp.API;
using CounterStrikeSharp.API.Core;
using CounterStrikeSharp.API.Modules.Utils;
using System.Drawing;

namespace MapNavigation
{
    public partial class MapNavigation
    {
        private readonly Dictionary<string, Dictionary<string, Vector>> _bombspots = [];

        private void GetBombspots()
        {
            DebugPrint("GetBombspots");
            _bombspots.Clear();
            var bombSpots = Utilities.FindAllEntitiesByDesignerName<CBombTarget>("func_bomb_target");
            foreach (CBombTarget entry in bombSpots)
            {
                if (!entry.IsValid
                    || entry.AbsOrigin == null) continue;
                _bombspots.Add(entry.IsBombSiteB ? "B" : "A", new Dictionary<string, Vector> {
                    { "origin", new Vector(entry.AbsOrigin.X, entry.AbsOrigin.Y, entry.AbsOrigin.Z) },
                    { "mins", new Vector(entry.Collision.Mins.X, entry.Collision.Mins.Y, entry.Collision.Mins.Z) },
                    { "maxs", new Vector(entry.Collision.Maxs.X, entry.Collision.Maxs.Y, entry.Collision.Maxs.Z) }
                });
            }
            DebugPrint($"Amount Bomb spots: {_bombspots.Count}");
        }

        private void HighlightBombspots()
        {
            foreach (var kvp in _bombspots)
            {
                DebugPrint($"Bomb spot: {kvp.Key}");
                Vector mins = kvp.Value["mins"];
                Vector maxs = kvp.Value["maxs"];

                // Draw the outline of the bomb spot
                Color outlineColor;
                try
                {
                    // Try to parse the color from Config.Bombspots.OutlineColor
                    string[] colorParts = Config.Bombspots.OutlineColor.Split(',');
                    if (colorParts.Length >= 3 &&
                        byte.TryParse(colorParts[0].Trim(), out byte r) &&
                        byte.TryParse(colorParts[1].Trim(), out byte g) &&
                        byte.TryParse(colorParts[2].Trim(), out byte b))
                    {
                        outlineColor = Color.FromArgb(255, r, g, b);
                    }
                    else
                    {
                        outlineColor = Color.FromArgb(255, 255, 255, 255);
                    }
                }
                catch
                {
                    outlineColor = Color.FromArgb(255, 255, 255, 255);
                }

                DrawBombSpotOutline(
                    kvp.Value["origin"],
                    mins,
                    maxs,
                    outlineColor,
                    beamWidth: Config.Bombspots.OutlineBeamWidth,
                    heightOffset: Config.Bombspots.OutlineHeightOffset,
                    duration: Config.Bombspots.OutlineDuration);
            }
        }

        private void DrawBombSpotOutline(Vector origin, Vector mins, Vector maxs, Color color, float beamWidth = 1.5f, float heightOffset = 0f, float duration = 0f)
        {
            if (!Config.Bombspots.OutlineEnabled || !_isDuringRound) return;
            if (heightOffset > Config.Bombspots.OutlineMaxHeightOffset)
                heightOffset = Config.Bombspots.OutlineHeightOffset;
            // Calculate the corners at the fixed height
            Vector corner1 = new(origin.X + mins.X, origin.Y + mins.Y, origin.Z + heightOffset);
            Vector corner2 = new(origin.X + maxs.X, origin.Y + mins.Y, origin.Z + heightOffset);
            Vector corner3 = new(origin.X + maxs.X, origin.Y + maxs.Y, origin.Z + heightOffset);
            Vector corner4 = new(origin.X + mins.X, origin.Y + maxs.Y, origin.Z + heightOffset);

            // Draw just a single rectangle outline at the fixed heights
            CreateBeam(corner1, corner2, color, beamWidth, duration);
            CreateBeam(corner2, corner3, color, beamWidth, duration);
            CreateBeam(corner3, corner4, color, beamWidth, duration);
            CreateBeam(corner4, corner1, color, beamWidth, duration);
            heightOffset += Config.Bombspots.OutlineHeightInterval;
            if (duration > 0f)
                AddTimer(duration + Config.Bombspots.OutlineDelay, () => DrawBombSpotOutline(origin, mins, maxs, color, beamWidth, heightOffset, duration));
        }

        private void HighlightBombpaths()
        {
            if (!Config.Bombspots.PathfindingEnabled || !_isDuringRound) return;
            // TODO: config parameters
            Color CT_ASpotColor = Color.FromArgb(255, 0, 119, 255);
            Color CT_BSpotColor = Color.FromArgb(255, 0, 154, 255);
            Color T_ASpotColor = Color.FromArgb(255, 255, 179, 0);
            Color T_BSpotColor = Color.FromArgb(255, 255, 128, 0);
            float duration = 0f;
            if (Config.Bombspots.PathfindingVisualizationMode == "always") duration = 0f;
            // Highlight bomb spots
            if (_currentMapConfig.PathCTToABombspot.Count > 0)
                Server.NextFrame(() => HighlightBombpath(_currentMapConfig.PathCTToABombspot.Select(sv => sv.ToVector()).ToList(), CT_ASpotColor, duration));
            if (_currentMapConfig.PathCTToBBombspot.Count > 0)
                Server.NextFrame(() => HighlightBombpath(_currentMapConfig.PathCTToBBombspot.Select(sv => sv.ToVector()).ToList(), CT_BSpotColor, duration));
            if (_currentMapConfig.PathTToABombspot.Count > 0)
                Server.NextFrame(() => HighlightBombpath(_currentMapConfig.PathTToABombspot.Select(sv => sv.ToVector()).ToList(), T_ASpotColor, duration));
            if (_currentMapConfig.PathTToBBombspot.Count > 0)
                Server.NextFrame(() => HighlightBombpath(_currentMapConfig.PathTToBBombspot.Select(sv => sv.ToVector()).ToList(), T_BSpotColor, duration));
        }

        private void HighlightBombpath(List<Vector> path, Color color, float duration = 0f)
        {
            if (path.Count == 0) return;
            Vector? prevPoint = null;
            foreach (Vector point in path)
            {
                if (prevPoint != null)
                    CreateBeam(prevPoint, point, color, Config.Bombspots.PathfindingBeamWidth, duration);
                prevPoint = point;
            }
        }

        private void InitiateBombspotPathfinding()
        {
            // get spawn center
            Vector? ctSpawnCenter = GetSpawnPointsCenter(_CTSpawnpoints);
            Vector? tSpawnCenter = GetSpawnPointsCenter(_TSpawnpoints);
            // Run pathfinding in a separate task to avoid blocking the main thread
            Server.NextFrameAsync(() => BombspotPathfinding(ctSpawnCenter, tSpawnCenter));
        }

        private void BombspotPathfinding(Vector? ctSpawnCenter, Vector? tSpawnCenter)
        {
            // initialize pathfinder
            var pathfinder = new AStarPathfinder
            {
                // basic configuration
                MaxStartDistance = Config.Bombspots.PathfindingStartDistance,
                MaxGoalDistance = Config.Bombspots.PathfindingGoalDistance,
                AddedHeight = Config.Bombspots.PathfindingAddedHeight,
                MaxStepDistance = Config.MaxPathfindingDistance,
                MinStepDistance = Config.MinPathfindingDistance,
                HeuristicWeight = Config.Bombspots.PathfindingHeuristicWeight,
                MaxIterations = Config.Bombspots.PathfindingMaxIterations,
                // optimization parameters
                StraightLineThreshold = Config.Bombspots.PathfindingStraightLineThreshold,
                CornerRoundingFactor = Config.Bombspots.PathfindingCornerRoundingFactor,
                MaxPointsPerCurve = Config.Bombspots.PathfindingMaxPointsPerCurve,
                OptimizedDistance = Config.Bombspots.PathfindingOptimizedDistance,
                HeightChangeReductionFactor = Config.Bombspots.PathfindingHeightChangeReductionFactor,
                HeightChangeThreshold = Config.Bombspots.PathfindingHeightChangeThreshold,
                // debug mode
                Debug = Config.Bombspots.PathfindingDebug,
            };
            foreach (var kvp in _bombspots)
            {
                if (ctSpawnCenter != null)
                {
                    // Find a path
                    List<Vector> path = pathfinder.FindPath(
                        ctSpawnCenter,
                        kvp.Value["origin"],
                        _playerPositions,
                        optimizePath: Config.Bombspots.PathfindingOptimizationEnabled
                    );
                    // Remove first and last points
                    path = [.. path.Skip(Config.RemoveFirstPathfindingPoints).Take(path.Count - (Config.RemoveLastPathfindingPoints * 2))];
                    // save path
                    if (kvp.Key == "A")
                        _currentMapConfig.PathCTToABombspot = path.Select(v => new SerializableVector(v)).ToList();
                    else
                        _currentMapConfig.PathCTToBBombspot = path.Select(v => new SerializableVector(v)).ToList();
                }
                if (tSpawnCenter != null)
                {
                    // Find a path
                    List<Vector> path = pathfinder.FindPath(
                        tSpawnCenter,
                        kvp.Value["origin"],
                        _playerPositions,
                        optimizePath: Config.Bombspots.PathfindingOptimizationEnabled
                    );
                    // Remove first and last points
                    path = [.. path.Skip(Config.RemoveFirstPathfindingPoints).Take(path.Count - (Config.RemoveLastPathfindingPoints * 2))];
                    // save path
                    if (kvp.Key == "A")
                        _currentMapConfig.PathTToABombspot = path.Select(v => new SerializableVector(v)).ToList();
                    else
                        _currentMapConfig.PathTToBBombspot = path.Select(v => new SerializableVector(v)).ToList();
                }
            }
        }
    }
}
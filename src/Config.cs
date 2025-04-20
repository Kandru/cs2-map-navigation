using CounterStrikeSharp.API.Core;
using CounterStrikeSharp.API.Modules.Extensions;
using CounterStrikeSharp.API.Modules.Utils;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace MapNavigation
{
    // Serializable vector class to replace the CounterStrikeSharp Vector
    public class SerializableVector
    {
        [JsonPropertyName("x")] public float X { get; set; }
        [JsonPropertyName("y")] public float Y { get; set; }
        [JsonPropertyName("z")] public float Z { get; set; }

        public SerializableVector() { }

        public SerializableVector(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public SerializableVector(Vector vector)
        {
            X = vector.X;
            Y = vector.Y;
            Z = vector.Z;
        }

        public Vector ToVector()
        {
            return new Vector(X, Y, Z);
        }
    }

    public class MapConfig
    {
        // path from T-Spawn to A-Bombspot
        [JsonPropertyName("path_T_A")] public List<SerializableVector> PathTToABombspot { get; set; } = [];
        // path from T-Spawn to B-Bombspot
        [JsonPropertyName("path_T_B")] public List<SerializableVector> PathTToBBombspot { get; set; } = [];
        // path from CT-Spawn to A-Bombspot
        [JsonPropertyName("path_CT_A")] public List<SerializableVector> PathCTToABombspot { get; set; } = [];
        // path from CT-Spawn to B-Bombspot
        [JsonPropertyName("path_CT_B")] public List<SerializableVector> PathCTToBBombspot { get; set; } = [];
    }

    public class BomspotsConfig
    {
        //// outline parameters
        // outline enabled
        [JsonPropertyName("outline_enabled")] public bool OutlineEnabled { get; set; } = true;
        // outline color
        [JsonPropertyName("outline_color")] public string OutlineColor { get; set; } = "255,0,0";
        // outline duration
        [JsonPropertyName("outline_duration")] public float OutlineDuration { get; set; } = 2f;
        // outline delay
        [JsonPropertyName("outline_delay")] public float OutlineDelay { get; set; } = 2f;
        // outline beam width
        [JsonPropertyName("outline_beam_width")] public float OutlineBeamWidth { get; set; } = 1.0f;
        // outline height offset
        [JsonPropertyName("outline_height_offset")] public float OutlineHeightOffset { get; set; } = 0f;
        // max outline height offset
        [JsonPropertyName("outline_max_height_offset")] public float OutlineMaxHeightOffset { get; set; } = 0f;
        // outline height interval
        [JsonPropertyName("outline_height_interval")] public float OutlineHeightInterval { get; set; } = 10f;
        //// pathfinding parameters
        // pathfinding enabled
        [JsonPropertyName("pathfinding_enabled")] public bool PathfindingEnabled { get; set; } = true;
        // pathfinding start distance
        [JsonPropertyName("pathfinding_start_distance")] public float PathfindingStartDistance { get; set; } = 100f;
        // pathfinding goal distance
        [JsonPropertyName("pathfinding_goal_distance")] public float PathfindingGoalDistance { get; set; } = 100f;
        // pathfinding added height
        [JsonPropertyName("pathfinding_added_height")] public float PathfindingAddedHeight { get; set; } = 5f;
        // pathfinding heuristic weight
        [JsonPropertyName("pathfinding_heuristic_weight")] public float PathfindingHeuristicWeight { get; set; } = 1f;
        // pathfinding max iterations
        [JsonPropertyName("pathfinding_max_iterations")] public int PathfindingMaxIterations { get; set; } = 100000;
        //// pathfinding optimization parameters
        // pathfinding is optimization enabled
        [JsonPropertyName("pathfinding_optimization_enabled")] public bool PathfindingOptimizationEnabled { get; set; } = true;
        // pathfinding How closely points should align to be considered a straight line
        [JsonPropertyName("pathfinding_straight_line_threshold")] public float PathfindingStraightLineThreshold { get; set; } = 0.95f;
        // pathfinding Factor for how much to round corners
        [JsonPropertyName("pathfinding_corner_rounding_factor")] public float PathfindingCornerRoundingFactor { get; set; } = 1f;
        // pathfinding Maximum number of points to add for a curve
        [JsonPropertyName("pathfinding_max_points_per_curve")] public int PathfindingMaxPointsPerCurve { get; set; } = 12;
        // pathfinding Minimum distance between points after optimization
        [JsonPropertyName("pathfinding_optimized_distance")] public float PathfindingOptimizedDistance { get; set; } = 3f;
        // pathfinding How much to reduce corner rounding when height changes
        [JsonPropertyName("pathfinding_height_change_reduction_factor")] public float PathfindingHeightChangeReductionFactor { get; set; } = 0.7f;
        // pathfinding Z difference to consider as significant height change
        [JsonPropertyName("pathfinding_height_change_threshold")] public float PathfindingHeightChangeThreshold { get; set; } = 0.8f;
        // pathfinding debug
        [JsonPropertyName("pathfinding_debug")] public bool PathfindingDebug { get; set; } = false;
        // pathfinding mode
        [JsonPropertyName("pathfinding_visualization_mode")] public string PathfindingVisualizationMode { get; set; } = "always";
        // pathfinding beam width
        [JsonPropertyName("pathfinding_beam_width")] public float PathfindingBeamWidth { get; set; } = 1.0f;
    }

    public class PluginConfig : BasePluginConfig
    {
        // disabled
        [JsonPropertyName("enabled")] public bool Enabled { get; set; } = true;
        // debug prints
        [JsonPropertyName("debug")] public bool Debug { get; set; } = false;
        // min pathfinding distance
        [JsonPropertyName("min_pathfinding_distance")] public float MinPathfindingDistance { get; set; } = 19f;
        // max pathfinding distance
        [JsonPropertyName("max_pathfinding_distance")] public float MaxPathfindingDistance { get; set; } = 51f;
        // remove first x pathfinding points
        [JsonPropertyName("remove_first_pathfinding_points")] public int RemoveFirstPathfindingPoints { get; set; } = 1;
        // remove last x pathfinding points
        [JsonPropertyName("remove_last_pathfinding_points")] public int RemoveLastPathfindingPoints { get; set; } = 2;
        // bomb spots
        [JsonPropertyName("bombspots")] public BomspotsConfig Bombspots { get; set; } = new();

    }

    public partial class MapNavigation : BasePlugin, IPluginConfig<PluginConfig>
    {
        public required PluginConfig Config { get; set; }
        private string _mapConfigPath = "MapConfigs";
        private MapConfig _currentMapConfig = new();

        public void OnConfigParsed(PluginConfig config)
        {
            Config = config;
            // update config and write new values from plugin to config file if changed after update
            Config.Update();
            // set map config path
            _mapConfigPath = Path.Combine(Directory.GetParent(Config.GetConfigPath())?.FullName ?? String.Empty, "MapConfigs");
            // create folder if it does not exist
            if (!Directory.Exists(_mapConfigPath))
                Directory.CreateDirectory(_mapConfigPath);
            Console.WriteLine(Localizer["core.config"]);
        }

        public void CreateOrLoadMapConfig()
        {
            // check if map config file exists
            string mapConfigPath = Path.Combine(_mapConfigPath, $"{_currentMapName}.json");
            if (File.Exists(mapConfigPath))
            {
                // load map config
                string json = File.ReadAllText(mapConfigPath);
                _currentMapConfig = JsonSerializer.Deserialize<MapConfig>(json) ?? new MapConfig();
            }
            else
            {
                // create new map config file
                _currentMapConfig = new MapConfig();
                string json = JsonSerializer.Serialize(_currentMapConfig, new JsonSerializerOptions { WriteIndented = true });
                File.WriteAllText(mapConfigPath, json);
            }
        }

        public void SaveMapConfig()
        {
            // check if map config file exists
            string mapConfigPath = Path.Combine(_mapConfigPath, $"{_currentMapName}.json");

            // save map config
            string json = JsonSerializer.Serialize(_currentMapConfig, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(mapConfigPath, json);
        }
    }
}

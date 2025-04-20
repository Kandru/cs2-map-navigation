using CounterStrikeSharp.API;
using CounterStrikeSharp.API.Core;

namespace MapNavigation
{
    public partial class MapNavigation
    {
        private readonly List<CHostage> _hostages = [];

        private void GetHostages()
        {
            DebugPrint("GetHostages");
            _hostages.Clear();
            var hostageSpawns = Utilities.FindAllEntitiesByDesignerName<CHostage>("info_hostage_spawn");
            foreach (CHostage entry in hostageSpawns)
            {
                if (!entry.IsValid) continue;
                _hostages.Add(entry);
            }
            DebugPrint($"Amount Hostages: {_hostages.Count}");
        }
    }
}
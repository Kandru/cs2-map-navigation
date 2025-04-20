using CounterStrikeSharp.API.Core;
using CounterStrikeSharp.API.Core.Attributes.Registration;
using CounterStrikeSharp.API.Modules.Commands;
using CounterStrikeSharp.API.Modules.Extensions;

namespace MapNavigation
{
    public partial class MapNavigation
    {
        [ConsoleCommand("test", "MapNavigation admin commands")]
        [CommandHelper(whoCanExecute: CommandUsage.CLIENT_AND_SERVER, minArgs: 0, usage: "<command>")]
        public void test(CCSPlayerController player, CommandInfo command)
        {
            CNavMesh navMesh = new CNavMesh(NavMesh.GetNavMeshAddress());
            List<CNavArea> navAreas = GetAllNavAreas(navMesh);
            foreach (CNavArea area in navAreas)
            {
                Console.WriteLine($"Area ID: {area.ID}");
            }
        }

        private List<CNavArea> GetAllNavAreas(CNavMesh navMesh)
        {
            List<CNavArea> navAreas = new List<CNavArea>();
            for (int i = 0; i < navMesh.Count; i++)
                navAreas.Add(navMesh[i]);
            return navAreas;
        }

        [ConsoleCommand("mapnavigation", "MapNavigation admin commands")]
        [CommandHelper(whoCanExecute: CommandUsage.SERVER_ONLY, minArgs: 1, usage: "<command>")]
        public void CommandMapVote(CCSPlayerController player, CommandInfo command)
        {
            string subCommand = command.GetArg(1);
            switch (subCommand.ToLower())
            {
                case "reload":
                    Config.Reload();
                    command.ReplyToCommand(Localizer["admin.reload"]);
                    break;
                default:
                    command.ReplyToCommand(Localizer["admin.unknown_command"].Value
                        .Replace("{command}", subCommand));
                    break;
            }
        }
    }
}

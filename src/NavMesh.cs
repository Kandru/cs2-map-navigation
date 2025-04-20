using CounterStrikeSharp.API;
using CounterStrikeSharp.API.Modules.Memory.DynamicFunctions;
using CounterStrikeSharp.API.Modules.Utils;
using System.Collections;
using System.Runtime.InteropServices;

namespace MapNavigation
{
    public class NavMesh
    {
        // Thanks to _xstage on the CounterStrikeSharp Discord server for a more reliable solution than using an offset to get the navmesh
        public static readonly MemoryFunctionWithReturn<nint, bool> CSource2Server_IsValidNavMesh = new("48 8D 05 ? ? ? ? 48 83 38 00 0F 95 C0");

        public static readonly nint NavMeshPtrAddress = GetNavMeshPtrAddress();

        public static nint GetNavMeshPtrAddress()
        {
            nint functionAddress = Marshal.ReadIntPtr(CSource2Server_IsValidNavMesh.Handle);
            return functionAddress.Rel(3);
        }

        public static nint GetNavMeshAddress() => Marshal.ReadIntPtr(NavMeshPtrAddress);

        public static CNavMesh? GetNavMesh()
        {
            nint navMeshAddress = GetNavMeshAddress();
            if (navMeshAddress == 0)
            {
                return null;
            }

            return new(navMeshAddress);
        }
    }

    public class CNavMesh(nint pointer) : NativeObject(pointer), IReadOnlyCollection<CNavArea>
    {
        public int Count => Marshal.ReadInt32(Handle + 8);

        public CNavArea this[int index]
        {
            get
            {
                nint navAreas = Marshal.ReadIntPtr(Handle + 16);
                return new(Marshal.ReadIntPtr(navAreas + index * 8));
            }
        }

        public IEnumerator<CNavArea> GetEnumerator()
        {
            for (int i = 0; i < Count; i++)
            {
                yield return this[i];
            }
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }

    public class CNavArea(nint pointer) : NativeObject(pointer)
    {
        public Vector Center => new(Handle + 12);
        public Vector Min => new(Handle + 36);
        public Vector Max => new(Handle + 48);

        public uint ID => unchecked((uint)Marshal.ReadInt32(Handle + 84));

        public byte BlockedTeam => Marshal.ReadByte(Handle + 92);
    }

    // Thanks to nuko8964 on the CounterStrikeSharp Discord server for the suggestion
    public static class IntPtrExtension
    {
        public static nint Rel(this nint address, int offset)
        {
            int relativeOffset = Marshal.ReadInt32(address + offset);
            return address + relativeOffset + offset + sizeof(int) /* The size of the relative offset */;
        }
    }
}
using BepInEx;
using BepInEx.Logging;
using BepInEx.Unity.IL2CPP;

namespace BlazeGolf;

[BepInPlugin(Guid, "BlazeGolf", "0.1.0")]
public class Plugin : BasePlugin
{
    public const string Guid = "com.blazegolf.predictor";

    internal static ManualLogSource Logger;

    public override void Load()
    {
        Logger = Log;
        AddComponent<Predictor>();
        Log.LogInfo("BlazeGolf loaded. H = auto-aim at hole, F5 = end-point camera, F6 = PhysX/spherecast, F7 = toggle line, F8 = dump runtime values, F9 = rebuild sim world.");
    }
}

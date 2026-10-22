# BlazeGolf

A BepInEx 6 (IL2CPP) mod for **Golf With Your Friends** that predicts where your shot will go and can
solve for a shot into the hole.

- **Prediction line** – the shot is simulated in a private copy of the course inside PhysX, using the
  game's own launch force and drag rules, so the line matches the real ball to within a few centimetres
  on static geometry.
- **End-point camera** – a picture-in-picture view of where the ball will stop.
- **Auto-aim (H)** – searches aim and power for a hole-in-one, or a two-shot plan when none exists, then
  holds that aim and power. It never takes the shot; you still click.
- **Moving obstacles** – windmills, spinners and other cyclic obstacles are recorded and replayed forward
  in time; timed shots get a release countdown.

## Keys

| Key | Action |
| --- | --- |
| H | Auto-aim at the hole (again to cancel/release) |
| F5 | Toggle the end-point camera |
| F6 | Switch PhysX / sphere-cast prediction |
| F7 | Toggle the prediction line |
| F8 | Log runtime physics values |
| F9 | Rebuild the simulated course |

## Building

1. Install [BepInEx 6 IL2CPP](https://builds.bepinex.dev/projects/bepinex_be) into the game folder and
   launch the game once so it generates `BepInEx/interop`.
2. `dotnet build -c Release` in `src/BlazeGolf`. The project references the game's BepInEx folder
   (override with `-p:GameDir=...`) and copies the DLL to `BepInEx/plugins/BlazeGolf`.

`NOTES.md` has the reverse-engineering notes on the game's shot and drag logic.

Please only use it where everyone playing knows about it.

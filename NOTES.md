# BlazeGolf – RE notes

Game: Golf With Your Friends, Unity 2021.3.28f1, IL2CPP, metadata v29.
Dump: `dump/` (Il2CppDumper v6.7.46) — `dump.cs`, `il2cpp.h`, `script.json`, `DummyDll/`.
`BallMovement.txt` = trimmed BallMovement class (fields + methods).

## Core class: `BallMovement : T17PhotonBehaviour` (TypeDefIndex 1852)

Shot / force
- `m_maxForce` (FloatData) 0x170, `minForce` (FloatData) 0x178
- `m_PowerCurve` (AnimationCurve) 0x350 — charge -> force mapping
- `m_chargeAxis` 0x3A0, `m_shootButton` 0x378
- `preHitLocation` 0x140, `preHitRotation` 0x14C

Drag / damping (game-side, layered on PhysX)
- `angDragToHitBall` 0x188, `angDragToSlowBall` 0x18C, `dragToHitBall` 0x190
- `dragToSlow` 0x1BC, `anglularDragToSlow` 0x1B8, `stoppingDragTimesangDragToSlowBall` 0x1C8
- `initialDrag` 0x1A4, `m_environmentalDragToApply` 0x198, `m_sandDrag` 0x19C, `m_glueDrag` 0x1A0
- `capUntilOverVel` 0x1A8, `SecondTillDrag` 0x1B5 (+ coroutine `WaitOneSecondForDrag`) — drag kicks in ~1s after hit
- `sleepSpeedThreshold` 0xA0, `sleepAccelerationThreshold` 0xA4, `m_maxTimeToForceSleep` 0x23C

Physics components
- `m_rigidBody` 0x260, `m_sphereCollider` 0x280, `m_meshCollider` 0x278
- `physicsMaterials[]` 0xF8, `m_frictionlessMaterial` 0x108
- `m_ballShape` 0xE0, `m_ballSize` 0xE8, `BallRadius` 0x1C0

Key methods (RVA in GameAssembly.dll)
| Method | RVA |
|---|---|
| BallMovement.CheckForBallHit | 0x3B7660 |
| BallMovement.ApplyOnShotStarted | 0x3B52D0 |
| BallMovement.FixedUpdateLoop | 0x3AEE90 |
| BallMovement.UpdateAllBalls (drag model) | 0x3AC270 |
| BallMovement.UpdateActiveBall | 0x3AD7F0 |
| BallMovement.SetPhysicsMaterial | 0x3B3440 |
| BallMovement.ApplyBallShapePhysics | 0x3B5F90 |
| BallSpinning.InitialiseSpinFromHit | 0x3C0CD0 |
| BallSpinning.ApplySpinEffect | 0x3C0E50 |
| BallHandbrake.GetHandbrakeDrag | 0x60F980 |

## Other relevant classes
- `BallSpinning` — spin/curve: `m_spinProfile` curve, `m_forceComponentSensitivity`, `ApplySpinEffect(rb, groundNormal, dt)` runs per physics step
- `BallHandbrake` — extra drag while brake held (`m_brakingDrag`)
- `MouseAim` / `CameraOrbit` — aim direction comes from the camera
- `NetworkBallSync` (Photon) — owner simulates locally and streams position/velocity, so local prediction = what actually happens
- Surface/zone modifiers to account for later: sand, glue, `Booster` (m_Force), gravity zones, trolleys, `Floatation`, powerups (`PowerupStickyGlue`, `PowerupSpikes`, `PowerupRandomShape`, ...)

## Decompiled logic (IDA, GameAssembly.dll.i64 has all names applied)

### Charging — CheckForBallHit (Update, while shoot held)
```
hitForce = clamp(hitForce + chargeAxis.value * settings.sensitivity (* -1 if inverted), minForce, maxForce)
// controller: delta = deadzoned(stick, 0.3) * unscaledDeltaTime * settings.x
```
`hitForce` lives on `User.m_hitForce` (User+0xB0), not on BallMovement.

### Shot — CheckForBallHit (on shoot release)
```
rb.angularDrag = 1.0
rb.drag        = initialDrag + m_environmentalDragToApply     // initialDrag = 0.5 (OnStart)
capUntilOverVel = true for 0.5 s                              // WaitOneSecondForShooting (uses m_waitHalfSecond)

t     = (hitForce - minForce) / (maxForce - minForce)
force = hitForce * m_PowerCurve.Evaluate(t)                   // only if curve != null

camXZ = cameraPos with y = restingPosition.y
dir   = -normalize(camXZ - restingPosition)                   // horizontal, camera -> ball
rb.AddForce(dir * (force + 1), ForceMode.Force)               // called from Update!
```
ForceMode.Force from Update → applied in the next FixedUpdate as Δv = F / mass * fixedDeltaTime.
Mass, fixedDeltaTime, min/maxForce and m_PowerCurve are asset/runtime values → read them live in the mod.

### Drag model — UpdateAllBalls (every frame, dt = frame time)
```
speed = |rb.velocity|
extra = handbrakeDrag + envDrag            // envDrag = water + (sand?8) + (glue?8)
rb.drag = initialDrag(0.5) + extra
if speed > 0.1:
    if speed <= 1.0 (and isMine):
        sphereCollider.contactOffset = 0.0005
        if !capUntilOverVel:
            rb.drag        = (1 - speed)^2 * dragToSlow(30) + extra + 0.5
            rb.angularDrag = (1 - speed)^2 * anglularDragToSlow(30)
    else (speed > 1):
        rb.angularDrag = 1.0; capUntilOverVel = false; contactOffset = 0.000005
idle = accel^2 < 0.1 && speed < 0.1   (accel from velocity delta / dt; hot-seat uses 0.2 s hold)
```
- rb.maxAngularVelocity = 10000 (OnStart)
- While idle before a shot, angularDrag is set to 1000 once WaitOneSecondForDrag (0.25 s) sees speed <= 0.1

### Spin — BallSpinning.ApplySpinEffect (FixedUpdateLoop, only if spin game option on and spinSpeed != 0)
```
v = |rb.velocity|^2 > 1 ? rb.velocity : groundNormal
rb.AddForce(cross(groundNormal, v) * spinSpeed, ForceMode.Force)
spinSpeed (on hit) = spinProfile.Evaluate(..) * spinScaleY * sign(intensity); stops at spinSpeed * spinStopPercentage
```

### Defaults (.ctor)
sleepSpeed 0.1, sleepAccel 0.1, dragToHitBall 10, angDragToHit/Slow 2, dragToSlow 30, angularDragToSlow 30,
sand/glue drag 8, maxHitCount 12, stick deadzone 0.3, forceSleep 5 s.

## Runtime findings (BepInEx mod, src/BlazeGolf)
- fixedDeltaTime 0.005 (200 Hz), gravity -30, bounceThreshold 0.2
- Ball: mass 1, inertia (1,1,1) (≈200× a real sphere → slides, barely spins), radius 1 × scale 0.07,
  ContinuousSpeculative, material GolfBall bounce 0.8 dynF 0.01 Average/Average
- minForce 0, maxForce 10500; m_PowerCurve has 0 keys → launch speed is exactly (hitForce+1)·dt/mass
- Runtime overrides: dragToSlow = 0 (code default 30), anglularDragToSlow = 40
- Course ground: layer 0, MeshCollider, material Grass dynF 0.2 bounce 0.01
- Ball stays on layer 21 (IgnoreBallCollision) while moving; GetIgnoreLayerCollision gives an empty mask for it
- Interop pitfall: pass Il2CppStructArray<RaycastHit> to *NonAlloc queries — a managed array is copied and hits are lost
- Hand-written friction never matched PhysX (effective μ≈0.026 vs combined 0.105) → prediction runs in a cloned
  PhysicsScene instead (SimWorld.cs). Accuracy there: 2–3 cm end error on normal shots.

## Predictor implications
- Sim in a separate PhysicsScene with cloned colliders, ball with same mass/material/radius/contactOffset.
- Step 0: velocity = dir * (force+1) / mass * fixedDeltaTime.
- Each step: reapply the drag model above (0.5 s grace with no slow-drag), spin force if active, envDrag from zones.
- Drag is recomputed per *frame* in-game, per physics step in the sim: tiny error at very high/low FPS.

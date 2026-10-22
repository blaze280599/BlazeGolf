using System;
using System.Collections.Generic;
using System.Text;
using Il2CppInterop.Runtime.InteropTypes.Arrays;
using UnityEngine;

namespace BlazeGolf;

// Finds the local ball each frame, logs its physics setup, and draws the predicted path
// of the shot currently being charged. Physics constants mirror BallMovement (see NOTES.md).
public class Predictor : MonoBehaviour
{
    // BallMovement defaults (.ctor / OnStart)
    const float InitialDrag = 0.5f;
    const float SleepSpeed = 0.1f;
    const float SlowDragMaxSpeed = 1.0f;
    const float ShotGraceTime = 0.5f;   // capUntilOverVel coroutine (m_waitHalfSecond)
    const float MaxSimTime = 8f;      // the game force-sleeps balls after ~5 s
    const float Skin = 0.002f;
    const int MaxPoints = 4000;

    BallMovement _ball;
    bool _enabled = true;
    bool _loggedForBall;

    LineRenderer _line;
    GameObject _marker;
    readonly List<Vector3> _points = new();
    // Must be an Il2Cpp array: a managed RaycastHit[] is copied on the call and the hits are lost.
    readonly Il2CppStructArray<RaycastHit> _hitBuffer = new(16);
    int _simHits;

    public Predictor(IntPtr ptr) : base(ptr) { }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.F7))
        {
            _enabled = !_enabled;
            Plugin.Logger.LogInfo($"Prediction {(_enabled ? "on" : "off")}");
        }
        if (Input.GetKeyDown(KeyCode.F5))
        {
            _pip.Enabled = !_pip.Enabled;
            Plugin.Logger.LogInfo($"End-point camera {(_pip.Enabled ? "on" : "off")}");
        }
        if (Input.GetKeyDown(KeyCode.F6))
        {
            _usePhysX = !_usePhysX;
            Plugin.Logger.LogInfo($"Prediction mode: {(_usePhysX ? "PhysX sim world" : "spherecast")}");
        }
        if (Input.GetKeyDown(KeyCode.F9) && _ball != null)
        {
            _simBroken = false;
            _sim.Rebuild(_ball);
        }

        if (_ball == null || _ball.WasCollected || !IsLocalBall(_ball))
        {
            _ball = FindLocalBall();
            _loggedForBall = false;
        }

        if (_ball != null && (!_loggedForBall || Input.GetKeyDown(KeyCode.F8)))
        {
            DumpRuntimeValues(_ball);
            _loggedForBall = true;
        }

        TrackFrameTime();
        if (_sim.Ready) _sim.SampleMovers();
        EnsureVisuals();
        HandleAutoAim();
        bool show;
        if (_solver.Status == ShotSolver.State.Solving)
            show = ShowPath(_solver.BestPoints);
        else
        {
            if (_locked) ApplyLock();
            show = _enabled && _ball != null && TryPredict(_ball);
        }
        _line.enabled = show;
        _marker.SetActive(show);
        if (show) _pip.Show(_points[_points.Count - 1], _points[Mathf.Max(0, _points.Count - 40)]);
        else _pip.Hide();
        ObserveMovingLayer();
        TrackShot();
    }

    // ---- shot comparison -----------------------------------------------------
    // Keeps the last prediction made while charging, records the real path once the ball
    // leaves idle, and logs predicted vs actual when it comes to rest.

    readonly List<Vector3> _predicted = new();
    readonly List<Vector3> _actual = new();
    readonly List<float> _actualT = new();
    readonly List<SimWorld.Contact> _predictedContacts = new();
    string _predictedInfo;
    Vector3 _predictedV0;
    bool _tracking;
    float _trackTime;

    void TrackShot()
    {
        if (_ball == null || _ball.m_rigidBody == null) { _tracking = false; return; }
        var rb = _ball.m_rigidBody;

        if (!_tracking)
        {
            if (!_ball.IsBallIdle && _predicted.Count > 1)
            {
                _tracking = true;
                _trackTime = 0f;
                _actual.Clear();
                _actualT.Clear();
                _actual.Add(_predicted[0]);
                _actualT.Add(0f);
                _actual.Add(rb.position);
                _actualT.Add(Time.deltaTime);
                Plugin.Logger.LogInfo($"Shot taken: {_predictedInfo} predV0={_predictedV0.magnitude:0.###} actualV={rb.velocity.magnitude:0.###} actualDir={rb.velocity.normalized} predDir={_predictedV0.normalized}");
            }
            return;
        }

        _trackTime += Time.deltaTime;
        _actual.Add(rb.position);
        _actualT.Add(_trackTime + _actualT[1]);
        bool timedOut = _trackTime > 20f || _ball.m_player.m_inHole;
        if (!timedOut && (_trackTime < 0.3f || !_ball.IsBallIdle)) return;

        _tracking = false;
        if (timedOut)
            Plugin.Logger.LogInfo($"Shot tracking ended early (inHole={_ball.m_player.m_inHole}, t={_trackTime:0.0}s)");
        Vector3 predEnd = _predicted[_predicted.Count - 1], realEnd = rb.position;
        float maxDev = 0f;
        foreach (var p in _actual)
            maxDev = Mathf.Max(maxDev, DistanceToPolyline(p, _predicted));
        Plugin.Logger.LogInfo(
            $"Shot result: time={_trackTime:0.00}s start={_predicted[0]} predEnd={predEnd} realEnd={realEnd} endError={Vector3.Distance(predEnd, realEnd):0.###}m " +
            $"predLen={PathLength(_predicted):0.##}m realLen={PathLength(_actual):0.##}m maxDeviation={maxDev:0.###}m");
        if (Vector3.Distance(predEnd, realEnd) > 0.5f)
            LogDivergence(realEnd);
        _predicted.Clear();
    }

    // Explains a missed prediction: where the time-aligned paths split, every impact the
    // simulated ball had, and what colliders exist at the predicted end in each world.
    void LogDivergence(Vector3 realEnd)
    {
        float dt = Time.fixedDeltaTime;
        var sb = new StringBuilder();
        sb.AppendLine("Divergence report:");
        int window = Mathf.CeilToInt(Time.smoothDeltaTime / dt) + 2;   // tolerate a frame of offset

        for (int i = 1; i < _actual.Count; i++)
        {
            int idx = Mathf.RoundToInt(_actualT[i] / dt);
            float best = float.MaxValue; int bestIdx = -1;
            for (int k = Mathf.Max(0, idx - window); k <= Mathf.Min(_predicted.Count - 1, idx + window); k++)
            {
                float d = Vector3.Distance(_actual[i], _predicted[k]);
                if (d < best) { best = d; bestIdx = k; }
            }
            if (bestIdx < 0)
            {
                sb.AppendLine($"  prediction ended at t={(_predicted.Count - 1) * dt:0.000}s while the real ball was still moving (real t={_actualT[i]:0.000}s pos={_actual[i]})");
                break;
            }
            if (best > 0.3f)
            {
                float realSpeed = Vector3.Distance(_actual[i], _actual[i - 1]) / Mathf.Max(1e-4f, _actualT[i] - _actualT[i - 1]);
                float predSpeed = bestIdx > 0 ? Vector3.Distance(_predicted[bestIdx], _predicted[bestIdx - 1]) / dt : 0f;
                sb.AppendLine($"  paths split at t={_actualT[i]:0.000}s: real={_actual[i]} ({realSpeed:0.##} m/s) pred={_predicted[bestIdx]} ({predSpeed:0.##} m/s) gap={best:0.###}m");
                sb.AppendLine($"  real colliders there: {RealCollidersAt(_actual[i], 0.3f)}");
                sb.AppendLine($"  sim colliders there:  {_sim.SimCollidersAt(_actual[i], 0.3f)}");
                break;
            }
        }

        sb.AppendLine($"  sim impacts ({_predictedContacts.Count}):");
        foreach (var c in _predictedContacts)
            sb.AppendLine($"    t={c.Step * dt:0.000}s pos={c.Pos} dv={c.DeltaV} |dv|={c.DeltaV.magnitude:0.##} hits=[{c.Colliders}]");

        Vector3 predEnd = _predicted[_predicted.Count - 1];
        sb.AppendLine($"  at predicted end {predEnd}: real=[{RealCollidersAt(predEnd, 1f)}] sim=[{_sim.SimCollidersAt(predEnd, 1f)}]");
        sb.AppendLine($"  at real end {realEnd}: real=[{RealCollidersAt(realEnd, 0.5f)}] sim=[{_sim.SimCollidersAt(realEnd, 0.5f)}]");
        Plugin.Logger.LogInfo(sb.ToString());
    }

    readonly Il2CppReferenceArray<Collider> _realOverlap = new(32);

    // Every collider in the game scene near `pos`, flagged with what made the sim skip it.
    string RealCollidersAt(Vector3 pos, float radius)
    {
        int n = Physics.OverlapSphereNonAlloc(pos, radius, _realOverlap, ~0, QueryTriggerInteraction.Collide);
        var parts = new List<string>();
        for (int i = 0; i < n; i++)
        {
            var c = _realOverlap[i];
            if (c == null) continue;
            var rb = c.attachedRigidbody;
            string flags = (c.isTrigger ? " trigger" : "")
                         + (rb != null ? (rb.isKinematic ? " kinematic" : " DYNAMIC") : "")
                         + (c.GetComponentInParent<BallMovement>() != null ? " ball" : "");
            parts.Add($"{c.name}(L{c.gameObject.layer} {c.GetIl2CppType().Name}{flags})");
        }
        return parts.Count == 0 ? "none" : string.Join(", ", parts);
    }

    static float PathLength(List<Vector3> pts)
    {
        float len = 0f;
        for (int i = 1; i < pts.Count; i++) len += Vector3.Distance(pts[i - 1], pts[i]);
        return len;
    }

    static float DistanceToPolyline(Vector3 p, List<Vector3> line)
    {
        float best = float.MaxValue;
        for (int i = 1; i < line.Count; i++)
        {
            Vector3 a = line[i - 1], ab = line[i] - a;
            float t = ab.sqrMagnitude > 0f ? Mathf.Clamp01(Vector3.Dot(p - a, ab) / ab.sqrMagnitude) : 0f;
            best = Mathf.Min(best, Vector3.Distance(p, a + ab * t));
        }
        return best;
    }

    static bool IsLocalBall(BallMovement ball)
    {
        var user = ball.m_player;
        return user != null && user.m_bIsLocal && user.m_bIsPrimary;
    }

    static BallMovement FindLocalBall()
    {
        foreach (var ball in FindObjectsOfType<BallMovement>())
            if (IsLocalBall(ball))
                return ball;
        return null;
    }

    // ---- logging -------------------------------------------------------------

    static void DumpRuntimeValues(BallMovement ball)
    {
        var rb = ball.m_rigidBody;
        var sphere = ball.m_sphereCollider;
        var sb = new StringBuilder();
        sb.AppendLine("==== BlazeGolf runtime values ====");
        sb.AppendLine($"fixedDeltaTime={Time.fixedDeltaTime} gravity={Physics.gravity} bounceThreshold={Physics.bounceThreshold}");
        sb.AppendLine($"defaultContactOffset={Physics.defaultContactOffset} solverIterations={Physics.defaultSolverIterations}/{Physics.defaultSolverVelocityIterations} sleepThreshold={Physics.sleepThreshold}");
        if (rb != null)
        {
            sb.AppendLine($"rb.mass={rb.mass} drag={rb.drag} angularDrag={rb.angularDrag} maxAngVel={rb.maxAngularVelocity}");
            sb.AppendLine($"rb.interpolation={rb.interpolation} collisionMode={rb.collisionDetectionMode} useGravity={rb.useGravity} inertia={rb.inertiaTensor} sleepThreshold={rb.sleepThreshold}");
        }
        if (sphere != null)
        {
            sb.AppendLine($"sphere.radius={sphere.radius} lossyScale={sphere.transform.lossyScale} contactOffset={sphere.contactOffset} enabled={sphere.enabled}");
            sb.AppendLine($"sphere.material={DescribeMaterial(sphere.sharedMaterial)}");
        }
        var mesh = ball.m_meshCollider;
        if (mesh != null)
            sb.AppendLine($"meshCollider.enabled={mesh.enabled} convex={mesh.convex} material={DescribeMaterial(mesh.sharedMaterial)}");
        sb.AppendLine($"minForce={ball.minForce?.Value} maxForce={ball.m_maxForce?.Value} BallRadius={ball.BallRadius} layer={ball.gameObject.layer}");
        sb.AppendLine($"dragToSlow={ball.dragToSlow} angDragToSlow={ball.anglularDragToSlow} initialDrag={ball.initialDrag} envDrag={ball.m_environmentalDragToApply}");
        sb.AppendLine($"PowerCurve: {DescribeCurve(ball.m_PowerCurve)}");
        Plugin.Logger.LogInfo(sb.ToString());
    }

    static string DescribeMaterial(PhysicMaterial m) =>
        m == null ? "null" :
        $"{m.name} bounce={m.bounciness} dynF={m.dynamicFriction} staticF={m.staticFriction} bounceCombine={m.bounceCombine} frictionCombine={m.frictionCombine}";

    static string DescribeCurve(AnimationCurve curve)
    {
        if (curve == null) return "null";
        var sb = new StringBuilder();
        foreach (var k in curve.keys)
            sb.Append($"[t={k.time} v={k.value} in={k.inTangent} out={k.outTangent}] ");
        sb.Append($"keys={curve.length} pre={curve.preWrapMode} post={curve.postWrapMode} samples:");
        for (int i = 0; i <= 10; i++)
            sb.Append($" {i / 10f:0.0}->{curve.Evaluate(i / 10f):0.###}");
        return sb.ToString();
    }

    // ---- prediction ----------------------------------------------------------

    bool TryPredict(BallMovement ball)
    {
        var user = ball.m_player;
        var rb = ball.m_rigidBody;
        var sphere = ball.m_sphereCollider;
        if (user == null || rb == null || sphere == null || !ball.IsBallIdle)
            return false;

        float hitForce = user.m_hitForce;
        if (hitForce <= 0f)
            return false;

        var camFollow = user.PlayerCamera;
        if (camFollow == null || camFollow.hitpoint == null)
            return false;

        // CheckForBallHit: force curve + direction from camera hitpoint to resting position.
        float min = ball.minForce.Value, max = ball.m_maxForce.Value;
        float force = hitForce;
        // The serialized curve is empty in practice and the game's launch speed matches the raw force.
        if (ball.m_PowerCurve != null && ball.m_PowerCurve.length > 0 && max > min)
            force *= ball.m_PowerCurve.Evaluate((hitForce - min) / (max - min));

        Vector3 rest = ball.restingPosition;
        Vector3 cam = camFollow.hitpoint.transform.position;
        cam.y = rest.y;
        Vector3 dir = -(cam - rest).normalized;

        float dt = Time.fixedDeltaTime;
        // ForceMode.Force from Update lands in the next FixedUpdate: dv = F/m * dt.
        Vector3 vel = dir * (force + 1f) / rb.mass * dt;
        Vector3 pos = rb.position;
        float radius = sphere.radius * MaxAbs(sphere.transform.lossyScale);
        int mask = ResolveMask(ball);
        var ballMat = sphere.sharedMaterial;
        float envDrag = ball.m_environmentalDragToApply;

        string mode;
        bool fresh;
        if (_usePhysX && TryEnsureSim(ball))
        {
            fresh = StepPhysX(ball, dir * (force + 1f), pos);
            mode = $"physx({_sim.ColliderCount} cols, {_sim.LastSteps} steps, {_sim.LastSimMs:0.0}ms total, {_jobStepsPerFrame} steps/frame)";
        }
        else
        {
            Simulate(pos, vel, dt, radius, mask, rb, ballMat, envDrag);
            fresh = true;
            mode = "spherecast";
        }
        if (_points.Count < 2)
            return false;

        _line.positionCount = _points.Count;
        for (int i = 0; i < _points.Count; i++)
            _line.SetPosition(i, _points[i]);
        _marker.transform.position = _points[_points.Count - 1];
        _marker.transform.localScale = Vector3.one * radius * 2f;

        if (!fresh)
            return true;
        _predicted.Clear();
        _predicted.AddRange(_points);
        _predictedContacts.Clear();
        if (mode.StartsWith("physx")) _predictedContacts.AddRange(_doneContacts);
        _predictedV0 = vel;
        _predictedInfo = $"mode={mode} hitForce={hitForce:0.#} curvedForce={force:0.#} t={(max > min ? (hitForce - min) / (max - min) : 0f):0.###} simHits={_simHits} mask=0x{mask:X} ({_maskSource}) collideLayer={ball.m_collideLayer} ignoreLayer={ball.m_ignoreLayer} ground=[{DescribeGround(rb.position, mask)}]";
        return true;
    }

    // ---- auto-aim ------------------------------------------------------------
    // H solves for a shot into the cup, then holds that aim and power until the player
    // shoots (or presses H again). The mod never fires the shot itself.

    // The solver may take most of a frame: a fast answer matters more than frame rate for the
    // second or so it runs.
    const double SolveBudgetMs = 45.0;

    readonly ShotSolver _solver = new();
    bool _locked;
    float _lockYaw, _lockPower;
    string _status;
    float _statusUntil;

    // Second shot of a two-shot plan: re-solve from wherever the first one actually stopped.
    bool _followUp, _awaitRest;
    float _restTime;
    const float RestDelay = 0.4f;

    void HandleAutoAim()
    {
        if (_awaitRest && _ball != null)
        {
            if (_ball.m_player != null && _ball.m_player.m_inHole) _awaitRest = false;
            else if (_ball.IsBallIdle && (_restTime += Time.deltaTime) >= RestDelay)
            {
                _awaitRest = false;
                Plugin.Logger.LogInfo("Two-shot plan: ball stopped, solving shot 2");
                StartSolve();
            }
            else if (!_ball.IsBallIdle) _restTime = 0f;
        }

        if (Input.GetKeyDown(KeyCode.H))
        {
            if (_solver.Status == ShotSolver.State.Solving) { _solver.Cancel(); SetStatus("Auto-aim cancelled", 2f); }
            else if (_locked) { _locked = false; _timed = false; _followUp = false; SetStatus("Auto-aim released", 2f); }
            else if (_awaitRest) { _awaitRest = false; SetStatus("Two-shot plan cancelled", 2f); }
            else StartSolve();
        }

        if (_solver.Status == ShotSolver.State.Solving)
        {
            if (_ball == null || !_ball.IsBallIdle) { _solver.Cancel(); SetStatus("Auto-aim cancelled (ball moved)", 2f); return; }
            _simRanThisFrame = true;
            _solver.Tick(SolveBudgetMs);
            SetStatus($"Auto-aim: {_solver.Phase} ({_solver.Evaluations} shots in {_solver.Batches} batches, best {(_solver.BestHoled ? "IN" : $"{_solver.BestCost:0.00} m")})", 1f);
            if (_solver.Status == ShotSolver.State.Done)
            {
                _locked = true;
                _lockYaw = _solver.BestYaw;
                _lockPower = _solver.BestPower;
                _hasJob = false;
                _followUp = _solver.TwoShot;
                string result = _solver.BestHoled ? "holes out"
                    : _solver.TwoShot ? "no hole-in-one; shot 1 of a 2-shot plan (shot 2 is solved automatically when the ball stops)"
                    : $"no holing shot found, closest {_solver.BestCost:0.00} m";
                SetStatus($"Auto-aim locked: {result} — power {_lockPower / _ball.m_maxForce.Value * 100f:0.#}%. Click to shoot without moving the mouse. H to release.", 30f);
                Plugin.Logger.LogInfo($"Auto-aim solved in {_solver.ElapsedMs:0}ms wall ({_sim.StatMs:0}ms sim): {_solver.Evaluations} shots, {_solver.Batches} batches, " +
                    $"{_sim.StatSimulateCalls} Simulate calls ({_sim.StatMs * 1000.0 / Mathf.Max(1, _sim.StatSimulateCalls):0}us each), {_sim.StatBallSteps} ball-steps; " +
                    $"yaw={_lockYaw:0.###} power={_lockPower:0.#} holed={_solver.BestHoled} cost={_solver.BestCost:0.###} {_solver.Phase}");
                SetUpTiming();
            }
        }

        if (_locked && _ball != null && _ball.IsBallIdle)
        {
            if (_timed) UpdateCountdown();
            else if (_sim.HasMovingObstacles && _solver.BestHoled)
                SetStatus(_doneGoesIn ? ">>> GOES IN — shoot now <<<" : "Auto-aim locked — waiting for the obstacles to line up…", 1f);
        }

        if (_locked && (_ball == null || !_ball.IsBallIdle))
        {
            _locked = false;
            _timed = false;
            _awaitRest = _followUp;
            _followUp = false;
            _restTime = 0f;
            SetStatus("", 0f);
        }
    }

    void StartSolve()
    {
        if (_ball == null || !_ball.IsBallIdle) { SetStatus("Auto-aim: wait for the ball to stop", 2f); return; }
        if (!TryEnsureSim(_ball)) { SetStatus("Auto-aim: sim world unavailable", 3f); return; }
        if (!_sim.HasGroundUnder(_ball.m_rigidBody.position)) _sim.Rebuild(_ball);

        var hole = FindHole(_ball);
        if (hole == null) { SetStatus("Auto-aim: hole not found", 3f); return; }
        var cup = FindCup(hole.Value);
        Plugin.Logger.LogInfo($"Auto-aim: hole {_ball.m_CurrentHoleNumber.m_Value} at {hole.Value}, cup trigger {(cup.HasValue ? cup.Value.ToString() : "none (using radius fallback)")}");

        _hasJob = false;
        _sim.SyncColliders(force: true);
        _solver.Start(_sim, _ball, hole.Value, cup, _jobStepsPerFrame = Mathf.Max(1, Mathf.RoundToInt(_baseFrameDt / Time.fixedDeltaTime)), Time.time);
        if (_sim.HasMovingObstacles)
            Plugin.Logger.LogInfo($"Auto-aim with moving obstacles: {_sim.DescribeMovers()}");
    }

    Vector3? FindHole(BallMovement ball)
    {
        int number = ball.m_CurrentHoleNumber.m_Value;
        Vector3 ballPos = ball.m_rigidBody.position;
        Transform match = null, nearest = null;
        float nearestDist = float.MaxValue;
        foreach (var flag in FindObjectsOfType<FlagPoint>())
        {
            var t = flag.HolePosition;
            if (t == null) continue;
            if (flag.m_holeNumber == number) match = t;
            float d = (t.position - ballPos).sqrMagnitude;
            if (d < nearestDist) { nearestDist = d; nearest = t; }
        }
        if (match == null && nearest != null)
            Plugin.Logger.LogWarning($"No FlagPoint for hole {number}; using the nearest one");
        return (match ?? nearest)?.position;
    }

    // The smallest trigger at the hole position is taken to be the cup's in-hole detector.
    Bounds? FindCup(Vector3 hole)
    {
        int near = Physics.OverlapSphereNonAlloc(hole, 1.5f, _realOverlap, ~0, QueryTriggerInteraction.Collide);
        for (int i = 0; i < near; i++)
        {
            var c = _realOverlap[i];
            if (c != null && c.isTrigger)
                Plugin.Logger.LogInfo($"  trigger near hole: {c.name} (L{c.gameObject.layer} {c.GetIl2CppType().Name}) bounds={c.bounds} sqrDist={c.bounds.SqrDistance(hole):0.####}");
        }

        int n = Physics.OverlapSphereNonAlloc(hole, 0.6f, _realOverlap, ~0, QueryTriggerInteraction.Collide);
        Bounds? best = null;
        float bestVolume = 1f;   // ignore large zones such as out-of-bounds volumes
        for (int i = 0; i < n; i++)
        {
            var c = _realOverlap[i];
            if (c == null || !c.isTrigger) continue;
            var b = c.bounds;
            float volume = b.size.x * b.size.y * b.size.z;
            if (b.SqrDistance(hole) < 0.05f * 0.05f && volume < bestVolume)
            {
                bestVolume = volume;
                best = b;
                Plugin.Logger.LogInfo($"Cup trigger candidate: {c.name} bounds={b} volume={volume:0.####}");
            }
        }
        return best;
    }

    void ApplyLock()
    {
        var user = _ball?.m_player;
        var cam = user?.PlayerCamera;
        var aim = cam?.MouseAiming;
        if (user == null || cam == null || cam.hitpoint == null || aim == null || aim.m_trans == null) return;

        // Rotate the orbit pivot (MouseAim.m_trans's parent) until the shot direction the game
        // derives from the hitpoint matches the solved yaw. One correction per frame.
        Vector3 rest = _ball.restingPosition;
        Vector3 hp = cam.hitpoint.transform.position;
        hp.y = rest.y;
        Vector3 current = -(hp - rest).normalized;
        float delta = Vector3.SignedAngle(current, ShotSolver.DirFromYaw(_lockYaw), Vector3.up);
        var pivot = aim.m_trans.parent;
        if (pivot != null && Mathf.Abs(delta) > 0.001f)
            pivot.Rotate(Vector3.up, delta, Space.World);

        user.m_hitForce = _lockPower;
    }

    bool ShowPath(List<Vector3> pts)
    {
        if (pts.Count < 2 || _ball == null) return false;
        _points.Clear();
        _points.AddRange(pts);
        _line.positionCount = _points.Count;
        for (int i = 0; i < _points.Count; i++)
            _line.SetPosition(i, _points[i]);
        var sphere = _ball.m_sphereCollider;
        _marker.transform.position = _points[_points.Count - 1];
        _marker.transform.localScale = Vector3.one * sphere.radius * MaxAbs(sphere.transform.lossyScale) * 2f;
        return true;
    }

    void SetStatus(string text, float seconds)
    {
        _status = text;
        _statusUntil = Time.unscaledTime + seconds;
    }

    // ---- shot timing for moving obstacles ---------------------------------------
    // The solver reports the launch moment its shot works at, the window around it, and the
    // periods of the obstacles on the path. The next good moment is that launch plus whole
    // cycles of every one of those obstacles; a countdown tells the player when to release.

    const float MinLeadTime = 1.5f;
    const int MaxCycleSearch = 60;

    bool _timed;
    float _target, _windowHalf, _nextLaunch;
    readonly List<float> _periods = new();
    string _countdown;
    bool _countdownNow;
    GUIStyle _bigStyle;

    void SetUpTiming()
    {
        _timed = false;
        _countdown = null;
        if (!_solver.Timed || !_solver.BestHoled) return;
        _periods.Clear();
        foreach (float p in _solver.Periods)
        {
            if (p <= 0f) { Plugin.Logger.LogInfo("Timing: an obstacle on the path is not periodic; using live 'shoot now' cue"); return; }
            bool dup = false;
            foreach (float q in _periods) if (Mathf.Abs(p - q) < 0.01f * q) dup = true;
            if (!dup) _periods.Add(p);
        }
        if (_periods.Count == 0) return;   // nothing moving near the path: any time works
        _target = _solver.TargetLaunch;
        _windowHalf = _solver.WindowHalf;
        _timed = ScheduleNext(Time.time + MinLeadTime);
        Plugin.Logger.LogInfo(_timed
            ? $"Timing: periods [{string.Join(", ", _periods)}]s, window ±{_windowHalf:0.###}s, next launch in {_nextLaunch - Time.time:0.00}s"
            : $"Timing: no common launch moment within {MaxCycleSearch} cycles of [{string.Join(", ", _periods)}]s; using live cue");
    }

    // Earliest moment >= `earliest` at which every obstacle is in the same phase as at the
    // solved launch, within the timing window.
    bool ScheduleNext(float earliest)
    {
        float p0 = _periods[0];
        int m0 = Mathf.CeilToInt((earliest - _target) / p0);
        for (int m = m0; m < m0 + MaxCycleSearch; m++)
        {
            float t = _target + m * p0;
            bool ok = true;
            foreach (float p in _periods)
            {
                float phase = Mathf.Repeat(t - _target + 0.5f * p, p) - 0.5f * p;
                if (Mathf.Abs(phase) > 0.6f * _windowHalf) { ok = false; break; }
            }
            if (ok) { _nextLaunch = t; return true; }
        }
        return false;
    }

    void UpdateCountdown()
    {
        float remaining = _nextLaunch - Time.time;
        if (remaining < -_windowHalf && !ScheduleNext(Time.time + 0.2f))
        {
            _timed = false;
            return;
        }
        remaining = _nextLaunch - Time.time;
        _countdownNow = Mathf.Abs(remaining) <= _windowHalf;
        _countdown = _countdownNow ? "RELEASE NOW!" : $"Release in {remaining:0.0}s";
        SetStatus($"Auto-aim locked — timed shot, window ±{_windowHalf * 1000f:0}ms. The shot fires when you release the button.", 1f);
    }

    void OnGUI()
    {
        if (_locked && _timed && !string.IsNullOrEmpty(_countdown))
        {
            _bigStyle ??= new GUIStyle(GUI.skin.label) { fontSize = 48, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter };
            var old = GUI.color;
            GUI.color = _countdownNow ? Color.green : Color.yellow;
            GUI.Label(new Rect(0f, Screen.height * 0.18f, Screen.width, 70f), _countdown, _bigStyle);
            GUI.color = old;
        }

        if (string.IsNullOrEmpty(_status) || Time.unscaledTime > _statusUntil) return;
        var rect = new Rect(20f, Screen.height - 60f, Mathf.Min(900f, Screen.width - 40f), 36f);
        GUI.Box(rect, "");
        GUI.Label(new Rect(rect.x + 10f, rect.y + 8f, rect.width - 20f, rect.height - 10f), _status);
    }

    readonly PipCamera _pip = new();
    readonly SimWorld _sim = new();
    bool _usePhysX = true;
    bool _simBroken;

    // Per-frame cost cap for the PhysX prediction; a full-power shot finishes in a few frames.
    const double SimBudgetMs = 4.0;
    const float ForceTolerance = 0.005f;      // relative change that triggers a re-simulation
    const float AngleTolerance = 0.1f;        // degrees
    const float PosTolerance = 0.01f;

    Vector3 _jobForce, _jobPos, _doneForce, _donePos;
    bool _hasJob, _hasDone;
    int _jobStepsPerFrame = 1;
    float _baseFrameDt = 1f / 60f;
    bool _simRanThisFrame;
    float _nextRebuildTime;
    readonly List<Vector3> _done = new();
    float _jobStartTime, _predLatency;
    bool _doneGoesIn;
    readonly List<SimWorld.Contact> _doneContacts = new();

    static bool SameShot(Vector3 forceA, Vector3 posA, Vector3 forceB, Vector3 posB) =>
        Mathf.Abs(forceA.magnitude - forceB.magnitude) <= ForceTolerance * Mathf.Max(1f, forceB.magnitude)
        && Vector3.Angle(forceA, forceB) <= AngleTolerance
        && (posA - posB).sqrMagnitude <= PosTolerance * PosTolerance;

    // Runs the PhysX prediction incrementally. The last finished path stays on screen while the
    // next one computes; a new job only starts once the running one is done, always from the
    // latest input. Returns true when _points holds a path for the current input.
    bool StepPhysX(BallMovement ball, Vector3 force, Vector3 pos)
    {
        // With moving obstacles the answer changes with time, so keep re-simulating.
        bool stale = !(_hasJob && SameShot(force, pos, _jobForce, _jobPos)) || _sim.HasMovingObstacles;
        if (!_sim.JobActive && stale)
        {
            _sim.SyncColliders(force: false);
            if (!_sim.HasGroundUnder(pos) && Time.time >= _nextRebuildTime)
            {
                Plugin.Logger.LogInfo("Sim world has no ground under the ball; rebuilding");
                _sim.Rebuild(ball);
                _nextRebuildTime = Time.time + 3f;
            }
            // The game refreshes drag once per rendered frame; use the frame time measured
            // while the prediction was idle so our own cost does not skew it.
            _jobStepsPerFrame = Mathf.Max(1, Mathf.RoundToInt(_baseFrameDt / Time.fixedDeltaTime));
            // Simulate as if fired when this result will be on screen: now plus the time the
            // previous prediction took to compute.
            _sim.BeginShot(ball, force, _jobStepsPerFrame, MaxSimTime, Time.time + _predLatency);
            _jobStartTime = Time.time;
            _jobForce = force;
            _jobPos = pos;
            _hasJob = true;
        }

        if (_sim.JobActive)
        {
            _simRanThisFrame = true;
            if (_sim.Step(SimBudgetMs))
            {
                _done.Clear();
                _done.AddRange(_sim.JobPoints);
                _doneContacts.Clear();
                _doneContacts.AddRange(_sim.JobContacts);
                _doneForce = _jobForce;
                _donePos = _jobPos;
                _hasDone = true;
                _predLatency = Mathf.Lerp(_predLatency, Time.time - _jobStartTime, 0.5f);
                _doneGoesIn = _solver.HasCup && _solver.PathHoles(_done);
            }
        }

        // Prefer the finished path if it was for this ball position; otherwise show progress.
        var src = _hasDone && (_donePos - pos).sqrMagnitude <= PosTolerance * PosTolerance ? _done : _sim.JobPoints;
        _points.Clear();
        _points.AddRange(src);
        return src == _done && SameShot(force, pos, _doneForce, _donePos);
    }

    void TrackFrameTime()
    {
        if (!_simRanThisFrame)
            _baseFrameDt = Mathf.Lerp(_baseFrameDt, Time.unscaledDeltaTime, 0.1f);
        _simRanThisFrame = false;
    }

    bool TryEnsureSim(BallMovement ball)
    {
        if (_simBroken) return false;
        try
        {
            _sim.EnsureBuilt(ball);
            return _sim.Ready;
        }
        catch (Exception e)
        {
            _simBroken = true;
            Plugin.Logger.LogError($"Sim world failed, falling back to spherecast: {e}");
            return false;
        }
    }

    // Sphere-cast integrator. Gravity, PhysX linear damping with the game's drag model,
    // and restitution against surfaces. Rolling friction/spin are not modelled yet.
    void Simulate(Vector3 pos, Vector3 vel, float dt, float radius, int mask, Rigidbody self, PhysicMaterial ballMat, float envDrag)
    {
        _points.Clear();
        _points.Add(pos);
        _simHits = 0;
        Vector3 g = Physics.gravity;
        float bounceThreshold = Physics.bounceThreshold;
        bool cap = true;
        int steps = Mathf.Min((int)(MaxSimTime / dt), MaxPoints);

        for (int step = 0; step < steps; step++)
        {
            float t = step * dt;
            float speed = vel.magnitude;
            if (speed > SlowDragMaxSpeed || t >= ShotGraceTime)
                cap = false;

            float drag = InitialDrag + envDrag;
            if (speed > SleepSpeed && speed <= SlowDragMaxSpeed && !cap)
            {
                float s = 1f - speed;
                drag = s * s * _ball.dragToSlow + envDrag + InitialDrag;
            }

            vel += g * dt;
            vel *= Mathf.Max(0f, 1f - drag * dt);   // PhysX linear damping

            float remaining = dt;
            for (int iter = 0; iter < 4 && remaining > 0f; iter++)
            {
                float dist = vel.magnitude * remaining;
                if (dist < 1e-6f) break;
                Vector3 dirN = vel / vel.magnitude;
                if (!CastNearest(pos, radius, dirN, dist + Skin, mask, self, out var hit))
                {
                    pos += vel * remaining;
                    break;
                }

                _simHits++;
                float travel = Mathf.Max(0f, hit.distance - Skin);
                pos += dirN * travel;
                remaining *= 1f - Mathf.Clamp01(travel / dist);

                var surfMat = hit.collider.sharedMaterial;
                float e = Combine(ballMat?.bounciness ?? 0f, surfMat?.bounciness ?? 0f,
                                  ballMat?.bounceCombine ?? PhysicMaterialCombine.Average,
                                  surfMat?.bounceCombine ?? PhysicMaterialCombine.Average);
                float mu = Combine(ballMat?.dynamicFriction ?? 0f, surfMat?.dynamicFriction ?? 0f,
                                   ballMat?.frictionCombine ?? PhysicMaterialCombine.Average,
                                   surfMat?.frictionCombine ?? PhysicMaterialCombine.Average);
                float vn = Vector3.Dot(vel, hit.normal);
                if (vn < 0f)
                {
                    Vector3 tangent = vel - vn * hit.normal;
                    float bounce = -vn > bounceThreshold ? -vn * e : 0f;
                    // Coulomb friction: the normal impulse caps how much sliding speed is removed.
                    // The ball's inertia is forced to (1,1,1), so it barely spins up and keeps sliding.
                    float normalImpulse = bounce - vn;
                    float vt = tangent.magnitude;
                    if (vt > 0f)
                        tangent *= Mathf.Max(0f, vt - mu * normalImpulse) / vt;
                    vel = tangent + hit.normal * bounce;
                }
                pos += hit.normal * Skin * 0.5f;
            }

            _points.Add(pos);

            if (t > ShotGraceTime && vel.magnitude < SleepSpeed)
                break;
        }
    }

    // Casts start BackOff behind the sphere so surfaces it is already touching (e.g. the ground
    // under a resting ball) are still reported; returned distances are relative to the real origin.
    const float BackOff = 0.05f;

    bool CastNearest(Vector3 origin, float radius, Vector3 dir, float dist, int mask, Rigidbody self, out RaycastHit best)
    {
        best = default;
        int n = Physics.SphereCastNonAlloc(origin - dir * BackOff, radius, dir, _hitBuffer, dist + BackOff, mask, QueryTriggerInteraction.Ignore);
        float bestDist = float.MaxValue;
        for (int i = 0; i < n; i++)
        {
            var h = _hitBuffer[i];
            if (h.collider == null || h.collider.attachedRigidbody == self) continue;
            if (h.distance <= 0f && h.point == Vector3.zero) continue;   // started overlapping
            if (Vector3.Dot(h.normal, dir) >= 0f) continue;              // surface we're moving away from
            if (h.distance < bestDist) { bestDist = h.distance; best = h; }
        }
        if (bestDist == float.MaxValue) return false;
        best.distance = Mathf.Max(0f, bestDist - BackOff);
        return true;
    }

    static float Combine(float a, float b, PhysicMaterialCombine ma, PhysicMaterialCombine mb)
    {
        // PhysX picks the higher-priority mode: Average < Minimum < Multiply < Maximum.
        var mode = (PhysicMaterialCombine)Math.Max((int)ma, (int)mb);
        return mode switch
        {
            PhysicMaterialCombine.Minimum => Mathf.Min(a, b),
            PhysicMaterialCombine.Multiply => a * b,
            PhysicMaterialCombine.Maximum => Mathf.Max(a, b),
            _ => (a + b) * 0.5f,
        };
    }

    // Idle balls sit on m_ignoreLayer; UpdateCollisionLayer switches to m_collideLayer once moving.
    // The layer seen on a moving ball is the most reliable source, so it is cached when observed.
    int _movingLayer = -1;
    string _maskSource;

    int ResolveMask(BallMovement ball)
    {
        int layer = _movingLayer >= 0 ? _movingLayer
                  : ball.m_collideLayer is >= 0 and < 32 ? ball.m_collideLayer
                  : -1;
        int mask = layer >= 0 ? CollisionMaskFor(layer) : 0;
        _maskSource = layer >= 0 ? $"layer{layer}" : "none";
        if (mask == 0)
        {
            mask = ~((1 << 2) | (1 << 21) | (1 << 22));
            _maskSource += "+fallback";
        }
        return mask;
    }

    void ObserveMovingLayer()
    {
        if (_ball == null || _ball.IsBallIdle) return;
        int layer = _ball.gameObject.layer;
        if (layer != _movingLayer)
        {
            _movingLayer = layer;
            Plugin.Logger.LogInfo($"Moving ball layer={layer} ({LayerMask.LayerToName(layer)}) matrixMask=0x{CollisionMaskFor(layer):X}");
        }
    }

    static string DescribeGround(Vector3 pos, int mask)
    {
        if (!Physics.Raycast(pos + Vector3.up * 0.5f, Vector3.down, out var hit, 5f, mask, QueryTriggerInteraction.Ignore))
            return "nothing within 5m";
        var go = hit.collider.gameObject;
        var m = hit.collider.sharedMaterial;
        string mat = m == null ? "none" : $"{m.name} dynF={m.dynamicFriction} bounce={m.bounciness} {m.frictionCombine}/{m.bounceCombine}";
        return $"{go.name} layer={go.layer} ({LayerMask.LayerToName(go.layer)}) collider={hit.collider.GetIl2CppType().Name} mat={mat}";
    }

    static int CollisionMaskFor(int layer)
    {
        int mask = 0;
        for (int i = 0; i < 32; i++)
            if (!Physics.GetIgnoreLayerCollision(layer, i))
                mask |= 1 << i;
        return mask;
    }

    static float MaxAbs(Vector3 v) => Mathf.Max(Mathf.Abs(v.x), Mathf.Max(Mathf.Abs(v.y), Mathf.Abs(v.z)));

    // ---- visuals -------------------------------------------------------------

    void EnsureVisuals()
    {
        if (_line != null && _marker != null) return;

        var mat = new Material(FindShader());
        mat.color = new Color(0.2f, 1f, 0.6f, 0.9f);
        DontDestroyOnLoad(mat);

        var lineGo = new GameObject("BlazeGolf_Line");
        DontDestroyOnLoad(lineGo);
        _line = lineGo.AddComponent<LineRenderer>();
        _line.material = mat;
        _line.startWidth = 0.04f;
        _line.endWidth = 0.02f;
        _line.startColor = mat.color;
        _line.endColor = mat.color;
        _line.useWorldSpace = true;
        _line.numCapVertices = 2;
        _line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        _line.receiveShadows = false;

        _marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        _marker.name = "BlazeGolf_Marker";
        Destroy(_marker.GetComponent<Collider>());
        var r = _marker.GetComponent<MeshRenderer>();
        r.material = mat;
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        DontDestroyOnLoad(_marker);
    }

    static Shader FindShader()
    {
        foreach (var name in new[] { "Universal Render Pipeline/Unlit", "Sprites/Default", "Unlit/Color", "Hidden/Internal-Colored" })
        {
            var s = Shader.Find(name);
            if (s != null)
            {
                Plugin.Logger.LogInfo($"Using shader {name}");
                return s;
            }
        }
        Plugin.Logger.LogWarning("No preferred shader found; falling back to Standard");
        return Shader.Find("Standard");
    }
}

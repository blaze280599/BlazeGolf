using System.Collections.Generic;
using SysStopwatch = System.Diagnostics.Stopwatch;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace BlazeGolf;

// A private PhysX world holding static copies of the course colliders plus a ghost ball that
// mirrors the local ball's Rigidbody. Shots are simulated here step by step with the game's
// drag model (BallMovement.UpdateAllBalls) so PhysX itself resolves friction, bounces and slopes.
public partial class SimWorld
{
    // BallMovement constants (see NOTES.md)
    const float InitialDrag = 0.5f;
    const float SleepSpeed = 0.1f;
    const float SleepAccelSq = 0.1f;
    const float SlowDragMaxSpeed = 1.0f;
    const float ShotGraceTime = 0.5f;
    const float SlowContactOffset = 0.0005f;
    const float FastContactOffset = 0.000005f;
    const string SimScenePrefix = "BlazeGolfSim_";

    Scene _scene;
    PhysicsScene _physics;
    Rigidbody _ghost;
    SphereCollider _ghostSphere;
    int _builtForScene = -1;

    public bool Ready => _ghost != null;
    public int ColliderCount { get; private set; }
    public double LastSimMs { get; private set; }
    public int LastSteps { get; private set; }

    public void EnsureBuilt(BallMovement ball)
    {
        int sceneHandle = ball.gameObject.scene.handle;
        if (Ready && _builtForScene == sceneHandle) return;
        Rebuild(ball);
    }

    public void Rebuild(BallMovement ball)
    {
        Destroy();
        var sw = SysStopwatch.StartNew();
        _scene = SceneManager.CreateScene(SimScenePrefix + Time.frameCount, new CreateSceneParameters { localPhysicsMode = LocalPhysicsMode.Physics3D });
        _physics = PhysicsSceneExtensions.GetPhysicsScene(_scene);

        ClearMirror();
        SyncColliders(force: true);

        CreateGhost(ball);
        _builtForScene = ball.gameObject.scene.handle;
        Plugin.Logger.LogInfo($"Sim world built: {ColliderCount} colliders cloned, {MoverCount} movers, {sw.ElapsedMilliseconds} ms");
    }

    public void Destroy()
    {
        if (_scene.IsValid() && _scene.isLoaded)
            SceneManager.UnloadSceneAsync(_scene);
        ClearMirror();
        _ghost = null;
        _ghostSphere = null;
        ClearBatchPool();
        _builtForScene = -1;
    }

    void CreateGhost(BallMovement ball)
    {
        var src = ball.m_rigidBody;
        var srcSphere = ball.m_sphereCollider;
        var go = new GameObject("sim_ghostBall");
        SceneManager.MoveGameObjectToScene(go, _scene);
        go.transform.localScale = srcSphere.transform.lossyScale;

        _ghostSphere = go.AddComponent<SphereCollider>();
        _ghostSphere.center = srcSphere.center;
        _ghostSphere.radius = srcSphere.radius;
        _ghostSphere.sharedMaterial = srcSphere.sharedMaterial;

        _ghost = go.AddComponent<Rigidbody>();
        CopyBody(src, _ghost);
    }

    static void CopyBody(Rigidbody src, Rigidbody dst)
    {
        dst.mass = src.mass;
        dst.useGravity = src.useGravity;
        dst.constraints = src.constraints;
        dst.collisionDetectionMode = src.collisionDetectionMode;
        dst.interpolation = RigidbodyInterpolation.None;
        dst.maxAngularVelocity = src.maxAngularVelocity;
        dst.maxDepenetrationVelocity = src.maxDepenetrationVelocity;
        dst.solverIterations = src.solverIterations;
        dst.solverVelocityIterations = src.solverVelocityIterations;
        dst.sleepThreshold = src.sleepThreshold;
        dst.centerOfMass = src.centerOfMass;
        dst.inertiaTensor = src.inertiaTensor;
        dst.inertiaTensorRotation = src.inertiaTensorRotation;
    }

    // ---- incremental shot simulation ------------------------------------------------------
    // A shot is started with BeginShot and advanced with Step under a per-frame time budget,
    // so a long prediction is spread over several frames instead of stalling one.

    const float FallLimit = 30f;
    const float ContactDeltaV = 1f;      // velocity change (beyond gravity) logged as an impact
    const int MaxContacts = 200;

    public struct Contact
    {
        public int Step;
        public Vector3 Pos;
        public Vector3 DeltaV;
        public string Colliders;
    }

    readonly List<Contact> _jobContacts = new();
    readonly Il2CppInterop.Runtime.InteropTypes.Arrays.Il2CppReferenceArray<Collider> _overlap = new(16);
    Vector3 _jobPrevVel;
    float _ghostRadius;

    public List<Contact> JobContacts => _jobContacts;

    // Names of sim colliders within `radius` of `pos`, excluding the ghost ball.
    public string SimCollidersAt(Vector3 pos, float radius)
    {
        int n = _physics.OverlapSphere(pos, radius, _overlap, ~0, QueryTriggerInteraction.Collide);
        var names = new List<string>();
        for (int i = 0; i < n; i++)
        {
            var c = _overlap[i];
            if (c == null || c == _ghostSphere) continue;
            names.Add($"{c.name}(L{c.gameObject.layer})");
        }
        return names.Count == 0 ? "none" : string.Join(", ", names);
    }

    readonly List<Vector3> _jobPoints = new();
    bool _jobActive;
    int _jobStep, _jobMaxSteps, _jobStepsPerFrame;
    bool _jobCap;
    Vector3 _jobFrameVel, _jobStart;
    float _jobFrameTime, _jobEnvDrag, _jobDragToSlow, _jobAngDragToSlow;
    double _jobMs;

    // Optional region (the cup trigger) that ends a job early and marks it as a hit.
    public Bounds? Target { get; set; }
    public bool JobHitTarget { get; private set; }

    // Geometric cup test used alongside / instead of the trigger: inside the cup radius and
    // sunk below the rim.
    public bool UseCup { get; set; }
    public Vector3 CupCenter { get; set; }
    public float CupRadius { get; set; }
    public float CupMaxY { get; set; }

    bool InTarget(Vector3 p)
    {
        if (Target.HasValue && Target.Value.Contains(p)) return true;
        if (!UseCup || p.y > CupMaxY) return false;
        float dx = p.x - CupCenter.x, dz = p.z - CupCenter.z;
        return dx * dx + dz * dz < CupRadius * CupRadius;
    }

    // Height of the sim world's surface under (x, z), searched downward from `fromY`.
    public float? SurfaceY(float x, float z, float fromY)
    {
        if (_physics.Raycast(new Vector3(x, fromY, z), Vector3.down, out RaycastHit hit, 20f, ~0, QueryTriggerInteraction.Ignore))
            return hit.point.y;
        return null;
    }

    public bool JobActive => _jobActive;
    public List<Vector3> JobPoints => _jobPoints;

    // False when the sim world has no ground under the ball (geometry not cloned yet).
    public bool HasGroundUnder(Vector3 pos) =>
        _physics.Raycast(pos + Vector3.up * 0.5f, Vector3.down, 5f, ~0, QueryTriggerInteraction.Ignore);

    // `force` is what the game passes to AddForce (curved hitForce + 1); `stepsPerFrame`
    // emulates the drag model being refreshed once per rendered frame, not every physics step.
    public void BeginShot(BallMovement ball, Vector3 force, int stepsPerFrame, float maxTime, float launchTime)
    {
        var src = ball.m_rigidBody;
        float dt = Time.fixedDeltaTime;
        _jobEnvDrag = ball.m_environmentalDragToApply;
        _jobDragToSlow = ball.dragToSlow;
        _jobAngDragToSlow = ball.anglularDragToSlow;

        PrepareMovers(launchTime);
        CopyBody(src, _ghost);
        _ghost.gameObject.layer = ball.gameObject.layer;
        _ghostSphere.contactOffset = ball.m_sphereCollider.contactOffset;
        _ghost.isKinematic = false;          // may have been parked by a solver batch
        _ghost.detectCollisions = true;
        _ghost.position = src.position;
        _ghost.rotation = src.rotation;
        _ghost.transform.SetPositionAndRotation(src.position, src.rotation);
        _ghost.velocity = Vector3.zero;
        _ghost.angularVelocity = src.angularVelocity;
        _ghost.WakeUp();

        // CheckForBallHit on release
        _ghost.angularDrag = 1f;
        _ghost.drag = InitialDrag + _jobEnvDrag;
        _ghost.AddForce(force, ForceMode.Force);

        _jobPoints.Clear();
        _jobPoints.Add(src.position);
        _jobContacts.Clear();
        _jobPrevVel = force / src.mass * dt;   // expected velocity after the launch step
        _ghostRadius = _ghostSphere.radius * _ghost.transform.lossyScale.x;
        _jobStart = src.position;
        _jobCap = true;
        _jobFrameVel = Vector3.zero;
        _jobFrameTime = 0f;
        _jobStep = 0;
        _jobMaxSteps = Mathf.CeilToInt(maxTime / dt);
        _jobStepsPerFrame = Mathf.Max(1, stepsPerFrame);
        _jobMs = 0;
        JobHitTarget = false;
        _jobActive = true;
    }

    // Advances the active shot until it finishes or `budgetMs` is spent. Returns true when done.
    public bool Step(double budgetMs)
    {
        if (!_jobActive) return true;
        var sw = SysStopwatch.StartNew();
        float dt = Time.fixedDeltaTime;
        bool done = false;

        while (!done && sw.Elapsed.TotalMilliseconds < budgetMs)
        {
            StepMovers((_jobStep + 1) * dt);
            _physics.Simulate(dt);
            Vector3 p = _ghost.position;
            _jobPoints.Add(p);

            Vector3 vNow = _ghost.velocity;
            Vector3 dv = vNow - _jobPrevVel - Physics.gravity * dt;
            if (_jobStep > 0 && dv.sqrMagnitude > ContactDeltaV * ContactDeltaV && _jobContacts.Count < MaxContacts)
                _jobContacts.Add(new Contact { Step = _jobStep + 1, Pos = p, DeltaV = dv, Colliders = SimCollidersAt(p, _ghostRadius * 1.5f) });
            _jobPrevVel = vNow;
            _jobFrameTime += dt;
            _jobStep++;

            if (InTarget(p)) { JobHitTarget = true; done = true; break; }
            if (_jobStep >= _jobMaxSteps || p.y < _jobStart.y - FallLimit) { done = true; break; }
            if (_jobStep % _jobStepsPerFrame != 0) continue;

            if (FrameUpdate(_ghost, _ghostSphere, _jobStep * dt, ref _jobCap, ref _jobFrameVel, ref _jobFrameTime,
                            _jobEnvDrag, _jobDragToSlow, _jobAngDragToSlow, out _))
                done = true;
        }

        _jobMs += sw.Elapsed.TotalMilliseconds;
        if (done)
        {
            _jobActive = false;
            LastSteps = _jobStep;
            LastSimMs = _jobMs;
        }
        return done;
    }

    public void CancelShot() => _jobActive = false;

    // BallMovement.UpdateAllBalls for one ball, run once per rendered frame. Returns true when
    // the game would consider the ball idle (or PhysX put it to sleep).
    static bool FrameUpdate(Rigidbody rb, SphereCollider col, float t, ref bool cap, ref Vector3 frameVel,
                            ref float frameTime, float envDrag, float dragToSlow, float angDragToSlow, out float speed)
    {
        Vector3 v = rb.velocity;
        speed = v.magnitude;
        float accelSq = ((v - frameVel) / frameTime).sqrMagnitude;
        frameVel = v;
        frameTime = 0f;
        if (t >= ShotGraceTime) cap = false;

        rb.drag = InitialDrag + envDrag;
        if (speed > SleepSpeed)
        {
            if (speed <= SlowDragMaxSpeed)
            {
                col.contactOffset = SlowContactOffset;
                if (!cap)
                {
                    float s = 1f - speed;
                    rb.drag = s * s * dragToSlow + envDrag + InitialDrag;
                    rb.angularDrag = s * s * angDragToSlow;
                }
            }
            else
            {
                rb.angularDrag = 1f;
                cap = false;
                col.contactOffset = FastContactOffset;
            }
        }

        return (t > ShotGraceTime && speed < SleepSpeed && accelSq < SleepAccelSq) || rb.IsSleeping();
    }
}

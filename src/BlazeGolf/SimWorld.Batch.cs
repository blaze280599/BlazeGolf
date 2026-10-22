using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using SysStopwatch = System.Diagnostics.Stopwatch;

namespace BlazeGolf;

// Batched shots for the solver: up to BatchSize ghost balls that ignore each other are launched
// together, so one PhysicsScene.Simulate call advances every candidate. A Simulate call's fixed
// cost dominates for a single ball, so this is far cheaper than simulating candidates in turn.
public partial class SimWorld
{
    public const int BatchSize = 40;

    const float HopelessMinTime = 1f;
    const float HopelessMaxSpeed = 2f;
    const float HopelessDecel = 0.3f;     // below the measured ~0.78 m/s², so reach is overestimated
    const float HopelessMargin = 3f;
    static readonly Vector3 ParkPosition = new(0f, -10000f, 0f);

    public class BatchShot
    {
        public float Yaw, Power;
        public Vector3 Force;
        public readonly List<Vector3> Points = new();
        public bool HitTarget, Done;
        public float MinApproach;

        internal Rigidbody Rb;
        internal SphereCollider Col;
        internal bool Cap;
        internal Vector3 FrameVel;
        internal float FrameTime;
    }

    readonly List<Rigidbody> _poolRb = new();
    readonly List<SphereCollider> _poolCol = new();
    readonly List<BatchShot> _batch = new();
    int _batchStep, _batchMaxSteps, _batchStepsPerFrame, _batchActive;
    float _batchEnvDrag, _batchDragToSlow, _batchAngDragToSlow;
    Vector3 _batchStart, _batchHole;

    public bool BatchActive => _batchActive > 0;
    public long StatSimulateCalls { get; private set; }
    public long StatBallSteps { get; private set; }
    public double StatMs { get; private set; }

    public void ResetStats()
    {
        StatSimulateCalls = 0;
        StatBallSteps = 0;
        StatMs = 0;
    }

    void ClearBatchPool()
    {
        _poolRb.Clear();
        _poolCol.Clear();
        _batch.Clear();
        _batchActive = 0;
    }

    void EnsurePool(BallMovement ball)
    {
        if (_poolRb.Count >= BatchSize) return;
        var srcSphere = ball.m_sphereCollider;
        for (int i = _poolRb.Count; i < BatchSize; i++)
        {
            var go = new GameObject("sim_batchBall" + i);
            SceneManager.MoveGameObjectToScene(go, _scene);
            go.transform.localScale = srcSphere.transform.lossyScale;
            var col = go.AddComponent<SphereCollider>();
            col.center = srcSphere.center;
            col.radius = srcSphere.radius;
            col.sharedMaterial = srcSphere.sharedMaterial;
            var rb = go.AddComponent<Rigidbody>();
            CopyBody(ball.m_rigidBody, rb);
            Park(rb);
            _poolRb.Add(rb);
            _poolCol.Add(col);
        }
    }

    // Parked balls stay in the scene (disabling would drop their IgnoreCollision pairs) but are
    // kinematic, collision-free and far away.
    static void Park(Rigidbody rb)
    {
        rb.isKinematic = true;
        rb.detectCollisions = false;
        rb.position = ParkPosition;
        rb.transform.position = ParkPosition;
    }

    // Launches every shot from the ball's current position. `hole` enables the early-out for
    // slow shots that can no longer reach it.
    public void BeginBatch(BallMovement ball, List<BatchShot> shots, int stepsPerFrame, float maxTime, Vector3 hole, float launchTime, Vector3? startPos = null)
    {
        _jobActive = false;
        EnsurePool(ball);
        PrepareMovers(launchTime);
        Park(_ghost);

        var src = ball.m_rigidBody;
        float dt = Time.fixedDeltaTime;
        _batchEnvDrag = ball.m_environmentalDragToApply;
        _batchDragToSlow = ball.dragToSlow;
        _batchAngDragToSlow = ball.anglularDragToSlow;
        // A planned follow-up shot starts at rest where the previous one stopped.
        Vector3 start = startPos ?? src.position;
        Quaternion startRot = startPos.HasValue ? Quaternion.identity : src.rotation;
        Vector3 startSpin = startPos.HasValue ? Vector3.zero : src.angularVelocity;
        _batchStart = start;
        _batchHole = hole;
        _batchStep = 0;
        _batchMaxSteps = Mathf.CeilToInt(maxTime / dt);
        _batchStepsPerFrame = Mathf.Max(1, stepsPerFrame);
        float contactOffset = ball.m_sphereCollider.contactOffset;
        int layer = ball.gameObject.layer;

        _batch.Clear();
        int count = Mathf.Min(shots.Count, BatchSize);
        for (int i = 0; i < count; i++)
        {
            var s = shots[i];
            var rb = _poolRb[i];
            var col = _poolCol[i];
            CopyBody(src, rb);
            rb.gameObject.layer = layer;
            col.contactOffset = contactOffset;
            rb.isKinematic = false;
            rb.detectCollisions = true;
            rb.position = start;
            rb.rotation = startRot;
            rb.transform.SetPositionAndRotation(start, startRot);
            rb.velocity = Vector3.zero;
            rb.angularVelocity = startSpin;
            rb.angularDrag = 1f;                       // CheckForBallHit on release
            rb.drag = InitialDrag + _batchEnvDrag;
            rb.AddForce(s.Force, ForceMode.Force);
            rb.WakeUp();

            s.Rb = rb;
            s.Col = col;
            s.Cap = true;
            s.FrameVel = Vector3.zero;
            s.FrameTime = 0f;
            s.Done = false;
            s.HitTarget = false;
            s.MinApproach = float.MaxValue;
            s.Points.Clear();
            s.Points.Add(start);
            _batch.Add(s);
        }
        for (int i = count; i < _poolRb.Count; i++)
            Park(_poolRb[i]);

        // Candidates share one start point; make sure they never touch each other or the ghost.
        for (int i = 0; i < count; i++)
        {
            Physics.IgnoreCollision(_poolCol[i], _ghostSphere);
            for (int j = i + 1; j < count; j++)
                Physics.IgnoreCollision(_poolCol[i], _poolCol[j]);
        }
        _batchActive = count;
    }

    // Advances the batch until every shot is done or `budgetMs` is spent. Returns true when done.
    public bool StepBatch(double budgetMs)
    {
        if (_batchActive == 0) return true;
        var sw = SysStopwatch.StartNew();
        float dt = Time.fixedDeltaTime;
        Vector2 hole2 = new(_batchHole.x, _batchHole.z);

        while (_batchActive > 0 && sw.Elapsed.TotalMilliseconds < budgetMs)
        {
            StepMovers((_batchStep + 1) * dt);
            _physics.Simulate(dt);
            StatSimulateCalls++;
            _batchStep++;
            float t = _batchStep * dt;
            bool frameTick = _batchStep % _batchStepsPerFrame == 0;

            foreach (var s in _batch)
            {
                if (s.Done) continue;
                Vector3 p = s.Rb.position;
                s.Points.Add(p);
                s.FrameTime += dt;
                StatBallSteps++;
                float approach = Vector2.Distance(new Vector2(p.x, p.z), hole2);
                if (approach < s.MinApproach) s.MinApproach = approach;

                if (InTarget(p)) { s.HitTarget = true; Finish(s); continue; }
                if (_batchStep >= _batchMaxSteps || p.y < _batchStart.y - FallLimit) { Finish(s); continue; }
                if (!frameTick) continue;

                if (FrameUpdate(s.Rb, s.Col, t, ref s.Cap, ref s.FrameVel, ref s.FrameTime,
                                _batchEnvDrag, _batchDragToSlow, _batchAngDragToSlow, out float speed))
                {
                    Finish(s);
                    continue;
                }
                // Slow and far: even with generous rolling it cannot reach the hole.
                if (t > HopelessMinTime && speed < HopelessMaxSpeed
                    && approach > speed * speed / (2f * HopelessDecel) + HopelessMargin)
                    Finish(s);
            }
        }

        StatMs += sw.Elapsed.TotalMilliseconds;
        return _batchActive == 0;
    }

    void Finish(BatchShot s)
    {
        s.Done = true;
        _batchActive--;
        Park(s.Rb);
    }

    // True when a ball-sized sphere can roll straight from `from` towards `to` without hitting
    // anything (stopping short of `to` by `stopShort`, i.e. before the cup).
    public bool LineClear(Vector3 from, Vector3 to, float radius, float stopShort)
    {
        Vector3 origin = from + Vector3.up * (radius * 0.5f);
        Vector3 target = new(to.x, origin.y, to.z);
        Vector3 d = target - origin;
        float dist = d.magnitude - stopShort;
        if (dist <= 0f) return true;
        return !_physics.SphereCast(origin, radius * 0.9f, d / d.magnitude, out RaycastHit _, dist, ~0, QueryTriggerInteraction.Ignore);
    }

    public void CancelBatch()
    {
        foreach (var s in _batch)
            if (!s.Done) { s.Done = true; Park(s.Rb); }
        _batchActive = 0;
    }
}

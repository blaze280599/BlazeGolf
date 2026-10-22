using System.Collections.Generic;
using UnityEngine;
using SysStopwatch = System.Diagnostics.Stopwatch;

namespace BlazeGolf;

// Searches aim (yaw) and power for a shot that ends in the cup, using the PhysX sim world.
// Candidates are evaluated in batches of up to SimWorld.BatchSize balls simulated together;
// the search is a resumable routine that Tick advances under a per-frame time budget.
public class ShotSolver
{
    public enum State { Idle, Solving, Done }

    const float MaxSimTime = 6f;
    const float CupRadius = 0.18f;
    const float RimSampleRadius = 0.35f;
    const float SinkDepth = 0.08f;          // ball centre this far below resting-on-rim height = in
    const int GridYaws = 30;                                            // every 12°
    static readonly float[] GridPowers = { 0.1f, 0.2f, 0.35f, 0.5f, 0.7f, 0.95f };
    const int RefineRounds = 5;
    const int RefineCandidates = 3;
    const int TimingSteps = 5;
    const int TwoShotCandidates = 10;
    const float TimingStep = 0.075f;

    public State Status { get; private set; } = State.Idle;
    public string Phase { get; private set; } = "";
    public int Evaluations { get; private set; }
    public int Batches { get; private set; }
    public double ElapsedMs { get; private set; }
    public float BestYaw { get; private set; }
    public float BestPower { get; private set; }
    public float BestCost { get; private set; } = float.MaxValue;
    public bool BestHoled { get; private set; }
    public readonly List<Vector3> BestPoints = new();

    // Set when moving obstacles matter: the launch moment the solution was found for (centred in
    // its window), the window half-width, and the periods of the obstacles near the path.
    public bool Timed { get; private set; }
    public float TargetLaunch { get; private set; }
    public float WindowHalf { get; private set; }
    public List<float> Periods { get; private set; } = new();

    struct Result
    {
        public float Yaw, Power, Cost;
        public bool Holed;
        public Vector3 End;
        public bool Rested;      // came to rest on a surface (usable as a second-shot position)
    }

    // Two-shot plan, used when no hole-in-one exists: the first shot (Best*) leaves the ball at
    // Leave, from where SecondYaw/SecondPower sinks it.
    public bool TwoShot { get; private set; }
    public Vector3 Leave { get; private set; }
    public float SecondYaw { get; private set; }
    public float SecondPower { get; private set; }
    bool _stage2;
    float _ballRadius;

    SimWorld _sim;
    BallMovement _ball;
    IEnumerator<bool> _routine;
    Vector3 _start, _hole;
    float _min, _max;
    int _stepsPerFrame;
    float _launchTime;
    readonly SysStopwatch _clock = new();
    readonly List<SimWorld.BatchShot> _shotPool = new();
    readonly List<SimWorld.BatchShot> _shots = new();
    readonly List<Result> _results = new();     // results of the latest batch, in submission order
    readonly List<Result> _all = new();         // every result so far

    public static Vector3 DirFromYaw(float yaw) =>
        new(Mathf.Sin(yaw * Mathf.Deg2Rad), 0f, Mathf.Cos(yaw * Mathf.Deg2Rad));

    static float YawOf(Vector3 v) => Mathf.Atan2(v.x, v.z) * Mathf.Rad2Deg;

    public void Start(SimWorld sim, BallMovement ball, Vector3 hole, Bounds? cup, int stepsPerFrame, float launchTime)
    {
        _sim = sim;
        _ball = ball;
        _hole = hole;
        _stepsPerFrame = stepsPerFrame;
        _launchTime = launchTime;
        _sim.CancelShot();
        _sim.Target = cup;
        ConfigureCup(ball);
        _sim.ResetStats();
        _start = ball.m_rigidBody.position;
        _min = ball.minForce.Value;
        _max = ball.m_maxForce.Value;
        Evaluations = 0;
        Batches = 0;
        BestCost = float.MaxValue;
        BestHoled = false;
        BestPoints.Clear();
        Timed = false;
        Periods = new List<float>();
        TwoShot = false;
        _stage2 = false;
        _all.Clear();
        _clock.Restart();
        _routine = Run();
        Status = State.Solving;
        if (!_routine.MoveNext()) Complete();
    }

    public void Cancel()
    {
        if (Status == State.Solving) _sim.CancelBatch();
        Finish();
        Status = State.Idle;
    }

    public void Tick(double budgetMs)
    {
        var sw = SysStopwatch.StartNew();
        while (Status == State.Solving)
        {
            double left = budgetMs - sw.Elapsed.TotalMilliseconds;
            if (left <= 0) break;
            if (_sim.BatchActive && !_sim.StepBatch(left)) break;
            Collect();
            if (!_routine.MoveNext()) Complete();
        }
    }

    void Complete()
    {
        Finish();
        Status = State.Done;
    }

    void Finish()
    {
        ElapsedMs = _clock.Elapsed.TotalMilliseconds;
        _clock.Stop();
        if (_sim != null) { _sim.Target = null; _sim.UseCup = false; }
        _routine = null;
    }

    public bool HasCup { get; private set; }
    float _cupMaxY;

    // Whether a path (e.g. the live prediction) ends up in the cup found by the last solve.
    public bool PathHoles(List<Vector3> pts)
    {
        foreach (var p in pts)
        {
            float dx = p.x - _hole.x, dz = p.z - _hole.z;
            if (p.y <= _cupMaxY && dx * dx + dz * dz < CupRadius * CupRadius) return true;
        }
        return false;
    }

    // The hole marker sits well below the green, so "in the cup" is judged against the rim:
    // the median surface height sampled in a ring around the hole in the sim world.
    void ConfigureCup(BallMovement ball)
    {
        var heights = new List<float>();
        for (int i = 0; i < 12; i++)
        {
            float a = i * Mathf.PI / 6f;
            float? y = _sim.SurfaceY(_hole.x + Mathf.Cos(a) * RimSampleRadius, _hole.z + Mathf.Sin(a) * RimSampleRadius, _hole.y + 3f);
            if (y.HasValue) heights.Add(y.Value);
        }
        var sphere = ball.m_sphereCollider;
        float radius = sphere.radius * sphere.transform.lossyScale.x;
        _ballRadius = radius;
        heights.Sort();
        float rim = heights.Count > 0 ? heights[heights.Count / 2] : _hole.y + 0.75f;
        _sim.UseCup = true;
        _sim.CupCenter = _hole;
        _sim.CupRadius = CupRadius;
        _sim.CupMaxY = rim + radius - SinkDepth;
        HasCup = true;
        _cupMaxY = _sim.CupMaxY;
        Plugin.Logger.LogInfo($"Cup: marker y={_hole.y:0.###} rim y={rim:0.###} ({heights.Count}/12 samples) holed when within {CupRadius}m and below y={_sim.CupMaxY:0.###}");
    }

    // ---- batches -------------------------------------------------------------

    void Add(float yaw, float power)
    {
        if (_shots.Count >= SimWorld.BatchSize) return;
        while (_shotPool.Count <= _shots.Count) _shotPool.Add(new SimWorld.BatchShot());
        var s = _shotPool[_shots.Count];
        s.Yaw = Mathf.Repeat(yaw, 360f);
        s.Power = Mathf.Clamp(power, _min, _max);
        s.Force = DirFromYaw(s.Yaw) * (s.Power + 1f);
        _shots.Add(s);
    }

    // Every batch assumes the same launch moment (unless told otherwise) so results stay
    // comparable when obstacles move.
    void Launch(float? launchTime = null, Vector3? start = null)
    {
        _results.Clear();
        _sim.BeginBatch(_ball, _shots, _stepsPerFrame, MaxSimTime, _hole, launchTime ?? _launchTime, start);
        Batches++;
    }

    void Collect()
    {
        if (_shots.Count == 0 || _sim.BatchActive) return;
        foreach (var s in _shots)
            _results.Add(Score(s));
        _all.AddRange(_results);
        _shots.Clear();
    }

    Result Score(SimWorld.BatchShot s)
    {
        Vector3 end = s.Points[s.Points.Count - 1];
        Vector2 hole2 = new(_hole.x, _hole.z);
        bool holed = s.HitTarget;

        float endDist = Vector2.Distance(new Vector2(end.x, end.z), hole2) + 0.5f * Mathf.Abs(end.y - _hole.y);
        float cost = holed ? 0f : 0.7f * endDist + 0.3f * s.MinApproach;
        Evaluations++;

        bool rested = false;
        if (!holed)
        {
            float? ground = _sim.SurfaceY(end.x, end.z, end.y + 0.5f);
            rested = ground.HasValue && end.y - ground.Value < _ballRadius + 0.15f && end.y > _start.y - 20f;
        }

        // Second-shot trials start somewhere else, so they must not replace the first-shot best.
        if (!_stage2 && cost < BestCost)
        {
            BestCost = cost;
            BestHoled = holed;
            BestYaw = s.Yaw;
            BestPower = s.Power;
            BestPoints.Clear();
            BestPoints.AddRange(s.Points);
        }
        return new Result { Yaw = s.Yaw, Power = s.Power, Cost = cost, Holed = holed, End = end, Rested = rested };
    }

    // ---- search --------------------------------------------------------------

    IEnumerator<bool> Run()
    {
        Vector3 toHole = _hole - _start; toHole.y = 0f;
        float yaw0 = YawOf(toHole);
        float range = _max - _min;

        // 1. Straight at the hole: a dense power sweep around the flat-ground estimate plus a
        //    coarse sweep of the full range, all in one batch.
        Phase = "direct";
        float est = EstimatePower(toHole.magnitude);
        for (int i = 0; i < 28; i++) Add(yaw0, est * Mathf.Lerp(0.5f, 1.8f, i / 27f));
        for (int i = 0; i < 12; i++) Add(yaw0, _min + (i + 0.5f) / 12f * range);
        Launch(); while (_sim.BatchActive) yield return true;

        // 2. Coarse grid for banks and blocked lines.
        if (!BestHoled)
        {
            Phase = "grid";
            for (int a = 0; a < GridYaws && !BestHoled; a++)
            {
                foreach (float f in GridPowers)
                    Add(yaw0 + a * 360f / GridYaws, _min + f * range);
                if (_shots.Count + GridPowers.Length > SimWorld.BatchSize || a == GridYaws - 1)
                {
                    Launch(); while (_sim.BatchActive) yield return true;
                }
            }
        }

        // 3. Shrinking local grids around the best few candidates, all candidates per batch.
        if (!BestHoled)
        {
            Phase = "refine";
            float sy = 6f, sp = 0.06f * range;
            for (int round = 0; round < RefineRounds && !BestHoled; round++)
            {
                foreach (var c in TopCandidates(RefineCandidates, sy))
                    for (int dy = -1; dy <= 1; dy++)
                        for (int dp = 0; dp < 4; dp++)
                            Add(c.Yaw + dy * sy, c.Power + (dp - 1.5f) * sp);
                Launch(); while (_sim.BatchActive) yield return true;
                sy *= 0.45f;
                sp *= 0.45f;
            }
        }

        // 3b. No hole-in-one: pick spots where first shots came to rest — closest to the hole with
        //     a clear line first — and look for a sinking second shot from each.
        if (!BestHoled)
        {
            Phase = "two-shot";
            foreach (var leave in LeaveCandidates(TwoShotCandidates))
            {
                Vector3 s2 = leave.End;
                Vector3 to2 = _hole - s2; to2.y = 0f;
                float yaw2 = YawOf(to2), est2 = EstimatePower(to2.magnitude);
                _stage2 = true;
                for (int i = 0; i < 28; i++) Add(yaw2, est2 * Mathf.Lerp(0.5f, 1.8f, i / 27f));
                for (int i = 0; i < 12; i++) Add(yaw2, _min + (i + 0.5f) / 12f * range);
                Launch(start: s2); while (_sim.BatchActive) yield return true;
                _stage2 = false;

                int idx = _results.FindIndex(r => r.Holed);
                if (idx < 0) continue;
                var second = new List<Vector3>(_shotPool[idx].Points);
                TwoShot = true;
                Leave = s2;
                SecondYaw = _results[idx].Yaw;
                SecondPower = _results[idx].Power;

                // Replay the first shot from the real ball so its path can be shown.
                Add(leave.Yaw, leave.Power);
                Launch(); while (_sim.BatchActive) yield return true;
                BestYaw = leave.Yaw;
                BestPower = leave.Power;
                BestPoints.Clear();
                BestPoints.AddRange(_shotPool[0].Points);
                BestPoints.AddRange(second);
                Phase = $"two-shot plan: shot 1 leaves the ball {Vector2.Distance(new Vector2(s2.x, s2.z), new Vector2(_hole.x, _hole.z)):0.0}m from the hole, shot 2 sinks it";
                break;
            }
            if (!TwoShot) Phase = "no hole-in-one or two-shot plan found";
        }

        // 4. Centre a holing shot in the range of aims and powers that still hole, then verify.
        if (BestHoled)
        {
            Phase = "centre";
            float by = BestYaw, bp = BestPower;
            float pStep = 0.008f * range, yStep = 0.2f;
            const int K = 6;
            for (int k = -K; k <= K; k++) Add(by, bp + k * pStep);
            for (int k = -K; k <= K; k++) if (k != 0) Add(by + k * yStep, bp);
            Launch(); while (_sim.BatchActive) yield return true;

            // _results: indices 0..2K are powers (k = -K..K), then yaws without k = 0.
            int pLo = 0, pHi = 0, yLo = 0, yHi = 0;
            while (pLo > -K && _results[K + pLo - 1].Holed) pLo--;
            while (pHi < K && _results[K + pHi + 1].Holed) pHi++;
            int YawIndex(int k) => 2 * K + 1 + (k < 0 ? k + K : k + K - 1);
            while (yLo > -K && _results[YawIndex(yLo - 1)].Holed) yLo--;
            while (yHi < K && _results[YawIndex(yHi + 1)].Holed) yHi++;
            float cp = bp + 0.5f * (pLo + pHi) * pStep;
            float cy = by + 0.5f * (yLo + yHi) * yStep;

            Add(cy, cp);
            Launch(); while (_sim.BatchActive) yield return true;
            if (_results[0].Holed)
            {
                BestYaw = _results[0].Yaw;
                BestPower = _results[0].Power;
                BestPoints.Clear();
                BestPoints.AddRange(_shotPool[0].Points);
            }
            Phase = $"centred (power ±{(pHi - pLo) * pStep / 2f / range * 100f:0.#}%, aim ±{(yHi - yLo) * yStep / 2f:0.##}°)";

            // 5. With moving obstacles the shot only works at certain moments: replay it with the
            //    launch shifted earlier/later to find how wide that window is, and aim for its middle.
            if (_sim.HasMovingObstacles)
            {
                string centred = Phase;
                Phase = "timing";
                var holed = new bool[2 * TimingSteps + 1];
                holed[TimingSteps] = true;
                for (int j = -TimingSteps; j <= TimingSteps; j++)
                {
                    if (j == 0) continue;
                    Add(BestYaw, BestPower);
                    Launch(_launchTime + j * TimingStep); while (_sim.BatchActive) yield return true;
                    holed[j + TimingSteps] = _results[0].Holed;
                }
                int lo = 0, hi = 0;
                while (lo > -TimingSteps && holed[TimingSteps + lo - 1]) lo--;
                while (hi < TimingSteps && holed[TimingSteps + hi + 1]) hi++;
                TargetLaunch = _launchTime + 0.5f * (lo + hi) * TimingStep;
                WindowHalf = 0.5f * (hi - lo) * TimingStep + 0.5f * TimingStep;
                Periods = _sim.PeriodsNear(BestPoints, 1f);
                Timed = true;
                Phase = $"{centred}, timing window ±{WindowHalf:0.###}s, {Periods.Count} obstacles on the path";
            }
        }
    }

    // Rest positions of earlier shots, best second-shot spots first: a clear rolling line to the
    // hole counts far more than distance. Spots closer than 1 m to an already picked one are skipped.
    List<Result> LeaveCandidates(int count)
    {
        var scored = new List<(float score, Result r)>();
        foreach (var r in _all)
        {
            if (!r.Rested) continue;
            float d = Vector2.Distance(new Vector2(r.End.x, r.End.z), new Vector2(_hole.x, _hole.z));
            bool clear = _sim.LineClear(r.End, _hole, _ballRadius, 0.3f);
            scored.Add((clear ? d : d + 100f, r));
        }
        scored.Sort((a, b) => a.score.CompareTo(b.score));
        var picked = new List<Result>();
        foreach (var (_, r) in scored)
        {
            bool near = false;
            foreach (var p in picked)
                if (Vector3.Distance(p.End, r.End) < 1f) { near = true; break; }
            if (near) continue;
            picked.Add(r);
            if (picked.Count == count) break;
        }
        return picked;
    }

    // Best results so far that are at least `minYawGap` apart, so candidates explore different lines.
    List<Result> TopCandidates(int count, float minYawGap)
    {
        _all.Sort((a, b) => a.Cost.CompareTo(b.Cost));
        var picked = new List<Result>();
        foreach (var r in _all)
        {
            bool distinct = true;
            foreach (var p in picked)
                if (Mathf.Abs(Mathf.DeltaAngle(p.Yaw, r.Yaw)) < minYawGap) { distinct = false; break; }
            if (!distinct) continue;
            picked.Add(r);
            if (picked.Count == count) break;
        }
        return picked;
    }

    // Flat-ground guess: drag 0.5 plus sliding friction ≈ 0.026·g, inverted by bisection.
    float EstimatePower(float distance)
    {
        float dt = Time.fixedDeltaTime, g = -Physics.gravity.y;
        float Travel(float v0)
        {
            float v = v0, d = 0f;
            for (int i = 0; i < 4000 && v > 0.1f; i++) { v = (v - 0.026f * g * dt) * (1f - 0.5f * dt); d += v * dt; }
            return d;
        }
        float lo = 0f, hi = (_max + 1f) * dt;
        for (int i = 0; i < 30; i++)
        {
            float mid = 0.5f * (lo + hi);
            if (Travel(mid) < distance) lo = mid; else hi = mid;
        }
        return Mathf.Clamp(0.5f * (lo + hi) / dt - 1f, _min, _max);
    }
}

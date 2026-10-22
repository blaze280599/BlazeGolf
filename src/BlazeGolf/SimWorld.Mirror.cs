using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

namespace BlazeGolf;

// Keeps the sim world's copy of the course in step with the game: colliders that appear later
// (placed ramps, spawned props) are cloned, ones that disappear are removed, and anything that
// moves is tracked. Movers' poses are recorded every frame so their future motion can be
// replayed inside a simulation — by repeating their cycle when they are periodic (windmills,
// sliders, lifts), otherwise by extrapolating their current velocity.
public partial class SimWorld
{
    const float SyncInterval = 1f;
    const float MoveEpsilon = 0.001f;
    const float RotateEpsilon = 0.05f;
    const float SampleInterval = 1f / 60f;
    const int HistoryCapacity = 1500;          // ~25 s at 60 Hz
    const float ResampleRate = 20f;
    const float PeriodCheckInterval = 1f;
    const float MinPeriod = 0.3f;
    const float PeriodWindow = 1f;
    const float PeriodMaxError = 0.03f;        // metres (+1 cm per degree of rotation)
    const float MaxExtrapolation = 1f;

    class Entry
    {
        public Collider Src;
        public Transform SrcT;
        public Transform CloneT;
        public Rigidbody CloneRb;
        public Collider CloneCol;
        public Vector3 LastPos;
        public Quaternion LastRot;
        public Track Track;
    }

    class Track
    {
        public readonly float[] T = new float[HistoryCapacity];
        public readonly Vector3[] P = new Vector3[HistoryCapacity];
        public readonly Quaternion[] R = new Quaternion[HistoryCapacity];
        public int Start, Count;
        public float Period;              // 0 = not periodic (yet)
        public float NextPeriodCheck;
        public bool Moving;               // has moved within the recorded history

        public int Idx(int i) => (Start + i) % HistoryCapacity;   // 0 = oldest
        public float Newest => T[Idx(Count - 1)];
        public float Oldest => T[Idx(0)];

        public void Add(float t, Vector3 p, Quaternion r)
        {
            int i;
            if (Count < HistoryCapacity) i = Idx(Count++);
            else { i = Start; Start = (Start + 1) % HistoryCapacity; }
            T[i] = t; P[i] = p; R[i] = r;
        }
    }

    // Key: native pointer of the source collider. Value null = seen but not cloneable.
    readonly Dictionary<IntPtr, Entry> _entries = new();
    readonly Dictionary<IntPtr, bool> _isBallCollider = new();
    readonly Dictionary<int, bool> _isSimScene = new();
    readonly List<Entry> _moving = new();
    readonly HashSet<IntPtr> _seen = new();
    readonly List<IntPtr> _gone = new();
    float _nextSync;
    float _launchTime;

    public int MoverCount => _moving.Count;
    public bool HasMovingObstacles { get; private set; }

    void ClearMirror()
    {
        _entries.Clear();
        _isBallCollider.Clear();
        _isSimScene.Clear();
        _moving.Clear();
        _nextSync = 0f;
        HasMovingObstacles = false;
    }

    bool Cloneable(Collider c)
    {
        if (c == null || !c.enabled || c.isTrigger) return false;

        // FindObjectsOfType sees every loaded scene, including our own sim scenes (an old one
        // can still be unloading during a rebuild). Mirroring those would copy the copies.
        int sceneHandle = c.gameObject.scene.handle;
        if (!_isSimScene.TryGetValue(sceneHandle, out bool isSim))
        {
            isSim = c.gameObject.scene.name.StartsWith(SimScenePrefix);
            _isSimScene[sceneHandle] = isSim;
        }
        if (isSim) return false;

        var key = c.Pointer;
        if (!_isBallCollider.TryGetValue(key, out bool isBall))
        {
            isBall = c.GetComponentInParent<BallMovement>() != null;
            _isBallCollider[key] = isBall;
        }
        return !isBall;
    }

    // Brings the clone set up to date. Cheap enough to run every second: new and vanished
    // colliders are handled, and any clone whose source has moved becomes a tracked mover.
    public void SyncColliders(bool force)
    {
        if (!force && Time.time < _nextSync) return;
        _nextSync = Time.time + SyncInterval;

        int added = 0, removed = 0, promoted = 0;
        _seen.Clear();
        foreach (var src in Object.FindObjectsOfType<Collider>())
        {
            if (!Cloneable(src)) continue;
            var key = src.Pointer;
            _seen.Add(key);
            if (!_entries.TryGetValue(key, out var e))
            {
                try
                {
                    e = CloneCollider(src);
                    _entries[key] = e;
                    if (e != null)
                    {
                        added++;
                        // Anything with a Rigidbody can move (kinematic animation or physics props).
                        if (src.attachedRigidbody != null) { Promote(e); promoted++; }
                    }
                }
                catch (Exception ex)
                {
                    _entries[key] = null;
                    Plugin.Logger.LogWarning($"Clone failed for {src.name}: {ex.Message}");
                }
                continue;
            }
            if (e == null || e.Track != null) continue;

            Vector3 p = e.SrcT.position;
            Quaternion r = e.SrcT.rotation;
            if (Dist(p, e.LastPos) > MoveEpsilon || AngleDeg(r, e.LastRot) > RotateEpsilon)
            {
                Promote(e);
                promoted++;
            }
        }

        _gone.Clear();
        foreach (var kv in _entries)
            if (!_seen.Contains(kv.Key)) _gone.Add(kv.Key);
        foreach (var key in _gone)
        {
            var e = _entries[key];
            _entries.Remove(key);
            _isBallCollider.Remove(key);
            if (e == null) continue;
            removed++;
            _moving.Remove(e);
            if (e.CloneT != null) Object.Destroy(e.CloneT.gameObject);
        }

        ColliderCount = 0;
        foreach (var e in _entries.Values) if (e != null) ColliderCount++;
        if (added + removed + promoted > 0)
            Plugin.Logger.LogInfo($"Sim world sync: +{added} -{removed} colliders, {promoted} new movers ({ColliderCount} cloned, {_moving.Count} movers)");
    }

    void Promote(Entry e)
    {
        if (e.CloneRb == null)
        {
            e.CloneRb = e.CloneT.gameObject.AddComponent<Rigidbody>();
            e.CloneRb.isKinematic = true;
            e.CloneRb.interpolation = RigidbodyInterpolation.None;
        }
        // Stagger analysis so many movers promoted together don't all run on the same frame.
        e.Track = new Track { NextPeriodCheck = Time.time + 2f + (_moving.Count % 30) * (PeriodCheckInterval / 30f) };
        _moving.Add(e);
    }

    Entry CloneCollider(Collider src)
    {
        var st = src.transform;
        var go = new GameObject("sim_" + src.name);
        go.layer = src.gameObject.layer;
        SceneManager.MoveGameObjectToScene(go, _scene);
        go.transform.SetPositionAndRotation(st.position, st.rotation);
        go.transform.localScale = st.lossyScale;

        Collider dst;
        switch (src.GetIl2CppType().Name)
        {
            case "BoxCollider":
            {
                var s = src.Cast<BoxCollider>(); var d = go.AddComponent<BoxCollider>();
                d.center = s.center; d.size = s.size; dst = d; break;
            }
            case "SphereCollider":
            {
                var s = src.Cast<SphereCollider>(); var d = go.AddComponent<SphereCollider>();
                d.center = s.center; d.radius = s.radius; dst = d; break;
            }
            case "CapsuleCollider":
            {
                var s = src.Cast<CapsuleCollider>(); var d = go.AddComponent<CapsuleCollider>();
                d.center = s.center; d.radius = s.radius; d.height = s.height; d.direction = s.direction; dst = d; break;
            }
            case "MeshCollider":
            {
                var s = src.Cast<MeshCollider>();
                if (s.sharedMesh == null) { Object.Destroy(go); return null; }
                var d = go.AddComponent<MeshCollider>();
                d.cookingOptions = s.cookingOptions; d.convex = s.convex; d.sharedMesh = s.sharedMesh; dst = d; break;
            }
            case "TerrainCollider":
            {
                var s = src.Cast<TerrainCollider>(); var d = go.AddComponent<TerrainCollider>();
                d.terrainData = s.terrainData; dst = d; break;
            }
            default:
                Object.Destroy(go);
                return null;
        }

        dst.sharedMaterial = src.sharedMaterial;
        dst.contactOffset = src.contactOffset;
        return new Entry { Src = src, SrcT = st, CloneT = go.transform, CloneCol = dst, LastPos = st.position, LastRot = st.rotation };
    }

    // ---- plain math (avoids interop calls in hot loops) ------------------------

    static float Dist(Vector3 a, Vector3 b)
    {
        float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
        return MathF.Sqrt(x * x + y * y + z * z);
    }

    static float AngleDeg(Quaternion a, Quaternion b)
    {
        float dot = MathF.Abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
        return dot >= 1f ? 0f : 2f * MathF.Acos(dot) * (180f / MathF.PI);
    }

    static Vector3 Lerp(Vector3 a, Vector3 b, float t) =>
        new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);

    // Normalised lerp: indistinguishable from slerp for the small steps between samples.
    static Quaternion Nlerp(Quaternion a, Quaternion b, float t)
    {
        float sign = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0f ? -1f : 1f;
        float x = a.x + (sign * b.x - a.x) * t, y = a.y + (sign * b.y - a.y) * t;
        float z = a.z + (sign * b.z - a.z) * t, w = a.w + (sign * b.w - a.w) * t;
        float n = MathF.Sqrt(x * x + y * y + z * z + w * w);
        return new Quaternion(x / n, y / n, z / n, w / n);
    }

    // ---- motion recording ----------------------------------------------------

    // Call once per frame: records movers' poses and refreshes their period estimates.
    public void SampleMovers()
    {
        float now = Time.time;
        bool anyMoving = false;
        foreach (var e in _moving)
        {
            if (e.SrcT == null || e.Src.WasCollected) continue;
            var tr = e.Track;
            if (tr.Count > 0 && now - tr.Newest < SampleInterval) { anyMoving |= tr.Moving; continue; }
            Vector3 p = e.SrcT.position;
            Quaternion r = e.SrcT.rotation;
            tr.Add(now, p, r);
            e.LastPos = p;
            e.LastRot = r;
            if (now >= tr.NextPeriodCheck)
            {
                tr.NextPeriodCheck = now + PeriodCheckInterval;
                DetectPeriod(tr);
            }
            anyMoving |= tr.Moving;
        }
        HasMovingObstacles = anyMoving;
    }

    static void Interp(Track tr, float t, out Vector3 p, out Quaternion r)
    {
        int lo = 0, hi = tr.Count - 1;
        if (t <= tr.Oldest) { p = tr.P[tr.Idx(0)]; r = tr.R[tr.Idx(0)]; return; }
        if (t >= tr.Newest) { p = tr.P[tr.Idx(hi)]; r = tr.R[tr.Idx(hi)]; return; }
        while (hi - lo > 1)
        {
            int mid = (lo + hi) / 2;
            if (tr.T[tr.Idx(mid)] <= t) lo = mid; else hi = mid;
        }
        int a = tr.Idx(lo), b = tr.Idx(hi);
        float span = tr.T[b] - tr.T[a];
        float f = span > 0f ? (t - tr.T[a]) / span : 0f;
        p = Lerp(tr.P[a], tr.P[b], f);
        r = Nlerp(tr.R[a], tr.R[b], f);
    }

    // Finds the shortest lag at which the last second of motion repeats.
    static void DetectPeriod(Track tr)
    {
        tr.Moving = false;
        if (tr.Count < 2) return;
        float span = tr.Newest - tr.Oldest;
        int n = Mathf.FloorToInt(span * ResampleRate);
        int w = Mathf.RoundToInt(PeriodWindow * ResampleRate);
        if (n < 4) return;

        var p = new Vector3[n];
        var r = new Quaternion[n];
        float maxMove = 0f;
        for (int k = 0; k < n; k++)          // k = 0 is the newest sample
        {
            Interp(tr, tr.Newest - k / ResampleRate, out p[k], out r[k]);
            maxMove = MathF.Max(maxMove, Dist(p[k], p[0]) + 0.01f * AngleDeg(r[k], r[0]));
        }
        tr.Moving = maxMove > MoveEpsilon * 5f;
        if (!tr.Moving || n < w + Mathf.RoundToInt(MinPeriod * ResampleRate) + 2) { tr.Period = 0f; return; }

        int minLag = Mathf.RoundToInt(MinPeriod * ResampleRate);
        int maxLag = n - w - 1;
        var err = new float[maxLag + 2];
        float best = float.MaxValue;
        for (int lag = minLag; lag <= maxLag; lag++)
        {
            float sum = 0f;
            for (int k = 0; k < w; k++)
                sum += Dist(p[k], p[k + lag]) + 0.01f * AngleDeg(r[k], r[k + lag]);
            err[lag] = sum / w;
            best = Mathf.Min(best, err[lag]);
        }
        if (best > PeriodMaxError) { tr.Period = 0f; return; }

        // Shortest lag close to the best one (avoids picking a multiple of the period),
        // refined to sub-sample precision with a parabola through its neighbours.
        for (int lag = minLag; lag <= maxLag; lag++)
        {
            if (err[lag] > Mathf.Max(1.5f * best, 0.005f)) continue;
            while (lag + 1 <= maxLag && err[lag + 1] < err[lag]) lag++;
            float offset = 0f;
            if (lag > minLag && lag < maxLag)
            {
                float a = err[lag - 1], b = err[lag], c = err[lag + 1];
                float denom = a - 2f * b + c;
                if (denom > 1e-6f) offset = Mathf.Clamp(0.5f * (a - c) / denom, -0.5f, 0.5f);
            }
            tr.Period = (lag + offset) / ResampleRate;
            return;
        }
    }

    // Pose of a mover at absolute game time `t` (may be in the future).
    static void PoseAt(Track tr, float t, out Vector3 p, out Quaternion r)
    {
        float newest = tr.Newest;
        if (!tr.Moving || t <= newest) { Interp(tr, Mathf.Min(t, newest), out p, out r); return; }

        if (tr.Period > 0f)
        {
            float back = tr.Period * Mathf.Ceil((t - newest) / tr.Period);
            if (t - back >= tr.Oldest) { Interp(tr, t - back, out p, out r); return; }
        }

        // Not periodic (or not enough history): carry on with the current velocity for a while.
        Interp(tr, newest, out var p1, out var r1);
        Interp(tr, newest - 0.1f, out var p0, out var r0);
        float dt = Mathf.Min(t - newest, MaxExtrapolation);
        p = p1 + (p1 - p0) / 0.1f * dt;
        (r1 * Quaternion.Inverse(r0)).ToAngleAxis(out float angle, out Vector3 axis);
        if (angle > 180f) angle -= 360f;
        r = Quaternion.AngleAxis(angle / 0.1f * dt, axis) * r1;
    }

    // ---- driving clones during a simulation -----------------------------------

    // Places every mover where it will be at `launchTime` (the moment the shot is assumed fired).
    void PrepareMovers(float launchTime)
    {
        _launchTime = launchTime;
        foreach (var e in _moving)
        {
            if (e.CloneT == null || e.Track.Count == 0) continue;
            PoseAt(e.Track, launchTime, out var p, out var r);
            e.CloneRb.position = p;
            e.CloneRb.rotation = r;
            e.CloneT.SetPositionAndRotation(p, r);
        }
    }

    // Moves movers to their pose `simTime` seconds after launch, via MovePosition so PhysX
    // gives contacts the obstacle's velocity (a windmill blade pushes the ball).
    void StepMovers(float simTime)
    {
        foreach (var e in _moving)
        {
            if (e.CloneRb == null || !e.Track.Moving || e.Track.Count == 0) continue;
            PoseAt(e.Track, _launchTime + simTime, out var p, out var r);
            e.CloneRb.MovePosition(p);
            e.CloneRb.MoveRotation(r);
        }
    }

    // Periods (0 = not periodic) of moving obstacles the path passes near. A rotating part sweeps
    // roughly its own extent around its centre, so that is the reach used.
    public List<float> PeriodsNear(List<Vector3> path, float margin)
    {
        var periods = new List<float>();
        foreach (var e in _moving)
        {
            if (!e.Track.Moving || e.CloneCol == null) continue;
            var b = e.CloneCol.bounds;
            float reach = b.extents.magnitude + margin;
            for (int i = 0; i < path.Count; i += 5)
                if (Dist(path[i], b.center) < reach)
                {
                    periods.Add(e.Track.Period);
                    Plugin.Logger.LogInfo($"  obstacle on path: {e.CloneT.name} period={(e.Track.Period > 0f ? $"{e.Track.Period:0.000}s" : "aperiodic")}");
                    break;
                }
        }
        return periods;
    }

    public string DescribeMovers()
    {
        var parts = new List<string>();
        foreach (var e in _moving)
            if (e.Track.Moving)
                parts.Add($"{e.CloneT.name}:{(e.Track.Period > 0f ? $"{e.Track.Period:0.00}s" : "aperiodic")}");
        return parts.Count == 0 ? "none moving" : string.Join(", ", parts);
    }
}

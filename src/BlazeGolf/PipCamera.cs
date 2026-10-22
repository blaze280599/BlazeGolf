using UnityEngine;

namespace BlazeGolf;

// Picture-in-picture view of the predicted resting spot, drawn in the top-right corner.
// It is a second base camera with a small viewport rect and a higher depth than the game
// camera, so URP renders it on top of the main view.
public class PipCamera
{
    static readonly Rect Viewport = new(0.73f, 0.64f, 0.25f, 0.33f);
    const float Height = 6f;          // above the end point
    const float BackOff = 4f;         // behind it, along the shot direction
    const float Smoothing = 10f;      // higher = snappier follow

    Camera _cam;
    Vector3 _pos;
    bool _placed;

    public bool Enabled { get; set; } = true;

    public void Show(Vector3 end, Vector3 from)
    {
        if (!Enabled) { Hide(); return; }
        if (!Ensure()) return;

        // Look down at the end point from behind and above, relative to how the ball arrives.
        Vector3 flat = end - from;
        flat.y = 0f;
        Vector3 back = flat.sqrMagnitude > 1e-4f ? -flat.normalized : Vector3.back;
        Vector3 side = Vector3.Cross(Vector3.up, back);

        // Prefer the view from behind; if a wall or roof blocks it, use whichever other angle has
        // the clearest line, pulling in just short of any obstruction. This stops the camera
        // from ending up pressed against a wall next to the end point.
        Vector3 origin = end + Vector3.up * 0.2f;
        Vector3 want = origin;
        float bestClear = -1f;
        foreach (var dir in new[] { back, -side, side, -back, Vector3.zero })
        {
            Vector3 offset = Vector3.up * Height + dir * BackOff;
            float length = offset.magnitude;
            float clear = Physics.Raycast(origin, offset / length, out var hit, length, ~((1 << 21) | (1 << 22)), QueryTriggerInteraction.Ignore)
                ? hit.distance - 0.3f : length;
            if (clear > bestClear) { bestClear = clear; want = origin + offset / length * Mathf.Max(0.5f, clear); }
            if (clear >= length) break;
        }

        _pos = _placed ? Vector3.Lerp(_pos, want, 1f - Mathf.Exp(-Smoothing * Time.unscaledDeltaTime)) : want;
        _placed = true;
        _cam.transform.position = _pos;
        _cam.transform.LookAt(end);
        _cam.enabled = true;
    }

    public void Hide()
    {
        if (_cam != null) _cam.enabled = false;
        _placed = false;
    }

    bool Ensure()
    {
        if (_cam != null) return true;
        var main = Camera.main;
        if (main == null) return false;

        var go = new GameObject("BlazeGolf_PipCamera");
        Object.DontDestroyOnLoad(go);
        _cam = go.AddComponent<Camera>();
        _cam.rect = Viewport;
        _cam.depth = main.depth + 10f;
        _cam.cullingMask = main.cullingMask;
        _cam.clearFlags = main.clearFlags;
        _cam.backgroundColor = main.backgroundColor;
        _cam.fieldOfView = 55f;
        _cam.nearClipPlane = 0.05f;
        _cam.farClipPlane = 300f;
        _cam.allowHDR = main.allowHDR;
        _cam.allowMSAA = false;
        _cam.enabled = false;
        Plugin.Logger.LogInfo($"PiP camera created (main camera '{main.name}', depth {main.depth})");
        return true;
    }
}

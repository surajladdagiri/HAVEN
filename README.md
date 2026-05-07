# HAVEN

## Desktop viewer

The iPhone app now exposes a live viewer stream for the AR mesh and full camera
pose. On the AR screen, the app shows a status line like:

`Waiting for Viewer @ 192.168.1.42:8080`

Use that IP address from your Mac while both devices are on the same network.

### Live phone stream

```bash
python3.11 scripts/iphone_scene_viewer.py --host 192.168.1.42
```

Notes:

- `python3.11` is preferred because it already has `open3d` installed here, so
  the viewer opens as a real interactive 3D point-cloud window by default.
- The default `Open3D` view now uses a chase-camera follow mode so the phone
  marker and nearby geometry stay visible while you demo.
- The desktop camera follows the phone pose, and the rendered "camera" marker
  shows the phone's current position and facing direction.
- The terminal now prints live packet / point-cloud stats every couple seconds,
  which helps tell the difference between a rendering issue and a stream issue.
- Use `--backend matplotlib` if you want the lighter fallback viewer instead.
- Use `--follow-mode first_person` if you want the desktop view to try to match
  the phone camera more directly.

### Local demo mode

```bash
python3.11 scripts/iphone_scene_viewer.py --demo
```

### Headless snapshot test

```bash
python3.11 scripts/iphone_scene_viewer.py \
  --demo \
  --snapshot /tmp/haven_viewer_demo.png
```

"""Record a trial to image frames from a fixed camera.

Isaac Sim only: imports omni.replicator. Never import this from a ROS 2
node.

Frames are taken on the SIMULATION clock, `fps` per simulated second,
and encoded at the same rate afterwards, so the video plays at
simulated real time however slowly Isaac Sim actually ran. Capturing on
wall-clock time would speed up or slow down the video with the frame
rate of the machine.

Only the camera is added. It has no collider and is not seen by the
LiDAR, so recording does not change what the robot perceives. A scene
may also ask for prims to be hidden while recording -- the factory
hides its ceiling, or the camera films the roof -- and `hide_prims`
explains why that is safe too. It does
cost render time; Nav2 runs on wall-clock time in this project, so check
a recorded trial's result against an unrecorded one before relying on it.

Writing happens on a background thread. Encoding JPEG on the simulation
thread stalled every step by the time it took.

Optional, all off unless a config asks (paper/demo captures):

    recording.label: false        no burned-in "<SCENARIO> t = ..." label
    recording.extra_cameras:      more cameras in the same simulation, each
      - name: CAM01_TOP           written to <out_dir>/<name>/frame_*.jpg
        ortho: {centre: [x, y], width_m: 30}           (orthographic, looking down)
      - name: CAM02_ISOMETRIC
        position: [..]; look_at: [..]; focal_length: 20  (perspective)
    a `snapshot` callable           maybe_capture(t, snapshot) appends one
                                    JSON line per captured frame to
                                    <out_dir>/frames.jsonl (sim time and
                                    whatever the callable returns), so
                                    overlays can be drawn afterwards at the
                                    exact frame.

Every extra camera is rendered on every step, so it costs simulation
speed like the main one does.
"""

from __future__ import annotations

import json
import queue
import threading
from pathlib import Path
from typing import Any, Dict

import numpy as np


def hide_prims(paths) -> list:
    """Take the lid off the building so the camera can see inside.

    The recording camera looks down from 17 m. The bay has a ceiling at
    5 m, so without this every factory video is a flat white rectangle --
    which is exactly what the first factory recordings were.

    This cannot change what the robot perceives, which is the rule the
    rest of this file is built around. The LiDAR sits at 0.31 m and
    reaches 10 degrees above horizontal out to 20 m, so the highest
    point it can return is 3.84 m: below the 4.55 m light fittings and
    the 5 m deck. Nothing hidden here was ever in a scan.

    Returns the paths actually hidden, so the log says what went.
    """
    from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
    from pxr import UsdGeom

    gone = []
    for path in paths:
        if not is_prim_path_valid(path):
            continue
        imageable = UsdGeom.Imageable(get_prim_at_path(path))
        if imageable:
            # Only works on a typed prim; the scene builder makes these
            # groups Xforms for exactly this reason.
            imageable.MakeInvisible()
            gone.append(path)
    return gone


class FrameRecorder:
    def __init__(self, cfg: Dict[str, Any], out_dir: Path, label: str):
        import omni.replicator.core as rep

        rec = cfg["recording"]
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.label = label
        self.interval = 1.0 / float(rec["fps"])
        self.quality = int(rec.get("jpeg_quality", 90))
        self.start = float(rec.get("start_sim_time",
                                   cfg["navigation"].get("trial_start_sim_time", 0.0)))
        self.trial_start = float(cfg["navigation"].get("trial_start_sim_time", 0.0))
        width, height = (int(v) for v in rec["resolution"])

        hidden = hide_prims(rec.get("hide", []))
        if hidden:
            print(f"  recording: hid {', '.join(hidden)}", flush=True)

        self.draw_label = bool(rec.get("label", True))
        self.views = [(self.out_dir, self._camera(rep, rec["camera"], None, width, height))]
        for extra in rec.get("extra_cameras") or []:
            sub = self.out_dir / str(extra["name"])
            sub.mkdir(parents=True, exist_ok=True)
            self.views.append((sub, self._camera(rep, extra, str(extra["name"]), width, height)))
        # kept for callers that used the single camera directly
        self.render_product, self.annotator = self.views[0][1]
        self._log = None

        self.next_time = self.start
        self.count = 0
        self._queue: "queue.Queue" = queue.Queue(maxsize=64)
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()

    @staticmethod
    def _camera(rep, spec, name, width, height):
        if spec.get("ortho"):
            # An unrotated USD camera looks down -z with +y up the image;
            # aperture in tenths of a scene unit, as isaac_sim/scripts/paper_shots.py.
            from isaacsim.core.utils.stage import get_current_stage
            from pxr import Gf, UsdGeom
            path = f"/World/RecordingOrtho_{name or 'main'}"
            cam = UsdGeom.Camera.Define(get_current_stage(), path)
            cam.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
            w = float(spec["ortho"]["width_m"])
            cam.CreateHorizontalApertureAttr(w * 10.0)
            cam.CreateVerticalApertureAttr(w * height / width * 10.0)
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200.0))
            xf = UsdGeom.Xformable(cam.GetPrim())
            xf.ClearXformOpOrder()
            cx, cy = spec["ortho"].get("centre", [0.0, 0.0])
            xf.AddTranslateOp().Set(Gf.Vec3d(float(cx), float(cy), float(spec["ortho"].get("z", 30.0))))
            product = rep.create.render_product(path, (width, height))
        else:
            cam = rep.create.camera(
                position=tuple(float(v) for v in spec["position"]),
                look_at=tuple(float(v) for v in spec["look_at"]),
                focal_length=float(spec.get("focal_length", 24.0)),
            )
            product = rep.create.render_product(cam, (width, height))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach(product)
        return product, annotator

    def maybe_capture(self, sim_time: float, snapshot=None) -> None:
        if sim_time + 1e-9 < self.next_time:
            return
        frames = []
        for out, (_product, annotator) in self.views:
            data = annotator.get_data()
            if data is None or np.asarray(data).size == 0:
                return
            frames.append((out, np.array(np.asarray(data)[..., :3], copy=True)))
        for out, frame in frames:
            self._queue.put((out, self.count, sim_time, frame))
        if snapshot is not None:
            if self._log is None:
                self._log = open(self.out_dir / "frames.jsonl", "a")
            try:
                extra = snapshot()
            except Exception as exc:  # a broken overlay log must never stop a trial
                extra = {"error": str(exc)}
            self._log.write(json.dumps({"index": self.count, "sim_time": round(sim_time, 4),
                                        "t": round(sim_time - self.trial_start, 4), **extra}) + "\n")
            self._log.flush()
        self.count += 1
        # Stay on the grid rather than drifting by whatever step the
        # capture happened to land on.
        while self.next_time <= sim_time:
            self.next_time += self.interval

    def _write_loop(self) -> None:
        from PIL import Image, ImageDraw, ImageFont
        try:
            font = ImageFont.load_default(size=28)
        except TypeError:
            font = ImageFont.load_default()
        while True:
            out, index, sim_time, frame = self._queue.get()
            image = Image.fromarray(frame)
            if self.draw_label:
                draw = ImageDraw.Draw(image)
                text = f"{self.label}   t = {sim_time - self.trial_start:6.1f} s"
                draw.rectangle((10, 10, 20 + draw.textlength(text, font=font), 52),
                               fill=(0, 0, 0))
                draw.text((16, 14), text, fill=(255, 255, 255), font=font)
            image.save(out / f"frame_{index:05d}.jpg", quality=self.quality)
            self._queue.task_done()

    def flush(self) -> None:
        self._queue.join()

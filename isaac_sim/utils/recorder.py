"""Record a trial to image frames from a fixed camera.

Isaac Sim only: imports omni.replicator. Never import this from a ROS 2
node.

Frames are taken on the SIMULATION clock, `fps` per simulated second,
and encoded at the same rate afterwards, so the video plays at
simulated real time however slowly Isaac Sim actually ran. Capturing on
wall-clock time would speed up or slow down the video with the frame
rate of the machine.

Only the camera is added. It has no collider and is not seen by the
LiDAR, so recording does not change what the robot perceives. It does
cost render time; Nav2 runs on wall-clock time in this project, so check
a recorded trial's result against an unrecorded one before relying on it.

Writing happens on a background thread. Encoding JPEG on the simulation
thread stalled every step by the time it took.
"""

from __future__ import annotations

import queue
import threading
from pathlib import Path
from typing import Any, Dict

import numpy as np


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

        cam = rep.create.camera(
            position=tuple(float(v) for v in rec["camera"]["position"]),
            look_at=tuple(float(v) for v in rec["camera"]["look_at"]),
            focal_length=float(rec["camera"].get("focal_length", 24.0)),
        )
        self.render_product = rep.create.render_product(cam, (width, height))
        self.annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotator.attach(self.render_product)

        self.next_time = self.start
        self.count = 0
        self._queue: "queue.Queue" = queue.Queue(maxsize=64)
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()

    def maybe_capture(self, sim_time: float) -> None:
        if sim_time + 1e-9 < self.next_time:
            return
        data = self.annotator.get_data()
        if data is None or np.asarray(data).size == 0:
            return
        frame = np.array(np.asarray(data)[..., :3], copy=True)
        self._queue.put((self.count, sim_time, frame))
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
            index, sim_time, frame = self._queue.get()
            image = Image.fromarray(frame)
            draw = ImageDraw.Draw(image)
            text = f"{self.label}   t = {sim_time - self.trial_start:6.1f} s"
            draw.rectangle((10, 10, 20 + draw.textlength(text, font=font), 52),
                           fill=(0, 0, 0))
            draw.text((16, 14), text, fill=(255, 255, 255), font=font)
            image.save(self.out_dir / f"frame_{index:05d}.jpg", quality=self.quality)
            self._queue.task_done()

    def flush(self) -> None:
        self._queue.join()

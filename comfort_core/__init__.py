"""Shared, framework-free logic for the Comfort_AMR experiments.

Everything in this package must import cleanly from BOTH interpreters:

  * Isaac Sim's bundled Python 3.11  (has omni, pxr; has no rclpy)
  * the system Python 3.12          (has rclpy, nav2; has no omni)

So: standard library, numpy and PyYAML only. No omni, no pxr, no rclpy.
See docs/SETUP.md section 2.
"""

from comfort_core.config import (load_config, nav2_map, project_root,
                                 resolve_path)
from comfort_core.frames import odom_to_world, world_to_odom, wrap_to_pi

__all__ = [
    "nav2_map",
    "load_config",
    "project_root",
    "resolve_path",
    "odom_to_world",
    "world_to_odom",
    "wrap_to_pi",
]

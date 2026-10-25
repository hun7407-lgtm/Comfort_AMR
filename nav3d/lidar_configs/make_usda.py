#!/usr/bin/env python3
"""Turn the measured L1 profile into an Isaac Sim sensor definition.

    ./nav3d/lidar_configs/make_usda.py

Isaac Sim 5.1 defines an RTX LiDAR as a USD prim, not as one of the
JSON files in the extension's data folder. Passing `config="Unitree_L1"`
to IsaacSensorCreateRtxLidar is accepted and then ignored, and editing
the attributes of a sensor that already exists crashes the renderer --
the plugin caches the emitter arrays at creation and re-reads only the
count, so it aborts on "Malformed model parameter length". The
supported route is to reference a .usda, which is what Isaac's own
Example_Rotary is.

Every per-emitter array has to be the same length as numberOfEmitters.
There are ten of them, and leaving even one at its old length is the
crash above.

channelId is 1-based. Numbering the emitters from 0, as everything else
in this project does, makes the whole sensor return nothing at all --
no points, no warning, no error in the log. It cost an afternoon to
find by bisecting against Isaac's own Example_Rotary, so: 1..n.
"""

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROFILE = HERE / "Unitree_L1.json"
OUT = HERE / "Unitree_L1.usda"


def arr(values, fmt="{:.4f}"):
    return "[" + ", ".join(fmt.format(v) if isinstance(v, float) else str(v)
                           for v in values) + "]"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--azimuth-density", type=int, default=4,
                        help="multiply the number of azimuth columns a "
                             "revolution by this. 1 is the real sensor's own "
                             "rate; the default stands in for its "
                             "non-repetitive scanning.")
    args = parser.parse_args()
    spec = json.loads(PROFILE.read_text(encoding="utf-8"))["profile"]
    n = int(spec["numberOfEmitters"])
    states = spec["emitterStates"]

    # EVERY emitter state, not just the first.
    #
    # The first version of this took emitterStates[0] and ignored the
    # other three. That quarters the sensor's horizontal resolution:
    # one state fires all 18 beams at the same bearing, so with 33
    # report positions a revolution the columns stand 10.9 degrees
    # apart, which is 0.76 m at 4 m. A person is then hit by exactly
    # one column and comes back as a vertical line 0.03 m wide --
    # measured, and it made the person detector reject every real
    # person as a sliver of wall while the height and extent were
    # perfect (top 1.55 m, extent 1.07 m).
    #
    # The four states are the real sensor's: the same 18 elevations
    # offset by 0, 2.727, 5.455 and 8.182 degrees of azimuth, which is
    # how the L1 fills in between its columns.
    for i, state in enumerate(states, start=1):
        for name in ("elevationDeg", "azimuthDeg", "fireTimeNs"):
            if len(state[name]) != n:
                print(f"[ERROR] state s{i:03d} {name} has {len(state[name])} "
                      f"entries, numberOfEmitters is {n}", file=sys.stderr)
                return 1

    # How many azimuth columns a revolution, as a multiple of the real
    # sensor's.
    #
    # The L1 is NON-REPETITIVE: every revolution paints different
    # bearings, so over a second or two it fills in between its columns
    # and a person comes back 0.66 m wide. A USD sensor profile is a
    # fixed grid -- the same bearings every revolution -- so stacking
    # sweeps in simulation adds points but no new directions.
    #
    # At the real sensor's 330 reports/s and 10 Hz that is 33 columns a
    # revolution, 10.9 degrees apart, which is 0.76 m at 4 m: a person
    # is hit by exactly one column and measures 0.03 m wide. Measured
    # twice, and it made the detector that runs on the real robot
    # reject every person in the scene as a sliver of wall while their
    # height and vertical extent were perfect (top 1.55 m, extent
    # 1.07 m). The four emitter states do not help -- they offset each
    # column by a couple of degrees rather than adding columns.
    #
    # Multiplying the report rate gives the fixed grid the angular
    # density the real sensor reaches by moving. It is a different
    # trade than the real sensor makes -- more directions every sweep
    # instead of different directions each sweep -- and it is the one
    # that keeps the SHAPE of a person right, which is what the
    # detector reads. Points per sweep stop matching the real
    # sensor's 593 as a result; points per second over the detector's
    # accumulation window are what match.
    density = int(args.azimuth_density)
    report_rate = int(spec["reportRateBaseHz"]) * density

    def state_block(index: int, state) -> str:
        tag = f"s{index:03d}"
        els = [float(v) for v in state["elevationDeg"]]
        azs = [float(v) for v in state["azimuthDeg"]]
        # Every emitter has to fire inside one report period, and the
        # period shrinks with the density. Isaac checks this and
        # refuses the whole profile otherwise -- "maxFireTime 2861952
        # is greater than reportRateBaseHz converted to nanoseconds
        # 757575", after which it quietly substitutes a dummy and the
        # sensor publishes nothing at all.
        fire = [int(v) // density for v in state["fireTimeNs"]]
        lines = [
            f'    float[] omni:sensor:Core:emitterState:{tag}:azimuthDeg = {arr(azs)}',
            f'    uint[] omni:sensor:Core:emitterState:{tag}:channelId = {arr(list(range(1, n + 1)))}',
            f'    float[] omni:sensor:Core:emitterState:{tag}:distanceCorrectionM = {arr([0.0] * n)}',
            f'    float[] omni:sensor:Core:emitterState:{tag}:elevationDeg = {arr(els)}',
            f'    uint[] omni:sensor:Core:emitterState:{tag}:fireTimeNs = {arr(fire)}',
            f'    float[] omni:sensor:Core:emitterState:{tag}:focalDistM = {arr([0.0] * n)}',
            f'    float[] omni:sensor:Core:emitterState:{tag}:focalSlope = {arr([0.0] * n)}',
            f'    float[] omni:sensor:Core:emitterState:{tag}:horOffsetM = {arr([0.0] * n)}',
            f'    float[] omni:sensor:Core:emitterState:{tag}:reportRateDiv = {arr([0.0] * n)}',
            f'    float[] omni:sensor:Core:emitterState:{tag}:vertOffsetM = {arr([0.0] * n)}',
        ]
        return "\n".join(lines)

    emitter_states = "\n".join(state_block(i, st)
                               for i, st in enumerate(states, start=1))


    body = f'''#usda 1.0
(
    defaultPrim = "Unitree_L1"
    doc = """Unitree 4D LiDAR L1, as measured on the NSL AMR.

Not a datasheet transcription. The scan pattern here comes from a 39 s
stationary recording of the real sensor, analysed by
nav3d/analyze_lidar_bag.py:

    vertical    0 .. 90 deg      (88.3 measured; L2 would be 96)
    horizontal  360 deg
    rate        9.4 Hz, 593 points per message, 5889 points/s
    range       0.28 .. 13.0 m seen indoors; 0.05 .. 30 m rated
    mounting    0.31 m above the floor, no rotation

The 18 elevations below are spread to match the measured density, which
is not uniform: the real sensor concentrates returns near the horizon
and near the zenith and thins out in between.

What this does NOT reproduce: the L1 is non-repetitive -- its `ring`
field is a sweep index, not a laser line, and each sweep paints
different directions, so coverage fills in over a second or two. This
is a fixed grid: the right points on average, but the same directions
every revolution."""
)

def OmniLidar "Unitree_L1" (
    prepend apiSchemas = ["OmniSensorGenericLidarCoreAPI"]
)
{{
    float omni:sensor:Core:avgPowerW = {spec["avgPowerW"]}
    float omni:sensor:Core:azimuthErrorMean = {spec["azimuthErrorMean"]}
    float omni:sensor:Core:azimuthErrorStd = {spec["azimuthErrorStd"]}
    float omni:sensor:Core:elevationErrorMean = {spec["elevationErrorMean"]}
    float omni:sensor:Core:elevationErrorStd = {spec["elevationErrorStd"]}
{emitter_states}
    float omni:sensor:Core:farRangeM = {spec["farRangeM"]}
    token omni:sensor:Core:intensityMappingType = "{spec["intensityMappingType"]}"
    token omni:sensor:Core:intensityProcessing = "NORMALIZATION"
    uint omni:sensor:Core:maxReturns = {spec["maxReturns"]}
    float omni:sensor:Core:minReflectance = {spec["minReflectance"]}
    float omni:sensor:Core:nearRangeM = {spec["nearRangeM"]}
    uint omni:sensor:Core:numberOfChannels = {n}
    uint omni:sensor:Core:numberOfEmitters = {n}
    uint omni:sensor:Core:emitterStateCount = {len(states)}
    uint omni:sensor:Core:stateResolutionStep = {int(spec.get("stateResolutionStep", 1))}
    uint omni:sensor:Core:pulseTimeNs = {spec["pulseTimeNs"]}
    float omni:sensor:Core:rangeAccuracyM = {spec["rangeAccuracyM"]}
    float omni:sensor:Core:rangeResolutionM = {spec["rangeResolutionM"]}
    token omni:sensor:Core:rayType = "{spec["rayType"]}"
    uint omni:sensor:Core:reportRateBaseHz = {report_rate}
    token omni:sensor:Core:rotationDirection = "{spec["rotationDirection"]}"
    uint omni:sensor:Core:scanRateBaseHz = {int(spec["scanRateBaseHz"])}
    token omni:sensor:Core:scanType = "{spec["scanType"].upper()}"
    bool omni:sensor:Core:skipDroppingInvalidPoints = 1
    float omni:sensor:Core:startAzimuthOffsetDeg = 0
    string omni:sensor:modelName = "Unitree_L1"
}}
'''
    OUT.write_text(body, encoding="utf-8")
    steps = report_rate // int(spec["scanRateBaseHz"])
    print(f"wrote {OUT}")
    print(f"      azimuth density x{density}: {360.0 / steps:.2f} deg between "
          f"columns, {0.76 / density:.2f} m apart at 4 m")
    print(f"      {n} emitters x {steps} azimuth steps = {n * steps} points/frame "
          f"at {int(spec['scanRateBaseHz'])} Hz = {n * steps * int(spec['scanRateBaseHz'])} points/s")
    every_el = [float(v) for st in states for v in st["elevationDeg"]]
    every_az = [float(v) for st in states for v in st["azimuthDeg"]]
    print(f"      elevation {min(every_el):+.2f} .. {max(every_el):+.2f} deg")
    print(f"      {len(states)} emitter state(s), azimuth offsets "
          f"{sorted({round(a, 3) for a in every_az})}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Unit checks for comfort_core/comfort_reference.py (Stage 2 simulation).

    python3 scripts/check_comfort_reference.py      # plain
    python3 -m pytest scripts/check_comfort_reference.py -q
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from comfort_core import comfort_reference as cr  # noqa: E402

TH = {"right": 0.0, "front": math.pi / 2, "left": math.pi, "back": 3 * math.pi / 2}


def test_table1_reproduced_at_the_four_sides():
    for ref in ("N22a_A", "N22a_B"):
        r = cr.REFERENCES[ref]
        for side, th in TH.items():
            a0, sig = cr.params(ref, th)
            assert abs(a0 - r["a0"][side]) < 1e-9 and abs(sig - r["sigma_cm"][side]) < 1e-9


def test_equation1_values():
    # C = 1 + a0 (1 - exp(-d^2 / 2 sigma^2)), study B back, d = 80 cm
    exp = 1 + 4.58 * (1 - math.exp(-80 ** 2 / (2 * 39.11 ** 2)))
    assert abs(cr.comfort("N22a_B", 0.80, TH["back"]) - exp) < 1e-9
    assert abs(cr.comfort("N22a_B", 0.0, TH["front"]) - 1.0) < 1e-12


def test_inverse_and_monotone():
    for ref in cr.REFERENCES:
        for th in TH.values():
            prev = 0.0
            for d in (0.2, 0.4, 0.6, 0.8, 1.0, 1.5):
                c = cr.comfort(ref, d, th)
                assert c >= prev
                prev = c
            for tg in (3.0, 4.0, 5.0):
                dd = cr.distance_for(ref, tg, th)
                if dd is not None:
                    assert abs(cr.comfort(ref, dd, th) - tg) < 1e-9


def test_unreachable_target_is_none():
    # study A (humanoid) back: max comfort 1 + 3.92 = 4.92 < 5
    assert cr.distance_for("N22a_A", 5.0, TH["back"]) is None
    assert abs(cr.max_comfort("N22a_A", TH["back"]) - 4.92) < 1e-9


def test_direction_convention():
    assert abs(cr.theta_from_rel(0.0) - TH["front"]) < 1e-12        # ahead of the person
    assert abs(cr.theta_from_rel(math.pi / 2) - TH["left"]) < 1e-12  # to their left
    assert abs(cr.theta_from_rel(-math.pi / 2) % (2 * math.pi) - TH["right"]) < 1e-12
    assert abs(cr.theta_from_rel(math.pi) - TH["back"]) < 1e-12
    assert cr.sector_of(0.1) == "front" and cr.sector_of(3.0) == "rear" and cr.sector_of(1.5) == "side"


def test_required_extents():
    e = cr.required_extents(5.0, "stationary", 0.24)
    d_back = cr.distance_for("N22a_B", 5.0, TH["back"])
    assert abs(e["rear"] - round(d_back - 0.24, 4)) < 1e-9
    m = cr.required_extents(5.0, "stationary", 0.24, basis="clearance_matched")
    assert abs((e["rear"] - m["rear"]) - (0.325 - 0.24)) < 1e-3
    assert cr.required_extents(6.0, "stationary", 0.24)["rear"] is None   # back never reaches 6
    assert cr.required_extents(5.0, "station_engaged", 0.24, engaged_extra_m=0.1)["rear"] > e["rear"]


def test_score_passage_straight_pass():
    # person at origin facing +y; AMR drives along y = -0.6 (behind them) at 0.5 m/s
    ticks = [{"t": i * 0.2, "amr": {"AMR1": [-3 + 0.1 * i, -0.6, 0.0]},
              "people": {"a": {"s": "stationary", "x": 0.0, "y": 0.0, "h": math.pi / 2, "v": 0.0}}}
             for i in range(61)]
    s = cr.score_passage(ticks, "AMR1", "a", 0.24, contact_m=0.49)
    assert abs(s["min_cc_m"] - 0.6) < 1e-9 and s["sector"] == "rear"
    assert abs(s["predicted_comfort_at_closest"] - round(cr.comfort("N22a_B", 0.6, TH["back"]), 3)) < 1e-9
    assert s["min_ttc"] is None                      # never on a collision course
    assert abs(s["amr_speed_at_closest_mps"] - 0.5) < 1e-6


if __name__ == "__main__":
    n = 0
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f()
            n += 1
    print(f"{n} checks passed")

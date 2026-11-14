#!/usr/bin/env python3
"""Correct a Stage 2 trial's status after the fact, without editing trials.csv.

    python3 scripts/stage2_mark_trial.py --session-dir results/stage2/261201_pilot \
        --trial-id 261201_pilot_P01_03 --status INVALID_TRIAL --reason "participant moved off the mark"

Appends one row to <session-dir>/status_amendments.csv
(comfort_core.stage2.STATUS_AMENDMENT_FIELDS). trials.csv stays exactly as
recorded; scripts/stage2_analysis.py applies the latest amendment per
trial (comfort_core.stage2.usable_ratings). Use it for what the recorder
could not know: a participant who asked to stop after the passing, a
repeated trial, a disturbed run. Reasons must not contain personal data.
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.stage2 import (STATUS_AMENDMENT_FIELDS, TRIAL_STATUSES,  # noqa: E402
                                 append_row, read_rows)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session-dir", required=True)
    ap.add_argument("--trial-id", required=True)
    ap.add_argument("--status", required=True, choices=TRIAL_STATUSES)
    ap.add_argument("--reason", required=True)
    ap.add_argument("--note", default="")
    args = ap.parse_args()
    d = Path(args.session_dir)
    known = {r.get("trial_id") for r in read_rows(d / "trials.csv")}
    if args.trial_id not in known:
        print(f"[stage2_mark_trial] {args.trial_id} is not in {d / 'trials.csv'}", file=sys.stderr)
        return 2
    append_row(d / "status_amendments.csv",
               {"trial_id": args.trial_id, "trial_status": args.status,
                "stop_reason": args.reason, "operator_note": args.note,
                "timestamp": datetime.now().isoformat(timespec="seconds")},
               STATUS_AMENDMENT_FIELDS)
    print(f"{args.trial_id} -> {args.status} ({args.reason})")
    return 0


if __name__ == "__main__":
    sys.exit(main())

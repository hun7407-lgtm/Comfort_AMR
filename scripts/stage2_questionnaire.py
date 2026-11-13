#!/usr/bin/env python3
"""Record a participant's rating of one passing (Stage 2).

    python3 scripts/stage2_questionnaire.py --session-dir results/stage2/261201_pilot \
        --participant P01 --trial-id 261201_pilot_P01_03 --condition b3o_r1.00 \
        --size 1.00 --order 3
    ... --comfort 5 --safety 6 --distance 4         non-interactive (values given)
    python3 scripts/stage2_questionnaire.py --show-items   print the items and anchors

Normally launched by scripts/stage2_trial.py right after the robot has
passed. Asks the primary item (perceived comfort, 1..7, after Neggers et
al. 2022) and the two secondary items, validates the range, and appends
one row to <session-dir>/questionnaire.csv with the schema in
comfort_core.stage2.QUESTIONNAIRE_FIELDS:

    participant_id, trial_id, condition, comfort_size, comfort_score,
    perceived_safety, distance_acceptability, trial_order, timestamp, notes

Only a pseudonymous participant code is recorded. No name, no age, no
contact: whatever consent form the study uses stays on paper.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.stage2 import (LIKERT_MAX, LIKERT_MIN, QUESTIONNAIRE_FIELDS,  # noqa: E402
                                 QUESTIONNAIRE_ITEMS, append_row, make_response)


def show_items(lang: str) -> None:
    for item in QUESTIONNAIRE_ITEMS:
        tag = "PRIMARY" if item["primary"] else "secondary"
        print(f"[{tag}] {item['key']}: {item[lang]}")
        print("    " + "   ".join(item[f"anchors_{lang}"]))
        print(f"    source: {item['source']}")


def ask(item, lang: str, required: bool) -> int | None:
    prompt = (f"{item[lang]}\n   {'   '.join(item[f'anchors_{lang}'])}\n   "
              f"[{LIKERT_MIN}-{LIKERT_MAX}{'' if required else ', Enter to skip'}] > ")
    while True:
        raw = input(prompt).strip()
        if not raw and not required:
            return None
        try:
            v = int(raw)
            if LIKERT_MIN <= v <= LIKERT_MAX:
                return v
        except ValueError:
            pass
        print(f"   please enter an integer {LIKERT_MIN}..{LIKERT_MAX}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session-dir")
    ap.add_argument("--participant")
    ap.add_argument("--trial-id")
    ap.add_argument("--condition")
    ap.add_argument("--size", type=float)
    ap.add_argument("--order", type=int)
    ap.add_argument("--comfort", type=int, help="primary item, 1..7")
    ap.add_argument("--safety", type=int)
    ap.add_argument("--distance", type=int)
    ap.add_argument("--notes", default="")
    ap.add_argument("--lang", choices=("ko", "en"), default="ko")
    ap.add_argument("--show-items", action="store_true")
    args = ap.parse_args()

    if args.show_items:
        show_items(args.lang)
        return 0
    for req in ("session_dir", "participant", "trial_id", "condition", "size", "order"):
        if getattr(args, req) is None:
            ap.error(f"--{req.replace('_', '-')} is required")

    answers = {"comfort_score": args.comfort, "perceived_safety": args.safety,
               "distance_acceptability": args.distance}
    if args.comfort is None:
        print(f"\n--- {args.trial_id}  ({args.condition}, {args.size:.2f} m) ---")
        for item in QUESTIONNAIRE_ITEMS:
            answers[item["key"]] = ask(item, args.lang, item["primary"])
    try:
        row = make_response(args.participant, args.trial_id, args.condition, args.size,
                            args.order, answers["comfort_score"], answers["perceived_safety"],
                            answers["distance_acceptability"], args.notes)
    except ValueError as exc:
        print(f"[stage2_questionnaire] not recorded: {exc}", file=sys.stderr)
        return 2
    path = Path(args.session_dir) / "questionnaire.csv"
    append_row(path, row, QUESTIONNAIRE_FIELDS)
    print(f"recorded {row['trial_id']}: comfort {row['comfort_score']}, "
          f"safety {row['perceived_safety'] or '-'}, distance {row['distance_acceptability'] or '-'} "
          f"-> {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

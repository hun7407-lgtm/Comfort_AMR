#!/usr/bin/env python3
"""Record a participant's rating of one passing (Stage 2).

    python3 scripts/stage2_questionnaire.py --session-dir results/stage2/261201_pilot \
        --participant P01 --trial-id 261201_pilot_P01_03 --condition b3o_r1.00 \
        --size 1.00 --order 3 --context c2_stationary_rear --passing-distance 0.75
    ... --rating comfort_score=5 --rating perceived_safety=6     non-interactive
    python3 scripts/stage2_questionnaire.py --show-items          print items and anchors

Normally launched by scripts/stage2_trial.py right after a NORMAL passing.
The items, their wording, scale and anchors come from the config
(stage2.questionnaire in configs/stage2/stage2_real.yaml), so a pilot can
change them without touching code; the config's `version` is written into
every row. One row is appended to <session-dir>/questionnaire.csv with the
schema comfort_core.stage2.QUESTIONNAIRE_FIELDS.

Only the anonymous participant code is recorded (checked against
stage2.participant_id_pattern). No name, no age, no contact: whatever
consent form the study uses stays on paper.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.config import load_with_base  # noqa: E402
from comfort_core.stage2 import (QUESTIONNAIRE_FIELDS, append_row, make_response,  # noqa: E402
                                 questionnaire_items, stage2_block)


def anchors(item, lang):
    return "   ".join((item.get("anchors") or {}).get(lang, []))


def show_items(items, lang: str) -> None:
    for item in items:
        tag = "PRIMARY" if item.get("primary") else "secondary"
        print(f"[{tag}] {item['key']} ({item['scale_min']}..{item['scale_max']}): {item['text'][lang]}")
        print("    " + anchors(item, lang))
        print(f"    source: {item.get('source', '')}")


def ask(item, lang: str) -> int | None:
    required = bool(item.get("primary") or item.get("required"))
    lo, hi = item["scale_min"], item["scale_max"]
    prompt = (f"{item['text'][lang]}\n   {anchors(item, lang)}\n   "
              f"[{lo}-{hi}{'' if required else ', Enter to skip'}] > ")
    while True:
        raw = input(prompt).strip()
        if not raw and not required:
            return None
        try:
            v = int(raw)
            if lo <= v <= hi:
                return v
        except ValueError:
            pass
        print(f"   please enter an integer {lo}..{hi}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/stage2/stage2_real.yaml")
    ap.add_argument("--session-dir")
    ap.add_argument("--participant")
    ap.add_argument("--trial-id")
    ap.add_argument("--condition")
    ap.add_argument("--size", type=float)
    ap.add_argument("--order", type=int)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--context", default="", help="Human Context, e.g. c2_stationary_rear")
    ap.add_argument("--passing-distance", type=float, default=None,
                    help="planned passing distance d, m (not the rating)")
    ap.add_argument("--rating", action="append", default=[], metavar="KEY=VALUE",
                    help="non-interactive answer, e.g. comfort_score=5 (repeatable)")
    ap.add_argument("--comment", default=None, help="optional free-text comment")
    ap.add_argument("--lang", choices=("ko", "en"), default=None)
    ap.add_argument("--show-items", action="store_true")
    args = ap.parse_args()

    cfg = load_with_base(args.config)
    items = questionnaire_items(cfg)
    q = stage2_block(cfg).get("questionnaire") or {}
    lang = args.lang or q.get("language") or "ko"
    if args.show_items:
        show_items(items, lang)
        return 0
    for req in ("session_dir", "participant", "trial_id", "condition", "size", "order"):
        if getattr(args, req) is None:
            ap.error(f"--{req.replace('_', '-')} is required")

    ratings = {}
    for kv in args.rating:
        k, _, v = kv.partition("=")
        ratings[k.strip()] = int(v)
    comment = args.comment
    if not args.rating:
        print(f"\n--- {args.trial_id}  ({args.condition}, {args.size:.2f} m) ---")
        for item in items:
            ratings[item["key"]] = ask(item, lang)
        if comment is None:
            comment = input(f"{(q.get('comment_prompt') or {}).get(lang, 'comment')} > ").strip()
    try:
        row = make_response(cfg, args.participant, args.trial_id, args.condition, args.size,
                            args.order, ratings, comment or "", context=args.context,
                            passing_distance_m=args.passing_distance, repeat=args.repeat)
    except ValueError as exc:
        print(f"[stage2_questionnaire] not recorded: {exc}", file=sys.stderr)
        return 2
    path = Path(args.session_dir) / "questionnaire.csv"
    append_row(path, row, QUESTIONNAIRE_FIELDS)
    print(f"recorded {row['trial_id']}: " + ", ".join(
        f"{it['key']} {row[it['key']] if row[it['key']] != '' else '-'}" for it in items)
        + f" -> {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

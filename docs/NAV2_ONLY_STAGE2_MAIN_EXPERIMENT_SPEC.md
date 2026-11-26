# NAV2-only experiment decision & Stage 2 main study (2026-11-26)

**Status:** Authoritative NEW design/implementation handoff, not evidence that experiments were run. **User decision:** The master's thesis proposes and validates a **system framework** that dynamically selects the operational size of literature-referenced asymmetric Comfort Spaces per Human State. It does **not** claim a new geometric Comfort Space or human-study validation. All compared robots must use **Nav2 navigation only** for movement; no custom Yield / Evasion / Backoff / Off-lane motion policy.

**Authority / supersession:** This decision overrides incompatible controller, deadlock and selection instructions in older \`docs/FINAL_261125_PLAN.md\`, \`docs/STAGE2_UNIFORM_SCALE_PROTOCOL.md\`, \`docs/STAGE2_SCALE_IMPLEMENTATION.md\`, and \`AGENTS.md\` for **NEW runs only**. Those documents and existing result batches remain immutable historical records. If these older files imply a live Yield Gate, follow this document for the new Nav2-only study. Do not silently relabel historical results.

## A. Critical audit of the current committed code (before ANY full run)

- \`configs/final/seeds/s1.yaml\`, \`s101.yaml\`: \`comfort_space.costmap.buffer_width: 0.1\`, \`yield.evade.enabled: false\`, \`recovery.backoff.enabled: false\`, \`recovery.off_lane.enabled: false\`, **BUT \`yield.enabled\` is not false**. \`configs/common.yaml\` defaults \`yield.enabled: true\`. In \`scripts/comfort_gate.py:_on_cmd()\`, that gate zeros a motion command when it predicts comfort-space intrusion. Therefore Stage 1's legacy \`R_nav_upper=0.90m\` is a **Gate/Hold-assisted operational bound**, not a verified "pure Nav2-only" upper bound.
- New \`configs/stage2scale/envs/*.yaml\` inherit \`configs/common.yaml\` and do not override the above. Thus the *committed 40-launch smoke* at \`results/batches/261126_stage2scale_v1_smoke/\` ran with \`costmap.buffer_width=0.35\`, \`yield.enabled=true\`, \`yield.evade.enabled=true\`, \`recovery.backoff.enabled=true\` and likely \`off_lane\` enabled via code default. One MV-P contact was observed during \`STEPPING ASIDE\`. **Historical smoke cannot validate the new Nav2-only decision.**
- \`scripts/launch_nodes.py:nav2_stack()\`: AMR1 Nav2 velocity is remapped to \`cmd_vel_nav\`, then \`scripts/nav2_bringup.launch.py\` launches \`comfort_gate.py\` -> \`cmd_vel_safe\` -> Nav2 Collision Monitor -> actuator. Fleet robots' Nav2 controller outputs directly to their namespaced \`cmd_vel_safe\` -> monitor. For the NEW runs, use the **same direct Nav2-to-Collision-Monitor connection** for AMR1 and fleet, ideally by NOT LAUNCHING \`comfort_gate.py\`. A demonstrably bitwise pass-through gate is a temporary implementation fallback only; verify no motion modifications, extra timers or conflicting velocity publishers. Do not disable the Collision Monitor.
- \`perception.source: tf\`, \`humans.state_source: ground_truth\` are the currently controlled B1/B2/B3 policy-comparison defaults. Do NOT describe the new main experiment as closed-loop LiDAR-based worker-state recognition unless a separate sensor-mode run is actually performed and evaluated.
- All three prior stages remain historical: \`results/batches/261125_final_stage1\`, \`261125_final_stage2_proxy\`, \`261125_final_stage3\`. They cannot be called new Nav2-only results.

## B. Definition of the NEW Nav2-only movement-control experiment

**Physical/ROS data path for each AMR:** Simulated humans' pose/state + geometry -> state-wise Comfort Space policy -> analytic non-lethal costmap filter mask (+0.10m external cost band) -> each AMR's Nav2 Global/Local Costmap -> NavFn Global Planner -> DWB Local Controller / native Nav2 behavior recovery -> **Nav2 Collision Monitor** -> differential-drive base. No separate human-social Hold/Yield Gate; no self-authored \`cmd_vel\` for sidestep, avoidance, recovery or backoff. The physical collision monitor can STOP/SLOW the robot to prevent contact: it is a **safety control**, not a Comfort Yield policy. \`traffic_rule\` stays off unless an independent, prespecified control study explicitly includes it.

**Required effective settings, for B1/B2/B3 and for Stage 1 revalidation, Stage 2 and Stage 3 new runs:**

\`\`\`yaml
comfort_space:
  costmap:
    buffer_width: 0.10
  yield:
    enabled: false
    evade:
      enabled: false
recovery:
  backoff:
    enabled: false
  off_lane:
    enabled: false
traffic_rule:
  enabled: false
\`\`\`

\`comfort_space.costmap.enabled\` and policy-specific mask must still work for B2/B3. In B1, no Comfort mask, but all other physical configuration and Nav2/safety controls remain the same. Keep \`comfort_space.min_radius=0.30m\`, \`mask_value=98\` (high finite cost, not lethal), robot footprint, lidar obstacle input, map, fixed 0.10m buffer, robot collision monitoring enabled, parameters of built-in Nav2 behavior tree/recoveries unchanged. **Do not claim "no reverse behavior at all" without checking Nav2's built-in recovery backup actions; claim only "no additional custom yielding/evasion/backoff/off-lane controller."**

**Runtime checks (not just YAML inspection):** inspect merged configs and actual ROS graph for all policies / robot namespaces; assert no active \`comfort_gate\` altering \`cmd_vel\`, no directly written \`cmd_vel\` from \`EvasionController\`, no unintended dual publishers, Nav2 output routed into collision monitor and collision monitor into actuators; compare policy-invariant configs by hash and capture the actual unmodified velocity pipeline. A short path-following and safety stop smoke is mandatory. Do not remove safety hardware/software, reduce protection margins, or use direct velocity control to force successful passage.

## C. Stage 1 is a prerequisite: old 0.90m is a *reference*, not new proof

Legacy observed Stage 1 0.90m came from Gate-enabled B2 Static Circle, 8 humans + 3 AMRs (seeds 1-30), with deadlock rule relying on gate-induced stops. After the Gate is removed, \`comfort-induced deadlock\` based on \`GATE_HOLD\` may cease to exist as a metric, and the success/protective-stop transition can shift. **Before claiming a Nav2-only R_nav_upper, run a fresh B1/B2 Static-Circle revalidation** with the exact new Nav2-only pipeline, fixed Stage 1 multi-AMR factory layout, matched seeds and repeats; start with the old transition neighborhood \`r=0.85, 0.90, 0.95, 1.00m\` and extend coarse range if transition is outside it. The old 0.90m can be used as a **legacy initialization anchor** for diagnostics, never automatically adopted as the new measured upper bound.

The research team must freeze a **new, implementable operational rule BEFORE new data**: B1-matched goal-completion rate and time, human/AMR contact, collision-monitor stops, Nav2 planning/recovery failures, residual width and other measured feasibility indicators. State and report the thresholds and confidence intervals as engineering choices; no \`GATE_HOLD\`-based event rule when the gate is disabled. Preserve all unsuccessful runs. If no meaningful upper transition appears, report "upper bound not identified in tested range" and choose a prespecified design anchor with the correct label instead of pretending 0.90m is the new upper bound.

Stage 2's scale normalization anchor \`R_ref\` must carry provenance: \`legacy_0.90m\` until revalidated, or \`new_nav2_only_upper\` after the revalidation. Document whether normalization requires regeneration; do not retroactively modify past results. Freeze the final \`R_ref\` BEFORE Stage 2 confirmatory runs.

## D. Stage 2 primary design — state-wise UNIFORM scale; not independent axes

Three **literature-referenced but quantitatively engineering-chosen** fixed shape templates; reference speed 1.0m/s for moving (constant magnitude in primary sweep, moving heading from velocity). Let \`E=(Front,Side,Rear)\`, \`E_H(1)=R_ref * E_H(raw)/max(E_H(raw))\`, \`E_H(alpha)=alpha * E_H(1)\`, one and only one swept \`alpha\` per trial. Keep the offset-ellipse geometry and orientations from \`comfort_core/comfort_space.py\`. No independent F/S/R parameter optimization, no changing geometry/size with a different radius-dependent aisle. The original 0.90m example is:

| Human state | raw F/S/R (m) | normalized F/S/R at alpha=1 and R_ref=0.90m |
|---|---|---|
| Station Engaged | 0.80 / 0.70 / 0.60 | 0.900 / 0.7875 / 0.675 |
| Stationary | 0.80 / 0.90 / 1.05 | 0.685714 / 0.771429 / 0.900 |
| Moving | 1.20 / 0.90 / 0.90 | 0.900 / 0.675 / 0.675 |

Main grid \`alpha=[1.00,0.90,0.80,0.70,0.60,0.50,0.45]\` for R_ref=0.90, with \`B1 (none)\`, \`B2 (static R_ref)\` under matched physical conditions. \`alpha=0.45\` is near the current \`min_radius=0.30\` clipping floor; if \`R_ref\` changes, **recompute the unclipped minimum alpha** and revise the grid BEFORE confirmatory experiments. Use \`aisle_scaling.enabled:false\` and directly generate \`F,S,R\` for each alpha; assert exact scale ratios and actual shape bounding radius.

**Stage 2 controlled contexts (one human + one AMR):**
- \`ST-F\`, \`ST-S\`, \`ST-R\`: standing human, AMR passes in front, side and rear (add left/right balancing for side, or explicitly scope inference).
- \`EN-R\`: engaged at station; **reference comfort is transferred from stationary human**, not directly measured under work tasks.
- \`MV-P\`: human + AMR in **different fixed parallel lanes**, pedestrian visible on desired left side and AMR right from the intended top-view convention, opposite x directions, actual lateral pass. Suggested lane gaps \`0.65/0.80/1.00m\` are **pilot candidates only**, not comfort targets. Log actual closest distance, relative bearing, person gait sway and potential safety-stop/contact.
- Overtaking \`MV-O\` remains unsupported until speeds/reference-domain permit; do not fake overtaking data.

**Pilot**: First REPEAT small Nav2-only smoke on fresh \`nav2only_v2\` batch ID (not \`261126_stage2scale_v1_smoke\`), including alpha 1.0/0.7/0.45 and B1/B2 at fixed widths and lane gaps; reject invalid geometry. Predeclare a static corridor width candidate set 2.0/2.2/2.4/2.6m and moving widths 2.6/2.8/3.0m, lane gaps above, then run pilot 2-3 independent launches/cell. **Compare B1/B2/each alpha on exactly the same physical scenario and chosen width/gap**, freeze before coarse. Reject a pilot geometry only for prespecified technical infeasibility or lack of alpha-to-actual-clearance sensitivity, not because it yields an unwelcome Comfort result. Record all candidate geometries, rejected and accepted, and their reasons.

**Main coarse**: five primary contexts × {B1,B2,seven alpha levels} × 10 independent launches/cell = **450 runs** at the frozen tight geometry (plus prespecified sensitivity and pilot, separately). Run-condition order interleaved/randomized; matched scenario seeds (not deterministic output guarantees). **Confirmation**: >=30 *new* launches for selected alpha and immediately smaller tested value, matched B1/B2, on tight geometry + a preregistered held-out wide geometry (and optional multi-AMR stress). If uncertainty too wide, increase independent launch count rather than claim robust improvement.

**Scores**:
- \`predicted_comfort_at_closest=C(d_cc,theta)\` from \`comfort_core/comfort_reference.py\`, using **logged actual human–AMR centre distance and bearing**, not configured radius/alpha or mask. External reference: Neggers 2022a standing AGV at ~0.35 m/s; Neggers 2022b walking Pepper head-on at ~0.35m/s. \`station_engaged\` transfer, higher robot speeds, noncomparable robot body size and unsupported moving angles must be **flagged as limitations**. Primary target \`C*=5/7\` is an operational assumption; sensitivity 4.5/5.5.
- Navigation: completed intended encounter + destination, no human contact, no terminal collision-monitor failure, correct sector and valid state. Evaluate noncompleted/blocked/failed navigation as **joint failure even if distance-based Comfort would be high**. Report actual centre distance, true oriented footprint-edge clearance, pedestrian/robot speeds, collision-monitor interventions, path length/time, Nav2 abort, time-to-pass, and predicted comfort on completed intended interactions.
- Joint pass \`J=1\` iff intended mission successfully passes and predicted Comfort >=C*. Acceptability candidate: nav completion >=0.80 AND joint success >=0.80; report raw counts/n and Wilson 95% intervals. Separate navigation, conditional-comfort and joint rate. 0.90 threshold sensitivity. No sample with n=1/2 should be described as final.
- Planner-related blockage may replace old comfort-gate-induced deadlock: classify Nav2 abort, no-progress timeout, physical collision monitor stop, geometric block, and costmap-caused route infeasibility where evidence supports it. Do **not** claim custom gate-induced deadlock when no Gate exists; causal attribution to Comfort mask requires matched B1/B2/alpha comparisons and detailed logs.
- **Nonmonotone selection**: larger mask can cause no progress, smaller mask can bring closer passage; do not enforce "all larger scales pass". Smallest *actually tested* alpha jointly feasible across all preregistered primary scenarios for each Human State (stationary requires ST-F/S/R); otherwise \`NO_COMMON_FEASIBLE_ALPHA\`.
- **Identification guard**: If \`alpha=0.45\` still passes C* and Nav2, report \`FLOOR_LIMITED/BOUND_NOT_OBSERVED\`, not validated comfort lower bound. If achieved actual distance barely changes vs alpha (provisional engineering detectability 0.10m), report \`UNIDENTIFIABLE_GEOMETRY\`, not a chosen comfort bound. If predicted comfort degrades below threshold at a smaller alpha and a larger tested alpha passes, report a tested \`IDENTIFIED_BRACKET\`, with uncertainty and speed-domain caveat. Collisions remain failures even if pose-centre comfort score is high.

## E. Stage 3 transfer and hypotheses (do not manufacture outcomes)

New stage 3 is **not** the historical \`261125_final_stage3\` gate-assisted batch. Transfer Stage 2's preselected per-state scales to a **fresh 8-worker / 3-AMR** factory cross scenario, same Nav2-only movement chain for AMR1/a1/a2, same frozen parameters, and **new holdout seeds** never used in Stage 1 revalidation or Stage 2 policy selection. Compare:
\`B1\` (no Comfort),
\`B2\` (static circle at newly justified common reference),
\`B3-global-alpha\` (same reference state-shaped templates but one shared alpha, if feasible, to isolate dynamic state-scale benefit),
\`B3-state-alpha\` (one alpha per state selected independently in Stage 2).

Make an explicit ablation separating the effect of asymmetric template and state-dependent scaling; report cases where a context lacks an identified comfort limit and label a value as a provisional engineering choice. Do not call a static ellipse or compressed shape the thesis' geometric invention. Hypothesis: dynamic per-state scale may preserve literature-predicted comfort while increasing multi-AMR completion / reducing no-progress relative to a comparable static policy. Not a proven result.

**Data limitations**: The current experiments feed true simulated human pose and ground-truth state into the Comfort generator. LiDAR + state estimator is a separate capability/evaluation, not a verified source of the current policy. Human Comfort is predicted from published human ratings, not measured from subjects in Isaac Sim.

## F. Engineering delivery / stop gates for Claude

1. Inspect \`AGENTS.md\`, \`configs/common.yaml\`, \`scripts/launch_nodes.py\`, \`scripts/nav2_bringup.launch.py\`, \`scripts/comfort_gate.py\`, \`configs/final/seeds/s1.yaml\`, \`configs/stage2scale/stage2scale.yaml\`, \`comfort_core/stage2scale.py\`, \`scripts/stage2scale_plan.py\`, \`scripts/stage2scale_report.py\`, \`scripts/check_stage2scale.py\`, previous reports.
2. Implement mode as **new versioned** configs/sources (or minimal backward-compatible flag), never globally mutate old trial settings, leave safety enabled. Add an assertive merged-config and runtime ROS topic guard verifying Nav2-only, **actual gate absence/pass-through**, buffer 0.10, mask works.
3. Tests: B1/B2/B3 same physical parameters; local controller truly wired to monitor for every AMR; positive mask response; no customized \`cmd_vel\` publishes; Nav2 native recoveries unchanged; safety stops still operate; uniform scale exact; moving two lanes; outcome scoring; nonmonotonic selection; missing data; reporter distinguishes earlier results.
4. Check that old reports dependent on \`GATE_HOLD\` and historical success/deadlock definitions are **revised for new runs** before new Stage 1 selection, Stage 2 or Stage 3 results.
5. Use fresh batch root (suggest \`results/batches/261126_nav2only_stage2_v2_*\` or date-of-run equivalent). **Never overwrite** old \`results/batches/261125_*\`, \`261126_stage2sim_*\`, or \`261126_stage2scale_v1_smoke\`. Log code commit, config/geometry hash, simulated time vs wall time, seed, attempt ID, velocity path source, all safety alerts, success/contact outcomes, and missing data.
6. Run verification in order: static/unit -> 1-trial runtime Nav2/safety smoke -> Stage 1 Nav2-only upper revalidation -> Stage 2 Nav2-only smoke -> pilot/freeze -> 450 coarse -> independent confirmation -> optional Stage 3 *only after policy frozen*. User requested a handoff **for running the main experiment**, so do not impose a pointless approval stop after tests pass. **Do** stop full campaign and report clearly if a safety test fails, collisions arise from unsafe geometry, Stage 1 upper cannot be justified, or insufficient/invalid data cannot be classified; never bypass the guard.
7. Deliver before/after config diffs, commands run, per-stage counts/denominators, confidence intervals, identified/unidentified contexts, diagnostic top-view videos, and Korean researcher summary. If Isaac Sim/ROS 2 environment is unavailable, execute all available offline checks and give exact executable commands **without claiming a run occurred**.

## G. Master thesis wording (only after matching verification)

Accurate if implemented/verified:
> "본 연구에서는 모든 AMR에 동일한 ROS 2 Nav2 기반 전역 경로계획 및 지역 주행제어를 적용하고, 인간 상태에 따른 Comfort Space를 Costmap에 반영하였다. 별도의 Human-aware 양보·측면회피·후진 제어 알고리즘은 사용하지 않았으며, 물리적 충돌방지 기능은 독립적인 안전계층으로 유지하였다. 제안 프레임워크는 새로운 비대칭 형상의 제안이 아니라 기존 연구를 참고한 상태별 공간 크기 설정 및 그에 따른 Comfort–Navigation 운용성 평가를 기여점으로 한다."

Do **not** use this as a description of old Stage 1–3 results that still have a Hold Gate.

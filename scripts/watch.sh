#!/usr/bin/env bash
# ---------------------------------------------------------------
# Watch the paper's episodes one at a time, in the Isaac Sim window.
#
#   ./scripts/watch.sh              list every situation worth seeing
#   ./scripts/watch.sh 1            play episode 1
#   ./scripts/watch.sh 1 2 3        play them in order, one after another
#   ./scripts/watch.sh must         the 6 episodes that show everything
#   ./scripts/watch.sh L2           layout preview 2 (nobody drives)
#   ./scripts/watch.sh layout       all four layout previews
#   ./scripts/watch.sh 1 --record   also write an mp4 of the run
#   ./scripts/watch.sh 1 --plain    without the floor labels
#   ./scripts/watch.sh 1 --dry-run  print what would run, launch nothing
#
# An episode is a real trial: Isaac Sim with a window, Nav2, the
# Comfort Space filter and the yield gate, driven to the same goal the
# batch used. It is NOT recorded as a result -- `--no-record` is always
# passed, so looking at an episode never adds a row to results/trials
# or changes a batch. The window is held open when the trial ends so
# the final pose can be looked at; press Enter in this terminal to
# close it.
#
# A layout preview is the other thing: no Nav2 and no driving. The AMR
# stands in its start slot, the people walk their exchange, and the
# floor carries the state rings and the walk routes. Use it to see WHO
# IS WHERE before watching anybody drive.
#
# The floor labels are decals with no colliders, 4 mm thick, and the
# lowest LiDAR beam is 0.31 m above the floor at the robot, so nothing
# reads them as obstacles. They are on by default here because the
# point of this script is to be able to tell N3 from S1 by eye.
# ---------------------------------------------------------------
set -uo pipefail

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

H4="configs/scenarios/paper_h4_a1_scr_s0001.yaml"
H8A1="configs/scenarios/paper_h8_a1_scr_s0001.yaml"
H8A2="configs/scenarios/paper_h8_a2_scr_s0001.yaml"
H8A3="configs/scenarios/paper_h8_a3_scr_s0001.yaml"

# id|config|policy|must|title|what to watch for
EPISODES=(
"1|$H8A1|b2|yes|8인 · AMR 1 · B2 고정 1.20 m|출발 칸을 못 벗어난다. 0.6 m 나아가고 31 s 멈춤. N3(5.50,+3.01)의 1.44 m 원을 0.01 m 침범해서다. 48시행 중 9건이 전부 이 모습"
"2|$H8A1|b3|yes|8인 · AMR 1 · B3 상태적응 타원|1번과 똑같은 임무를 25 s에 통과한다. N3가 station_engaged(0.60 m)라 칸 출구가 열린다. 논문의 핵심 대비"
"3|$H8A1|b1|yes|8인 · AMR 1 · B1 Comfort Space 없음|2번과 구별이 안 된다. 주행시간 차 0.2 s, 양보 0 s, 침입 0 m. \"B3는 B1에 코드만 더한 것 아닌가\"를 눈으로 확인하는 자리"
"4|$H4|b2|yes|4인 · AMR 1 · B2 (다른 임무, 25.64 m)|여기서는 통로를 지난다. 두 번 비켜서고 +21 s. 비켜선 자리가 E2의 보행 경로라 E2가 멈춘 로봇에 닿는다. 돌릴 때마다 성공(2/3)과 livelock(1/3)이 갈린다"
"5|$H4|b3|yes|4인 · AMR 1 · B3|같은 임무를 33 s에 통과. 4번과 비교할 기준"
"6|$H8A3|b3|yes|8인 · AMR 3 · B3|추가 AMR 2대가 서·동 도크에서 돈다. ego 통로에 닿지 않는 것을 눈으로 확인 — 지금 AMR 대수가 무효 인자인 이유"
"7|$H8A1|b3r|no|8인 · AMR 1 · B3-R 상태적응 원|2번과 모든 수치가 같다. 원/타원은 걷는 사람에게만 다르고, 이 임무에는 걷는 사람이 로봇 통로에 없다. 형상 ablation의 대조가 0인 것을 확인"
"8|$H4|b1|no|4인 · AMR 1 · B1|5번과 동일. 4인에서도 B1≡B3"
"9|$H4|b3r|no|4인 · AMR 1 · B3-R|5번과 동일"
"10|$H8A2|b3|no|8인 · AMR 2 · B3|6번의 AMR 2대 판. a1/a2/a3가 같은 결과인 것을 확인"
)

LAYOUTS=(
"L1|$H4|4인 · AMR 1 · 임무 carts_s2 → carts_n3 (25.64 m)|교대 N2↔E1, S2↔E2. 8인 셀과 임무가 다르다 — 밀도 비교가 교란된 바로 그 지점"
"L2|$H8A1|8인 · AMR 1 · 임무 carts_n3 → carts_s3 (19.64 m)|교대 N1↔E1, S2↔E2. 1·2·3·7번 에피소드가 전부 이 배치"
"L3|$H8A2|8인 · AMR 2|추가 AMR 1대의 작업 경로"
"L4|$H8A3|8인 · AMR 3|추가 AMR 2대의 작업 경로"
)

usage() { sed -n '2,30p' "$0"; }

listing() {
    echo
    echo "  ============================================================"
    echo "   레이아웃 미리보기 — 4개. 누가 어디 있는지 먼저 본다"
    echo "   (Nav2 없음, 로봇은 출발 칸에 서 있고 사람만 걷는다)"
    echo "  ============================================================"
    for row in "${LAYOUTS[@]}"; do
        IFS='|' read -r id cfg title note <<< "$row"
        printf "\n   %-4s %s\n        %s\n" "$id" "$title" "$note"
    done
    echo
    echo "  ============================================================"
    echo "   에피소드 — 10개 중 ★ 6개가 필수. 실제로 주행한다"
    echo "  ============================================================"
    for row in "${EPISODES[@]}"; do
        IFS='|' read -r id cfg pol must title note <<< "$row"
        mark=" "; [ "$must" = "yes" ] && mark="★"
        printf "\n   %s %-3s %s\n        %s\n" "$mark" "$id" "$title" "$note"
    done
    echo
    echo "  ------------------------------------------------------------"
    echo "   필수 6개면 관찰된 모든 거동을 봅니다:  ./scripts/watch.sh must"
    echo "   선택 4개(7-10)는 \"차이가 없다\"는 것 자체를 확인하는 용도입니다."
    echo "   한 편당 Isaac Sim 기동 1-2 분 + 주행 25-55 초."
    echo "   결과는 기록되지 않습니다 (--no-record). 배치는 건드리지 않습니다."
    echo "  ------------------------------------------------------------"
    echo
}

overlay_for() {
    # Floor labels on, for the length of this look only. The scenario
    # file itself still says what a measured run reads.
    local cfg="$1" out="$2"
    cat > "$out" <<YAML
# Written by scripts/watch.sh. Throwaway: it switches the floor labels
# on for one look and no trial ever reads it.
base: $cfg
markers:
  enabled: true
preview:
  enabled: true
YAML
}

play_episode() {
    local id="$1"; shift
    local row cfg pol must title note
    for row in "${EPISODES[@]}"; do
        IFS='|' read -r rid cfg pol must title note <<< "$row"
        [ "$rid" = "$id" ] && break
        cfg=""
    done
    [ -n "$cfg" ] || { echo "[ERROR] no episode $id (1-10)" >&2; return 2; }
    [ -f "$cfg" ] || { echo "[ERROR] missing $cfg -- run scripts/make_scenario.py" >&2; return 1; }

    echo
    echo "=============================================================="
    echo " 에피소드 $id   $title"
    echo "--------------------------------------------------------------"
    echo " 볼 것: $note"
    echo " 설정 : $cfg"
    echo " 정책 : $pol"
    echo "=============================================================="
    echo

    local use_cfg="$cfg" tmp=""
    if [ "$PLAIN" -eq 0 ]; then
        tmp="$(mktemp --suffix=.yaml)"
        overlay_for "$cfg" "$tmp"
        use_cfg="$tmp"
    fi
    if [ "$DRY" -eq 1 ]; then
        echo "[dry-run] ./scripts/run_scenario.sh $pol --config $use_cfg --no-record --hold ${EXTRA[*]+${EXTRA[*]}}"
        [ -n "$tmp" ] && { echo "[dry-run] overlay was:"; sed 's/^/          /' "$tmp"; rm -f "$tmp"; }
        return 0
    fi
    ./scripts/run_scenario.sh "$pol" --config "$use_cfg" --no-record --hold \
        "${EXTRA[@]+"${EXTRA[@]}"}"
    local code=$?
    [ -n "$tmp" ] && rm -f "$tmp"
    echo "[watch] 에피소드 $id 끝 (exit $code; 0 = SUCCESS, 그 외 = 미도달)"
    return 0
}

play_layout() {
    local id="$1"
    local row rid cfg title note found=""
    for row in "${LAYOUTS[@]}"; do
        IFS='|' read -r rid cfg title note <<< "$row"
        [ "$rid" = "$id" ] && { found=1; break; }
    done
    [ -n "$found" ] || { echo "[ERROR] no layout $id (L1-L4)" >&2; return 2; }
    echo
    echo "=============================================================="
    echo " 레이아웃 $id   $title"
    echo "--------------------------------------------------------------"
    echo " 볼 것: $note"
    echo " 창을 닫으면 다음으로 넘어갑니다."
    echo "=============================================================="
    if [ "$DRY" -eq 1 ]; then
        echo "[dry-run] ./scripts/preview_scenario.sh --config $cfg"
        return 0
    fi
    ./scripts/preview_scenario.sh --config "$cfg"
}

PLAIN=0
DRY=0
EXTRA=()
ITEMS=()
while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help)  usage; exit 0 ;;
        --plain)    PLAIN=1; shift ;;
        --dry-run)  DRY=1; shift ;;
        --record|--record-costmap) EXTRA+=("$1"); shift ;;
        must)       ITEMS+=(1 2 3 4 5 6); shift ;;
        all)        ITEMS+=(1 2 3 4 5 6 7 8 9 10); shift ;;
        layout)     ITEMS+=(L1 L2 L3 L4); shift ;;
        [Ll][1-4])  ITEMS+=("L${1:1:1}"); shift ;;
        [0-9]|10)   ITEMS+=("$1"); shift ;;
        *) echo "[ERROR] 모르는 인자: $1" >&2; echo "       ./scripts/watch.sh --help" >&2; exit 2 ;;
    esac
done

if [ ${#ITEMS[@]} -eq 0 ]; then
    listing
    exit 0
fi

n=0
for item in "${ITEMS[@]}"; do
    n=$((n+1))
    if [ ${#ITEMS[@]} -gt 1 ]; then
        echo
        echo "[watch] ${n}/${#ITEMS[@]}"
    fi
    case "$item" in
        L*) play_layout "$item" ;;
        *)  play_episode "$item" ;;
    esac
done
echo
echo "[watch] 다 봤습니다. 목록은 ./scripts/watch.sh"

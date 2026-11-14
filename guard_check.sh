#!/usr/bin/env bash
# guard_check.sh — 三方对比实验的守门检查
#
#   bash guard_check.sh
#
# 检查四个工作区的：分支 / HEAD / 脏文件 / SH_MODE / simfile 指向 /
# 是否越界改了 pmpc_core。只读，不改任何文件。
#
# 退出码：0 = 全部通过，1 = 有违规。

set -u

CODE="D:/OneDrive - mail.scut.edu.cn/Thesis/Code/Controller"
FAIL=0

# 期望值：工作区 | 分支 | SH_MODE | SH_I | dlc 数据集 | slalom 数据集 | fishhook 数据集
EXPECT=(
  "MAIN|jturn-current|0|-|Run_4d669775-9558-4a06-a4ef-d6a3994def34|Run_fb630ab5-84ec-4ba0-953f-264cba8ad32c|Run_81631180-fa65-4272-a318-93dc477b6e33"
  "Baseline|ctrl-baseline|2|1.0|Run_bl_dlc|Run_bl_slalom|Run_bl_fishhook"
  "Proposed_noComp|ctrl-nocomp|0|-|Run_nc_dlc|Run_nc_slalom|Run_nc_fishhook"
  "Proposed_Comp|ctrl-comp|0|-|Run_cp_dlc|Run_cp_slalom|Run_cp_fishhook"
)

red()  { printf '\033[31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[32m%s\033[0m\n' "$*"; }
ylw()  { printf '\033[33m%s\033[0m\n' "$*"; }

bad() { red "  ✗ $*"; FAIL=1; }
ok()  { grn "  ✓ $*"; }
warn(){ ylw "  ! $*"; }

dataset_of() {
  [ -f "$1" ] || { echo "(缺失)"; return; }
  grep -o 'Run_[A-Za-z0-9_-]*' "$1" 2>/dev/null | head -1
}

echo "=============================================="
echo " 三方对比实验 · 守门检查"
echo " $(date '+%Y-%m-%d %H:%M:%S')"
echo "=============================================="

for row in "${EXPECT[@]}"; do
  IFS='|' read -r WS BR SHM SHI DLC SLA FH <<< "$row"
  DIR="$CODE/$WS"

  echo
  echo "── $WS  ($BR) ──────────────────────────"

  if [ ! -d "$DIR" ]; then bad "工作区不存在：$DIR"; continue; fi
  cd "$DIR" || { bad "无法进入 $DIR"; continue; }

  # 1. 分支
  CUR=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
  [ "$CUR" = "$BR" ] && ok "分支 $CUR" || bad "分支是 $CUR，应为 $BR"

  # 2. HEAD
  echo "  HEAD $(git log --oneline -1)"

  # 3. SH_MODE
  INV="nlcsnn/inv_current_impl_10ms.m"
  if [ -f "$INV" ]; then
    GOT=$(grep -oE '^\s*SH_MODE\s*=\s*[0-9]+' "$INV" | grep -oE '[0-9]+$')
    if [ "$GOT" = "$SHM" ]; then
      ok "SH_MODE = $GOT"
    else
      bad "SH_MODE = $GOT，应为 $SHM"
    fi
    if [ "$SHM" = "2" ]; then
      GOTI=$(grep -oE '^\s*SH_I\s*=\s*[0-9.]+' "$INV" | grep -oE '[0-9.]+$')
      [ "$GOTI" = "$SHI" ] && ok "SH_I = $GOTI" || bad "SH_I = $GOTI，应为 $SHI"
    fi
  else
    warn "找不到 $INV"
  fi

  # 4. simfile 指向
  for pair in "simfile_dlc.sim:$DLC" "simfile_slalom.sim:$SLA" "simfile.sim:$FH"; do
    F="${pair%%:*}"; WANT="${pair##*:}"
    GOT=$(dataset_of "$F")
    if [ "$GOT" = "$WANT" ]; then
      ok "$F -> $GOT"
    elif [ "$GOT" = "(缺失)" ]; then
      warn "$F 不存在"
    else
      bad "$F -> $GOT，应为 $WANT"
    fi
  done

  # 5. 越界改 pmpc_core
  if [ "$WS" = "Baseline" ]; then
    N=$(git status --porcelain -- pmpc_core/ | wc -l)
    [ "$N" -eq 0 ] && ok "pmpc_core/ 未改（Baseline 不该碰）" \
                   || bad "Baseline 改了 pmpc_core/ 的 $N 个文件"
  fi
  if [ "$WS" = "MAIN" ]; then
    N=$(git status --porcelain -- nlcsnn/ | wc -l)
    [ "$N" -eq 0 ] && ok "nlcsnn/ 未改" || warn "MAIN 改了 nlcsnn/ 的 $N 个文件"
  fi
  for w in Proposed_noComp Proposed_Comp; do
    if [ "$WS" = "$w" ]; then
      N=$(git status --porcelain -- nlcsnn/ init_valve2.m | wc -l)
      [ "$N" -eq 0 ] && ok "nlcsnn/ + init_valve2.m 未改" \
                     || bad "$WS 改了禁区文件（$N 个）"
    fi
  done

  # 6. 脏文件
  DIRTY=$(git status --porcelain | grep -vE '\.(vs|vsb)$' | grep -vE '^\?\? slprj/' || true)
  if [ -z "$DIRTY" ]; then
    ok "工作区干净"
  else
    warn "有未提交改动："
    echo "$DIRTY" | sed 's/^/      /' | head -12
    [ "$(echo "$DIRTY" | wc -l)" -gt 12 ] && echo "      ..."
  fi
done

echo
echo "=============================================="
if [ "$FAIL" -eq 0 ]; then
  grn " 全部通过"
else
  red " 有违规项，见上方 ✗"
fi
echo "=============================================="
exit "$FAIL"

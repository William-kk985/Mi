#!/bin/bash
# ============================================================
# sync_to_nx.sh — 安全同步到 NX（保护伙伴未同步的改动）
#
# 为什么需要这个脚本：
#   · 直接 `rsync -avz --delete` 会以本地为准"镜像"NX
#     → 会【删除/覆盖】NX 上伙伴改了但没 git 同步的内容
#     （历史教训：.github/ 被删；伙伴改在已有文件里的部分会被整文件覆盖）
#
# 安全原则：
#   1. 不带 --delete          → 绝不删除 NX 上任何多出的文件
#   2. 保护未提交修改的文件  → NX 上 git 状态为 modified 的文件【跳过不覆盖】
#                              伙伴改在哪些文件里，那些文件就原样保留
#   3. --exclude '.github/'   → 不碰 NX 仓库的 .github 配置
#   4. 检查被删除的文件       → 提示可用 git checkout 恢复
#
# 注意：
#   · 若伙伴改的文件【恰好也是你这次要改的】→ 会被跳过、传不过去，
#     清单里会列出来，你看到后用 scp 单文件手动合并推送即可。
#   · 治本方案：伙伴改完 commit + push，你 git pull 合并，再部署。
#
# 用法：bash sync_to_nx.sh
# ============================================================
set -e

# NX 连接别名: 有线=cyberdog(192.168.44.1), WiFi=cyberdog-wifi(10.179.x)
# build_race.sh 会自动探测并 export NX_HOST; 独立用时默认有线
NX_HOST="${NX_HOST:-cyberdog}"
LOCAL="/home/kaka/Mi/cyberdog_sim/src/cyberdog_simulator/cyberdog_race"
REMOTE="$NX_HOST:/SDCARD/race_ws/src/cyberdog_race"
SSH_OPTS="-o StrictHostKeyChecking=no -o ConnectTimeout=20 -o ServerAliveInterval=10"
NX_REPO="/SDCARD/race_ws/src/cyberdog_race"

MOD_LIST=$(mktemp)
MOD_RSYNC=$(mktemp)
trap 'rm -f "$MOD_LIST" "$MOD_RSYNC"' EXIT

echo "=============================================="
echo "  安全同步: VM → NX"
echo "  源:   $LOCAL"
echo "  目标: $REMOTE"
echo "=============================================="

echo ""
echo "=== 1. NX 上未提交改动总览（伙伴或本地的） ==="
ssh $SSH_OPTS "$NX_HOST" \
  "cd $NX_REPO && git status --short 2>/dev/null | head -30 || echo '(非 git 仓库)'"

echo ""
echo "=== 2. 收集 NX 上【已修改未提交】的文件 → 本次【跳过不覆盖】 ==="
ssh $SSH_OPTS "$NX_HOST" \
  "cd $NX_REPO && git status --porcelain 2>/dev/null | awk '\$1 ~ /M/ {print \$2}'" \
  > "$MOD_LIST" || true

if [ -s "$MOD_LIST" ]; then
  echo "  以下文件在 NX 上被修改且未提交，已保护（不覆盖，保留伙伴版本）："
  sed 's/^/    · /' "$MOD_LIST"
  # 转成 rsync 规则：根相对路径，避免误匹配同名文件
  sed 's|^|/|' "$MOD_LIST" > "$MOD_RSYNC"
else
  echo "  （无 —— NX 上没有未提交的修改，可放心全覆盖）"
  : > "$MOD_RSYNC"
fi

echo ""
echo "=== 3. 检查 NX 上被删除的文件（可用 git 恢复） ==="
DELETED=$(ssh $SSH_OPTS "$NX_HOST" \
  "cd $NX_REPO && git status --porcelain 2>/dev/null | awk '\$1 ~ /D/ {print \$2}'" || true)
if [ -n "$DELETED" ]; then
  echo "  以下文件在 NX 上被删（可能之前误删），建议恢复："
  echo "$DELETED" | sed 's/^/    · /'
  echo "  恢复命令（在 NX 上执行）： git checkout -- <文件路径>"
else
  echo "  （无）"
fi

echo ""
echo "=== 4. 开始推送（跳过上述受保护文件） ==="
rsync -avz -e "ssh $SSH_OPTS" \
  --exclude '.github/' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude-from="$MOD_RSYNC" \
  "$LOCAL/" "$REMOTE/"

echo ""
echo "=== ✅ 同步完成 ==="
echo "  · 未删除 NX 上任何文件"
echo "  · 伙伴改过的文件被跳过（见第 2 步清单），改动保留"
echo "  · 若某受保护文件你也改了（需要推送），用 scp 单文件推送："
echo "      scp <本地文件> cyberdog:$NX_REPO/<路径>"
echo "  · 治本：伙伴 commit + push → 你 git pull 合并后再部署"

#!/usr/bin/env bash
# 批量编译各赛段组合版本并存档
# 用法: bash /SDCARD/race_bins/build_bins.sh 123456 1 2 3 4 5 6 12 34 56
# 每个版本: 编译 → save_bin.sh startrace<combo> → 二进制+run脚本
# 注意: 全部编完后 install 里留下的是最后一个版本, 建议最后编 123456
set -e
set -o pipefail   # (2026-08-21 修复: colcon|tail 管道会掩盖失败→存错二进制)

CONF=/SDCARD/race_ws/src/cyberdog_race/include/cyberdog_race/debug_config.hpp
[ -f "$CONF" ] || { echo "✗ 找不到 $CONF"; exit 1; }

source /etc/mi/ros2_env.conf
cd /SDCARD/race_ws

set_config() {
    local combo="$1"
    # 重置三行宏为注释 (幂等)
    sed -i \
        -e 's|^#define DEBUG_SINGLE_STAGE.*|// #define DEBUG_SINGLE_STAGE|' \
        -e 's|^#define DEBUG_START_STAGE.*|// #define DEBUG_START_STAGE|' \
        -e 's|^#define DEBUG_END_STAGE.*|// #define DEBUG_END_STAGE|' \
        -e 's|^#define DEBUG_STAGE4_NO_COMP.*|// #define DEBUG_STAGE4_NO_COMP|' \
        -e 's|^#define DEBUG_STAGE2_TEST.*|// #define DEBUG_STAGE2_TEST|' \
        -e 's|^#define DEBUG_STAGE4_TEST_ROUTE2.*|// #define DEBUG_STAGE4_TEST_ROUTE2|' \
        -e 's|^#define DEBUG_STAGE4_TEST_ROUTE.*|// #define DEBUG_STAGE4_TEST_ROUTE|' \
        "$CONF"
    case "$combo" in
        1|2|3|4|5|6)
            sed -i "s|^// #define DEBUG_SINGLE_STAGE.*|#define DEBUG_SINGLE_STAGE $combo|" "$CONF"
            ;;
        4test) sed -i -e 's|^// #define DEBUG_SINGLE_STAGE.*|#define DEBUG_SINGLE_STAGE 4|' \
                    -e 's|^// #define DEBUG_STAGE4_TEST_ROUTE.*|#define DEBUG_STAGE4_TEST_ROUTE|' "$CONF";;
        4test2) sed -i -e 's|^// #define DEBUG_SINGLE_STAGE.*|#define DEBUG_SINGLE_STAGE 4|' \
                     -e 's|^// #define DEBUG_STAGE4_TEST_ROUTE2.*|#define DEBUG_STAGE4_TEST_ROUTE2|' "$CONF";;
        2test) sed -i -e 's|^// #define DEBUG_SINGLE_STAGE.*|#define DEBUG_SINGLE_STAGE 2|' \
                    -e 's|^// #define DEBUG_STAGE2_TEST.*|#define DEBUG_STAGE2_TEST|' "$CONF";;
        12) sed -i -e 's|^// #define DEBUG_START_STAGE.*|#define DEBUG_START_STAGE 1|' \
                  -e 's|^// #define DEBUG_END_STAGE.*|#define DEBUG_END_STAGE 2|' "$CONF";;
        34) sed -i -e 's|^// #define DEBUG_START_STAGE.*|#define DEBUG_START_STAGE 3|' \
                  -e 's|^// #define DEBUG_END_STAGE.*|#define DEBUG_END_STAGE 4|' "$CONF";;
        34test) sed -i -e 's|^// #define DEBUG_START_STAGE.*|#define DEBUG_START_STAGE 3|' \
                     -e 's|^// #define DEBUG_END_STAGE.*|#define DEBUG_END_STAGE 4|' \
                     -e 's|^// #define DEBUG_STAGE4_TEST_ROUTE.*|#define DEBUG_STAGE4_TEST_ROUTE|' "$CONF";;
        34test2) sed -i -e 's|^// #define DEBUG_START_STAGE.*|#define DEBUG_START_STAGE 3|' \
                      -e 's|^// #define DEBUG_END_STAGE.*|#define DEBUG_END_STAGE 4|' \
                      -e 's|^// #define DEBUG_STAGE4_TEST_ROUTE2.*|#define DEBUG_STAGE4_TEST_ROUTE2|' "$CONF";;
        56) sed -i -e 's|^// #define DEBUG_START_STAGE.*|#define DEBUG_START_STAGE 5|' \
                  -e 's|^// #define DEBUG_END_STAGE.*|#define DEBUG_END_STAGE 6|' "$CONF";;
        123456) ;;   # 全注释=正式全跑
        123456test) sed -i -e 's|^// #define DEBUG_STAGE4_TEST_ROUTE.*|#define DEBUG_STAGE4_TEST_ROUTE|' "$CONF";;   # (2026-08-22 全跑+Stage4 test路线)
        123456test2) sed -i -e 's|^// #define DEBUG_STAGE4_TEST_ROUTE2.*|#define DEBUG_STAGE4_TEST_ROUTE2|' "$CONF";;   # (2026-08-22 全跑+Stage4 第三版路线)
        *) echo "✗ 未知组合 $combo"; exit 1;;
    esac
    grep -E "DEBUG_(SINGLE|START|END)_STAGE" "$CONF" | grep -v "^//" | grep -v "只跑\|开始\|结束" || echo "  (全部注释=正式全跑模式)"
}

for combo in "$@"; do
    echo "════════ 编译 startrace${combo} ════════"
    set_config "$combo"
    # (2026-08-21 修复: sed 改 debug_config.hpp 后 colcon 增量不重编→保存错位二进制,
    #  实测 startrace2/12/123456/2test/4test 全是一个 md5; touch 全部 cpp 强制重编)
    find /SDCARD/race_ws/src/cyberdog_race/src -name "*.cpp" -exec touch {} +
    MAKEFLAGS=-j6 colcon build --merge-install --packages-select cyberdog_race \
        --executor sequential 2>&1 | tail -2
    bash /SDCARD/race_bins/save_bin.sh "startrace${combo}"
    # (2026-08-22 防重复: 与已存版本md5相同=宏没生效/编译错位, 警告勿当独立版本)
    MY_MD5=$(cut -d' ' -f1 "/SDCARD/race_bins/race_controller_startrace${combo}.md5" 2>/dev/null || true)
    if [ -n "$MY_MD5" ]; then
        DUP=$(cd /SDCARD/race_bins && grep -l "^${MY_MD5}$" race_controller_startrace*.md5 2>/dev/null | grep -v "race_controller_startrace${combo}.md5" | head -1 || true)
        if [ -n "$DUP" ]; then
            echo "⚠⚠⚠ startrace${combo} 与 ${DUP%.md5} 内容完全相同! 宏可能没生效, 勿当独立版本用!"
        else
            echo "✓ md5独立: ${MY_MD5:0:8} (与其他版本不同)"
        fi
    fi
    echo ""
done
echo "════════ 全部完成, 现存版本: ════════"
ls /SDCARD/race_bins/ | grep -v ".sh\|.md5"

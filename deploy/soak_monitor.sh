#!/bin/sh
# 10-minute (600s) board soak & memory/MMZ/FD leak monitor for cv610_live_hand*.
# Usage: sh soak_monitor.sh [duration_sec=600] [interval_sec=10] [out_csv]

DURATION_SEC="${1:-600}"
INTERVAL_SEC="${2:-10}"
OUT_CSV="${3:-/mnt/share/soak_metrics_$(date '+%Y%m%d_%H%M%S' 2>/dev/null || echo 'run').csv}"

STOP_REQUESTED=0
SAMPLES=0
TARGET_PID=""
TARGET_CMD=""
INIT_RSS_KB=""
FINAL_RSS_KB=""
PEAK_RSS_KB=0
INIT_VSZ_KB=""
FINAL_VSZ_KB=""
INIT_FDS=""
FINAL_FDS=""
INIT_MMZ_KB=""
FINAL_MMZ_KB=""
PID_CHANGED=0
PROCESS_DIED=0

on_signal()
{
    STOP_REQUESTED=1
}

trap on_signal INT TERM

find_live_pid()
{
    found_pid=""
    for p in /proc/[0-9]*; do
        [ -r "$p/cmdline" ] || continue
        pid_num="${p##*/}"
        [ "$pid_num" = "$$" ] && continue
        cmd="$(tr '\000' '\n' < "$p/cmdline" 2>/dev/null | head -n 1)"
        case "$cmd" in
            *cv610_live_hand*)
                found_pid="$pid_num"
                break
                ;;
        esac
    done
    printf '%s' "$found_pid"
}

read_proc_status_field()
{
    status_file="/proc/$1/status"
    field="$2"
    [ -r "$status_file" ] || { printf '%s' "-1"; return; }
    awk -v key="${field}:" '$1 == key { print $2; found=1; exit } END { if (!found) print "-1" }' "$status_file" 2>/dev/null
}

read_meminfo_field()
{
    field="$1"
    [ -r /proc/meminfo ] || { printf '%s' "-1"; return; }
    awk -v key="${field}:" '$1 == key { print $2; found=1; exit } END { if (!found) print "-1" }' /proc/meminfo 2>/dev/null
}

read_open_fds()
{
    fd_dir="/proc/$1/fd"
    [ -d "$fd_dir" ] || { printf '%s' "-1"; return; }
    cnt=0
    for f in "$fd_dir"/*; do
        [ -e "$f" ] || [ -L "$f" ] && cnt=$((cnt + 1))
    done
    printf '%s' "$cnt"
}

read_mmz_free_kb()
{
    mmz_file=""
    if [ -r /proc/umap/media-mem ]; then
        mmz_file="/proc/umap/media-mem"
    elif [ -r /proc/media-mem ]; then
        mmz_file="/proc/media-mem"
    else
        printf '%s' "-1"
        return
    fi

    awk '
    tolower($0) ~ /(remain|free)/ {
        for (i = 1; i <= NF; i++) {
            tok = tolower($i)
            prev = (i > 1) ? tolower($(i - 1)) : ""
            if (tok ~ /(remain|free)[=:]/ || prev ~ /(remain|free)[=:]?$/) {
                val = tok
                sub(/^.*[:=]/, "", val)
                if (val ~ /^[0-9]+kb?$/) {
                    sub(/kb?$/, "", val)
                    free_kb += val + 0
                    found = 1
                } else if (val ~ /^[0-9]+mb?$/) {
                    sub(/mb?$/, "", val)
                    free_kb += (val + 0) * 1024
                    found = 1
                } else if (val ~ /^[0-9]+$/) {
                    free_kb += val + 0
                    found = 1
                }
            }
        }
    }
    END { if (found) printf "%d", free_kb; else print "-1" }
    ' "$mmz_file" 2>/dev/null
}

OUT_DIR="${OUT_CSV%/*}"
if [ "$OUT_DIR" != "$OUT_CSV" ] && [ ! -d "$OUT_DIR" ]; then
    mkdir -p "$OUT_DIR" 2>/dev/null || OUT_CSV="/tmp/soak_metrics.csv"
fi

if ! printf 'timestamp_iso,elapsed_s,pid,vm_rss_kb,vm_vsz_kb,threads,open_fds,mem_free_kb,mem_avail_kb,mmz_free_kb\n' > "$OUT_CSV" 2>/dev/null; then
    OUT_CSV="/tmp/soak_metrics.csv"
    printf 'timestamp_iso,elapsed_s,pid,vm_rss_kb,vm_vsz_kb,threads,open_fds,mem_free_kb,mem_avail_kb,mmz_free_kb\n' > "$OUT_CSV"
fi

TARGET_PID="$(find_live_pid)"
if [ -z "$TARGET_PID" ]; then
    printf 'ERROR: no running cv610_live_hand* process found in /proc\n' >&2
    exit 1
fi
TARGET_CMD="$(tr '\000' ' ' < "/proc/$TARGET_PID/cmdline" 2>/dev/null)"

printf 'soak_monitor: pid=%s duration=%ss interval=%ss csv=%s\n' \
    "$TARGET_PID" "$DURATION_SEC" "$INTERVAL_SEC" "$OUT_CSV"
printf 'soak_monitor: cmd=%s\n' "$TARGET_CMD"

START_EPOCH="$(date +%s 2>/dev/null || printf '0')"
ELAPSED=0

while [ "$STOP_REQUESTED" -eq 0 ] && [ "$ELAPSED" -le "$DURATION_SEC" ]; do
    NOW_PID="$(find_live_pid)"
    NOW_EPOCH="$(date +%s 2>/dev/null || printf '%s' "$ELAPSED")"
    if [ "$START_EPOCH" -gt 0 ] 2>/dev/null && [ "$NOW_EPOCH" -ge "$START_EPOCH" ] 2>/dev/null; then
        ELAPSED=$((NOW_EPOCH - START_EPOCH))
    fi
    TS_ISO="$(date '+%Y-%m-%dT%H:%M:%S' 2>/dev/null || printf 't+%ss' "$ELAPSED")"

    if [ -z "$NOW_PID" ]; then
        PROCESS_DIED=1
        printf '[soak] t=%ss ERROR process exited unexpectedly\n' "$ELAPSED" >&2
        break
    fi
    if [ "$NOW_PID" != "$TARGET_PID" ]; then
        PID_CHANGED=1
        printf '[soak] t=%ss WARN pid changed %s -> %s\n' "$ELAPSED" "$TARGET_PID" "$NOW_PID" >&2
        TARGET_PID="$NOW_PID"
    fi

    RSS_KB="$(read_proc_status_field "$NOW_PID" "VmRSS")"
    VSZ_KB="$(read_proc_status_field "$NOW_PID" "VmSize")"
    THREADS="$(read_proc_status_field "$NOW_PID" "Threads")"
    OPEN_FDS="$(read_open_fds "$NOW_PID")"
    MEM_FREE="$(read_meminfo_field "MemFree")"
    MEM_AVAIL="$(read_meminfo_field "MemAvailable")"
    MMZ_FREE="$(read_mmz_free_kb)"

    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "$TS_ISO" "$ELAPSED" "$NOW_PID" "$RSS_KB" "$VSZ_KB" "$THREADS" \
        "$OPEN_FDS" "$MEM_FREE" "$MEM_AVAIL" "$MMZ_FREE" >> "$OUT_CSV"

    printf '[soak] t=%ss pid=%s rss=%skB vsz=%skB thr=%s fds=%s mem_free=%skB mmz_free=%skB\n' \
        "$ELAPSED" "$NOW_PID" "$RSS_KB" "$VSZ_KB" "$THREADS" "$OPEN_FDS" "$MEM_FREE" "$MMZ_FREE"

    SAMPLES=$((SAMPLES + 1))
    if [ -z "$INIT_RSS_KB" ] && [ "$RSS_KB" -ge 0 ] 2>/dev/null; then
        INIT_RSS_KB="$RSS_KB"
        INIT_VSZ_KB="$VSZ_KB"
        INIT_FDS="$OPEN_FDS"
        INIT_MMZ_KB="$MMZ_FREE"
    fi
    if [ "$RSS_KB" -ge 0 ] 2>/dev/null; then
        FINAL_RSS_KB="$RSS_KB"
        if [ "$RSS_KB" -gt "$PEAK_RSS_KB" ]; then
            PEAK_RSS_KB="$RSS_KB"
        fi
    fi
    FINAL_VSZ_KB="$VSZ_KB"
    FINAL_FDS="$OPEN_FDS"
    FINAL_MMZ_KB="$MMZ_FREE"

    [ "$ELAPSED" -ge "$DURATION_SEC" ] && break
    sleep "$INTERVAL_SEC" || break
    if [ "$START_EPOCH" -eq 0 ] 2>/dev/null; then
        ELAPSED=$((ELAPSED + INTERVAL_SEC))
    else
        NOW_AFTER="$(date +%s 2>/dev/null || printf '0')"
        ELAPSED=$((NOW_AFTER - START_EPOCH))
    fi
done

RSS_DELTA_KB="N/A"
FD_DELTA="N/A"
MMZ_DELTA_KB="N/A"
VERDICT="PASS"

if [ -n "$INIT_RSS_KB" ] && [ -n "$FINAL_RSS_KB" ] && [ "$INIT_RSS_KB" -ge 0 ] 2>/dev/null; then
    RSS_DELTA_KB=$((FINAL_RSS_KB - INIT_RSS_KB))
fi
if [ -n "$INIT_FDS" ] && [ -n "$FINAL_FDS" ] && [ "$INIT_FDS" -ge 0 ] 2>/dev/null && [ "$FINAL_FDS" -ge 0 ] 2>/dev/null; then
    FD_DELTA=$((FINAL_FDS - INIT_FDS))
fi
if [ -n "$INIT_MMZ_KB" ] && [ -n "$FINAL_MMZ_KB" ] && [ "$INIT_MMZ_KB" -ge 0 ] 2>/dev/null && [ "$FINAL_MMZ_KB" -ge 0 ] 2>/dev/null; then
    MMZ_DELTA_KB=$((FINAL_MMZ_KB - INIT_MMZ_KB))
fi

if [ "$PROCESS_DIED" -ne 0 ] || [ "$PID_CHANGED" -ne 0 ] || [ "$SAMPLES" -eq 0 ]; then
    VERDICT="FAIL"
elif [ "$RSS_DELTA_KB" != "N/A" ] && [ "$RSS_DELTA_KB" -gt 4096 ] 2>/dev/null; then
    VERDICT="FAIL"
elif [ "$FD_DELTA" != "N/A" ] && [ "$FD_DELTA" -gt 4 ] 2>/dev/null; then
    VERDICT="FAIL"
fi

printf '\n### Soak Monitor Summary\n\n'
printf '| 指标 | 数值 |\n|---|---|\n'
printf '| 采样点数 | %s |\n' "$SAMPLES"
printf '| PID 连续性 | pid_changed=%s, process_died=%s |\n' "$PID_CHANGED" "$PROCESS_DIED"
printf '| VmRSS (初始 / 峰值 / 结束) | %s / %s / %s kB (delta=%s kB) |\n' \
    "${INIT_RSS_KB:-N/A}" "$PEAK_RSS_KB" "${FINAL_RSS_KB:-N/A}" "$RSS_DELTA_KB"
printf '| VmSize (初始 / 结束) | %s / %s kB |\n' \
    "${INIT_VSZ_KB:-N/A}" "${FINAL_VSZ_KB:-N/A}"
printf '| Open FDs (初始 / 结束) | %s / %s (delta=%s) |\n' \
    "${INIT_FDS:-N/A}" "${FINAL_FDS:-N/A}" "$FD_DELTA"
printf '| MMZ Free (初始 / 结束) | %s / %s kB (delta=%s kB) |\n' \
    "${INIT_MMZ_KB:-N/A}" "${FINAL_MMZ_KB:-N/A}" "$MMZ_DELTA_KB"
printf '| CSV 记录 | %s |\n' "$OUT_CSV"
printf '\nSOAK_RESULT=%s\n' "$VERDICT"

[ "$VERDICT" = "PASS" ]

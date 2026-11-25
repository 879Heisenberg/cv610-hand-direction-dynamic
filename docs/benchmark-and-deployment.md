# 实机评测、离线消融与长稳验证指南

本文档记录 `hand_direction` 的板端实测性能、真实采集 CSV 连续流消融报告、离线回放工具用法及长稳监控方案。

---

## 1. Host 自动化测试与基准验证

在 WSL (`Ubuntu-22.04`) 中执行 clean Release 构建与单元测试（`-Wall -Wextra -Wpedantic -O3` 零告警）：

```bash
cmake -S . -B build-host -DCMAKE_BUILD_TYPE=Release
cmake --build build-host --clean-first --parallel
ctest --test-dir build-host --output-on-failure -V
```

`ctest` 共 5 项用例全部通过：
1. `direction`：方向判定、速度/位移门槛、起划停留修剪、快速反切回中回归测试；
2. `gesture_engine`：中心 ROI 门控、激活确认、2000 ms 冷却期遮蔽回中、丢检复位测试；
3. `preprocess`：NV21/RGB Letterbox 变换及 NEON/标量一致性精度校验；
4. `postprocess`：3549 候选框布局校验、NMS 与坐标逆变换校验；
5. `replay_bench`：`hand_direction_bench --self-test`（130 条合成片段消融 + 10 次上划/冷却回中连续流 + CSV 往返校验）。

### `hand_direction_bench --built-in` 合成基准结果（130 片段：`80` 正 + `50` 负）

| 指标 | `legacy_v1`（旧基线） | `current_dirv4`（当前） |
|---|---:|---:|
| 正样本总准确率 | 50.0% (40/80) | **100.0% (80/80)** |
| `UP / DOWN / LEFT / RIGHT` 召回率 | 55% / 55% / 45% / 45% | **100% / 100% / 100% / 100%** |
| 正样本漏触发率 | 50.0% (40/80) | **0.0% (0/80)** |
| 负样本误触发率 | 40.0% (20/50) | **0.0% (0/50)** |
| 触发延迟 P50 / P95 / Max (ms) | 266 / 266 / 266 | **190 / 266 / 266** |

---

## 2. CV610 实机连续流消融测试

### 2.1 板端 CSV 录制方法（“框内立即回中”连续流）
板端程序支持通过环境变量录制 `640x360` 推理逐帧检测结果（需等待 `Ctrl+C` 正常退出以确保文件缓冲刷新落盘）：

```sh
cd /root/hand_direction && HAND_DIRECTION_RECORD_PATH=/mnt/share/board_up.csv HAND_DIRECTION_RECORD_LABEL=UP ./bin/cv610_live_hand ./model/hand_detect_static.om
```

测试动作节奏：手在中心 `80% × 60%` ROI 框内静止 -> 干脆向目标方向划动 -> 屏幕出现触发反馈并进入 `2000 ms` 琥珀色 `COOL` 冷却条时**立即回中** -> 保持静止等待恢复 `READY` 后开始下一轮，每方向连续 10 次。

### 2.2 实机采集消融结果（2026-11-24 实测，40 次正样本 + 33.4 s 负样本流）
* 评测 ROI：`(0.330, 0.200) - (0.667, 0.800)`，横纵比校正 `x_scale=1.778`。

| 指标 | `legacy_v1`（旧基线） | `current_dirv4`（当前） |
|---|---:|---:|
| 正样本总准确率 | 70.0% (28/40) | **90.0% (36/40)** |
| UP 召回率 | 70.0% (7/10) | **80.0% (8/10)** |
| DOWN 召回率 | 10.0% (1/10) | **90.0% (9/10)** |
| LEFT 召回率 | 100.0% (10/10) | **90.0% (9/10)** |
| RIGHT 召回率 | 100.0% (10/10) | **100.0% (10/10)** |
| 正样本漏触发率 | 30.0% (12/40) | **10.0% (4/40)** |
| 回中反向误触发 (`Return FP`) | 9 | **1** |
| 其他误向触发 | 0 | **0** |
| 负样本误触发 (次 / 次·分) | 4 (7.19/min) | **1 (1.80/min)** |
| 超额重复触发数 | 0 | **0** |
| 有效划动时长 P50 / P95 / Max (ms) | 607 / 640 / 644 | **270 / 571 / 671** |

---

## 3. CV610 交叉编译与板端长稳监控

### 3.1 交叉编译板端应用（WSL `Ubuntu-22.04`）
```bash
cmake -S . -B build-cv610-live \
  -DCMAKE_TOOLCHAIN_FILE=/root/work/hi_aiot_solution/hi_aiot_solution/toolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DHAND_DIRECTION_BUILD_TESTS=OFF \
  -DHAND_DIRECTION_ENABLE_CV610=ON \
  -DHAND_DIRECTION_SDK_ROOT=/root/work/hi_aiot_solution/hi_aiot_solution/sdk/Hi3516CV610/soc \
  -DHAND_DIRECTION_CV610_SAMPLE_COMM_DIR=/root/work/hi_aiot_solution/hi_aiot_solution/apps/cv610_ev_board/samples/capture_picture/comm \
  -DHAND_DIRECTION_DRM_ROOT=/root/work/hi_aiot_solution/hi_aiot_solution/_build/open_source/drm/release
cmake --build build-cv610-live --target cv610_live_hand cv610_gesture_sound cv610_smoke --parallel 4
```

### 3.2 10 分钟长稳泄漏监控 (`deploy/soak_monitor.sh`)
在 `cv610_live_hand` 运行期间，于板端串口或后台运行：
```sh
sh /mnt/share/soak_monitor.sh 600 10 /mnt/share/soak_metrics.csv
```
脚本使用纯 POSIX / BusyBox `ash` 兼容语法周期采样 `VmRSS`、`VmSize`、打开文件描述符数（FD）、`/proc/umap/media-mem` MMZ 占用以及进程是否重启，最终输出汇总结论与 CSV 明细。

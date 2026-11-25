# CV610 LCD 显示与固定手势动效接口说明

> 适用工程：`work/hand_direction`  
> 当前版本：检测到 UP/DOWN/LEFT/RIGHT 后播放固定方向动画，不再绘制真实手部轨迹。  
> 用途：交给动效模块的重构 agent，作为接口边界、行为契约和验收依据。

## 1. 本次行为基线

- 方向识别至少需要 **5 个有效采样点**。第 5 点满足方向、位移、速度、直线度条件后才产生事件。
- 前 4 个有效点只建立识别窗口，不产生方向事件。
- 显示层不再接收或绘制真实手部轨迹点。
- 每次收到一个方向事件，播放一段约 **1000 ms** 的固定屏幕坐标动画。
- 动画路径只由方向决定，与手在画面中的真实位置无关。
- 检测框、最高置信度、手部中心点和方向文字继续保留。
- 连续复用同一事件快照时，不重启动画；动画进度由事件时间戳唯一决定。

## 2. 相关源码

| 文件 | 职责 |
|---|---|
| `app/display_drm.h` | 屏幕显示模块的唯一公开接口 |
| `app/display_drm.c` | DRM/LCD、NV21 转 RGB565、检测框、中心点、固定路径动画和文字绘制 |
| `app/live_main.c` | 摄像头主循环、推理线程结果复制、`hand_display_overlay` 组装 |
| `include/hand_direction/gesture_engine.h` | 手势状态机、事件快照和动画时长配置 |
| `include/hand_direction/types.h` | `hand_point`、`hand_detection`、`hand_gesture_event` |
| `include/hand_direction/direction.h` | 方向识别内部窗口；仅供识别器使用，不进入显示层 |
| `include/hand_direction/trajectory.h` | 识别器内部采样点缓存；不属于显示接口 |
| `app/live_media.h` | 摄像头输出固定为 `640x360 NV21` |

关键实现入口：

- `hand_display_open()`
- `hand_display_render()`
- `hand_display_close()`
- `draw_detections()`
- `draw_gesture_overlay()`
- `gesture_animation_path()`
- `draw_gesture_arrow()`

建议重构时保留函数职责边界，避免把识别状态机、摄像头和网络逻辑带入显示模块。

## 3. 当前数据流

```text
OS04D10
  -> VI / ISP / VPSS
  -> 640x360 NV21
  -> live_main 主线程
       |-- 每帧渲染 LCD
       |
       `-- 每 2 个摄像头帧提交一次推理
             -> YOLO / NPU 检测
             -> 手势引擎
             -> 5 点方向识别
             -> 生成方向事件
             -> 在 mutex 下发布 latest 快照

live_main 主线程
  -> 复制 latest 检测框和 gesture snapshot
  -> 组装 hand_display_overlay
  -> hand_display_render()
       -> NV21 中心裁剪并转 RGB565
       -> 绘制检测框
       -> 绘制固定方向动画
       -> 绘制手部中心点
       -> DRM atomic commit
  -> JD9853 240x320 LCD
```

推理线程拥有检测器、手势引擎和方向识别窗口。主线程只拥有摄像头帧、overlay 临时对象、DRM 双缓冲和 LCD 提交。

## 4. 手势事件产生规则

默认参数来自 `hand_direction_config_init_defaults()`：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `window_ms` | 667 ms | 识别采样时间窗 |
| `max_gap_ms` | 250 ms | 最大采样间隔 |
| `min_samples` | 5 | 最少有效采样点 |
| `min_span_ms` | 250 ms | 首尾点最小时间跨度 |
| `min_displacement` | 0.10 | 归一化最小位移 |
| `min_speed_per_second` | 0.20 | 最小速度 |
| `min_linearity` | 0.80 | 位移与路径长度比 |
| `axis_ratio` | 1.50 | 主轴相对次轴的最小倍数 |

手势引擎默认参数：

| 参数 | 默认值 |
|---|---:|
| `activation_class_id` | `0` |
| `activation_confidence` | `0.60` |
| `lost_reset_ms` | 250 ms |
| `cooldown_ms` | 600 ms |
| `event_animation_ms` | 1000 ms |

状态简要：

```text
WAITING_FOR_HAND
  -> 检测到满足类别和置信度的手
  -> WAITING_FOR_ACTIVATION
  -> 首次有效手部位置直接进入 CAPTURING
  -> 第 5 个有效采样点满足方向条件
  -> 产生 hand_gesture_event
  -> COOLDOWN
```

手部丢失超过 `lost_reset_ms` 时：

1. 立即重置 tracker 和方向识别窗口。
2. 状态回到 `WAITING_FOR_HAND`。
3. 若最近方向事件仍在动画窗口内，事件快照继续保留，供显示动画播完。
4. 动画过期后清除事件快照。

## 5. 公开显示接口

`app/display_drm.h` 当前接口：

```c
typedef struct hand_display hand_display;

typedef struct {
    int center_valid;
    hand_point center;
    int event_valid;
    hand_gesture_event event;
    uint64_t event_animation_ms;
} hand_display_overlay;

int hand_display_open(hand_display **display,
                      char *error,
                      size_t error_size);

int hand_display_render(hand_display *display,
                        const uint8_t *nv21,
                        uint32_t width,
                        uint32_t height,
                        const hand_detection *detections,
                        size_t detection_count,
                        const hand_display_overlay *overlay,
                        char *error,
                        size_t error_size);

void hand_display_close(hand_display *display);
```

接口中没有以下字段：

```text
trajectory
trajectory_count
display_trajectory
```

识别器内部仍保留采样缓存用于方向计算，但它不会穿过显示接口。

## 6. Overlay 字段契约

| 字段 | 来源 | 显示规则 |
|---|---|---|
| `center_valid` | 手势快照 `has_center` | 为真且坐标合法时绘制白色中心点 |
| `center` | 平滑后检测框中心 | 使用完整摄像头画面的归一化坐标 |
| `event_valid` | 手势快照 `event_visible` | 为真时允许显示方向文字和固定动画 |
| `event.direction` | `hand_gesture_event` | UP、DOWN、LEFT、RIGHT；NONE 不绘制 |
| `event.timestamp_ms` | 事件产生时的摄像头时间戳 | 动画唯一时钟，必须非 0 |
| `event_animation_ms` | 手势引擎配置 | 动画总时长；0 时显示层回退到 1000 ms |

`overlay == NULL` 合法，表示当前没有手势 HUD 数据。

## 7. 数据所有权

| 数据 | 所有者 | 显示模块权限 | 生命周期 |
|---|---|---|---|
| `nv21` | 主线程 `camera_frame` | 只读 | 到本次 `hand_display_render()` 返回 |
| `detections[]` | 主线程 `latest` 快照 | 只读 | 到本次渲染返回 |
| `overlay` | 主线程栈变量 | 只读 | 到本次渲染返回 |
| `event`、`center` | 已复制进 overlay | 只读值 | 到本次渲染返回 |
| DRM map、framebuffer、blob | `hand_display` | 读写 | open 到 close |

显示模块不得跨帧保存 `nv21`、`detections` 或 `overlay` 指针。

## 8. 坐标与裁剪

- 摄像头源帧固定为 `640x360 NV21`，但接口仍通过 `width/height` 传入实际尺寸。
- LCD 为竖屏 `240x320`。
- 背景从源画面中心裁出与 LCD 同宽高比的区域，再缩放到 `240x320`。
- 检测框、手部中心点使用**完整摄像头画面**的归一化坐标，再由显示层映射到裁剪区域。
- 固定方向动画直接使用 LCD 屏幕坐标，不受摄像头裁剪影响。

改分辨率时，检测框和中心点应继续遵循同一归一化坐标契约；固定动画坐标需按新的 LCD 尺寸重新设计。

## 9. 固定方向动画路径

路径定义为 `240x320` 屏幕坐标：

| 方向 | 起点 | 终点 | 视觉含义 |
|---|---|---|---|
| UP | `(120,270)` | `(120,50)` | 从下向上滑动 |
| DOWN | `(120,50)` | `(120,270)` | 从上向下滑动 |
| LEFT | `(205,160)` | `(35,160)` | 从右向左滑动 |
| RIGHT | `(35,160)` | `(205,160)` | 从左向右滑动 |

路径是固定预设，不读取事件中的真实手部坐标。事件中的 `x/y` 不用于动画路径，只保留为识别结果元数据。

## 10. 动画时间曲线

动画总时长：

```c
animation_ms = overlay->event_animation_ms != 0
                   ? overlay->event_animation_ms
                   : 1000;
```

滑动阶段：

```c
travel_ms = min(animation_ms, 650);
```

时间段行为：

| 时间 | 行为 |
|---|---|
| `age < 650 ms` | 头部从起点移动到终点，尾迹长度约 22%，透明度为 255 |
| `650 ms <= age < 1000 ms` | 轨迹停在终点并线性淡出 |
| `age >= 1000 ms` | 不绘制方向动画 |
| `now < event.timestamp_ms` | 不绘制，防止错误时钟域导致越界动画 |

滑动使用 smoothstep：

```c
p = t * t * (3 - 2 * t);
```

尾部进度：

```c
tail = max(0, head - 0.22)
```

绘制组成：

- 7 像素青色辉光拖尾。
- 3 像素青色主线。
- 白色箭头位于当前头部。
- 白色圆点位于箭头尖端。
- 淡出阶段整条路径和箭头一起降低 Alpha。

## 11. 动画时钟与重复快照

- `event.timestamp_ms` 来自 `CLOCK_MONOTONIC` 对应的摄像头捕获时间。
- 显示层使用 `CLOCK_MONOTONIC` 的当前时间计算 `age_ms`。
- 同一个事件在多个摄像头帧中被重复传入时，时间戳不变，因此动画进度连续。
- 新方向事件必须使用新时间戳，才能触发新动画。
- 当前显示模块不保存 `last_event_timestamp`，动画是无状态计算。

如果重构后引入动效状态对象，必须保留“同一时间戳只触发一次”的规则。

## 12. 每帧绘制顺序

`hand_display_render()` 每帧执行：

1. 从 NV21 中心裁剪、旋转/缩放为竖屏 RGB565 背景。
2. 绘制检测框。
3. 绘制固定方向动画。
4. 绘制手部中心点。
5. 提交 DRM atomic frame。

检测信息规则：

- 所有检测框使用青色。
- 最高置信度检测框改为黄色。
- 最佳检测框附近显示 `HAND x.xx`。
- 没有检测框时在左上角显示 `NO HAND`。
- 方向文字固定在左侧 `(4,30)`，避免覆盖左上角检测信息。
- 手部中心点为 3 像素黑色底、1 像素白色芯。

## 13. 线程和生命周期

- 当前所有显示调用都在 `live_main` 主线程完成。
- 推理线程只发布检测和手势快照，不直接访问 LCD。
- `hand_display` 内部没有锁。
- 不得在 `hand_display_render()` 和 `hand_display_close()` 之间并发访问。
- 显示初始化失败时，摄像头和推理仍需继续运行。
- 渲染失败时，`live_main` 关闭显示对象，但推理循环继续。

如引入异步动效线程，必须同时设计：

- back buffer 所有权；
- DRM 扫描期间禁止覆盖；
- frame queue 或三缓冲；
- stop/join 顺序；
- `hand_display_close()` 与渲染线程的退出同步。

第一阶段建议保持同步渲染和无状态动画，不要仅增加一个线程共享当前 front buffer。

## 14. 重构边界

动效模块可以修改：

- 路径形状、箭头样式、拖尾长度、颜色、缓动函数和淡出曲线；
- 检测框、中心点和文字的视觉样式；
- 动效内部状态组织；
- 离屏画布或图层合成方式。

动效模块不应修改：

- YOLO 检测和 NMS；
- 5 点方向识别阈值；
- 手势状态机冷却时间；
- 摄像头 VI/ISP/VPSS 通路；
- 网络、NFS、自启动和部署脚本。

若必须修改公开 ABI，需同步修改 `app/live_main.c`，并说明调用方变更。

## 15. 日志和调试

| 环境变量 | 作用 |
|---|---|
| `HAND_DRM_DEVICE` | 覆盖 DRM 设备，默认 `/dev/dri/card0` |
| `HAND_DIRECTION_DISABLE_DISPLAY` | 非空时禁用 LCD |
| `HAND_DIRECTION_TRACE_FRAMES` | 非空且非 `0` 时输出帧跟踪 |

`live_main` 每 30 帧输出：

```text
camera_fps
display_ms
infer_ms
prep_ms
flush_ms
npu_ms
post_ms
detections
gesture_events
```

固定路径动画不应明显影响 `infer_ms`、`prep_ms` 和 `npu_ms`；重点观察 `display_ms`。

## 16. 构建与测试

Host 构建：

```powershell
wsl.exe bash -lc "cd '/mnt/c/Users/32774/Documents/Codex/2026-11-13/https-docs-hisilicon-com-projects-hi/work/hand_direction' && cmake --build build-host --parallel && ctest --test-dir build-host --output-on-failure"
```

CV610 交叉编译：

```powershell
wsl.exe bash -lc "cd '/mnt/c/Users/32774/Documents/Codex/2026-11-13/https-docs-hisilicon-com-projects-hi/work/hand_direction' && cmake --build build-cv610-live --parallel"
```

CV610 产物：

```text
build-cv610-live/cv610_live_hand
```

Host 测试只覆盖识别和状态机逻辑，不编译 `display_drm.c`，因此不能替代 CV610 交叉编译和板端视觉验收。

## 17. 板端运行

```sh
killall -TERM cv610_live_hand 2>/dev/null
sleep 1
if pidof cv610_live_hand >/dev/null 2>&1; then
    killall -KILL cv610_live_hand 2>/dev/null
fi

cd /root/hand_direction
export LD_LIBRARY_PATH=/lib:/usr/lib
export HAND_DRM_DEVICE=/dev/dri/card0
./bin/cv610_live_hand ./model/hand_detect_static.om
```

启动日志应包含：

```text
min_samples=5
direction_window=667 ms
animation=1000 ms
```

## 18. 板端视觉验收

- [ ] LCD 正常点亮，方向为竖屏，无上下翻转或左右错位。
- [ ] 摄像头背景中心裁剪比例正确。
- [ ] 无手时显示 `NO HAND`。
- [ ] 检测框位置与画面一致，最高置信度框为黄色。
- [ ] `HAND x.xx` 可读且不超出屏幕。
- [ ] 前 4 个有效采样点不产生方向事件。
- [ ] 第 5 个有效采样点满足条件后才产生方向事件。
- [ ] UP 动画固定从下向上播放。
- [ ] DOWN 动画固定从上向下播放。
- [ ] LEFT 动画固定从右向左播放。
- [ ] RIGHT 动画固定从左向右播放。
- [ ] 屏幕中不再出现真实手部轨迹点。
- [ ] 快速移动或检测框跳变时没有跨屏飞线。
- [ ] 同一事件被多个显示帧复用时动画连续，不重复重启。
- [ ] 动画约 1 秒后自动淡出，不残留无限拖尾。
- [ ] 手部丢失后识别窗口已重置，但正在播放的方向动画可以播完。
- [ ] 连续两次方向事件之间遵守 600 ms 冷却。
- [ ] `display_ms` 没有不可接受的增长。
- [ ] 停止程序后 LCD、映射、framebuffer 和 DRM fd 正确释放。
- [ ] 显示初始化失败时，摄像头和推理继续运行。
- [ ] 连续运行 10 分钟以上无卡死、花屏、缓冲冲突或内存持续增长。

## 19. 动效重构交付时必须说明

1. 是否修改公开 ABI；若修改，`live_main.c` 如何适配。
2. 动画状态保存在哪里，生命周期由谁管理。
3. 事件上升沿如何判断，重复快照如何避免重启。
4. 使用哪些固定路径、时钟域、缓动和淡出参数。
5. 是否新增离屏画布，以及每帧额外内存带宽。
6. Host 测试与 CV610 交叉编译结果。
7. 板端 `display_ms` 与当前版本对比。
8. 第 18 节验收项逐项结果。

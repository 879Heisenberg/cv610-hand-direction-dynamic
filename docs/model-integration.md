# 模型接入与端侧推理管线说明

本文档说明 Hi3516CV610 实机检测后端（`src/detector_cv610.c`）、NEON 预处理（`src/preprocess.c`）、后处理（`src/postprocess.c`）与静态模型导出规范的当前实现。

---

## 1. 模型与算子契约

* **板端运行时模型**：[model/hand_detect_static.om](../model/hand_detect_static.om)（已随仓库保留，输入 `images: [1, 3, 416, 416]` FP32 NCHW，输出 `output0: [1, 5, 3549]`）。
* **模型导出脚本**：[model/export_cv610_static.py](../model/export_cv610_static.py) 从端到端 YOLO26 checkpoint 导出静态 ONNX（`opset=19`, `batch=1`, `nms=False`, `end2end=False`），显式校验并剔除 CV610 不支持或高耗时算子（`TopK`、`GatherElements`、`Mod`）。
* **模型角色**：只负责单类别手掌/手部检测（`class_id = 0`），**不负责**方向分类；四方向与防误触由 C 模块结合运动几何和状态机完成。

---

## 2. 检测后端实现 (`src/detector_cv610.c`)

当编译启用 `-DHAND_DIRECTION_ENABLE_CV610=ON` 时，工程编入 `src/detector_cv610.c`（主机仅编译 `-DHAND_DIRECTION_ENABLE_CV610=OFF` 时使用 `src/detector_cv610_placeholder.c` 占位）。

`svp_acl` 推理流程如下：
1. 初始化 `svp_acl` 设备上下文，加载 `.om` 模型并解析描述（3 个输入：`image`、`task`、`work`；1 个输出：`[1, 5, 3549]`）。
2. 在首次（或输入分辨率变化时）预初始化 Letterbox 填充区域及 ACL 行 stride 尾部，避免每帧清零整个 `3x416x416` tensor。
3. 调用 `src/preprocess.c` 将 `640x360 NV21` 转换为 `416x416 FP32 NCHW` 写入 NPU 输入内存并执行 cache flush。
4. 同步执行 NPU 推理，记录分阶段耗时：`prep_ms`、`flush_ms`、`npu_ms`、`post_ms`。
5. 调用 `src/postprocess.c` 解析 `3549` 个候选框并完成置信度过滤、NMS（默认 `conf >= 0.25`, `nms_iou <= 0.45`）和 Letterbox 坐标逆变换，输出归一化 `[0, 1]` 检测框。

---

## 3. CPU 预处理优化 (`src/preprocess.c`)

`640x360 NV21 -> 416x416 FP32 NCHW` 热点路径包含两级优化：
* **标量快速路径**：针对 NV21 专用转换、按输出行预计算 Y/VU 指针、直接写入 R/G/B 三个平面、仅填充有效图像区之外的 Letterbox padding。
* **ARMv7 NEON 加速路径**：在 `-mcpu=cortex-a7 -mfloat-abi=softfp -mfpu=neon-vfpv4 -O3` 下自动启用，采用**双行 RGB 行缓存**对源图每行仅做一次 YUV->RGB 转换供双线性插值复用，每次向量操作处理 8 像素并附标量尾部处理，数值与标量路径完全一致（主机对比 `519,168/519,168` FP32 像素零误差），使 CV610 板端 `prep_ms` 降至 **`14.3–17.5 ms`（稳态 `~15.4 ms`）**。

---

## 4. 识别管线数据流

```text
hand_frame (640x360 NV21)
  -> hand_detector (svp_acl + preprocess + postprocess)
  -> hand_detection[] (normalized [0, 1])
  -> hand_tracker (time-scaled gate margin + speed-adaptive EMA)
  -> hand_direction (dirv4: dwell trim, x_scale=1.778, half_cosine, segment consistency)
  -> hand_gesture_engine (80%x60% center ROI + 2000 ms cooldown)
  -> hand_gesture_event -> LCD HUD Overlay + 4-Dir WAV Audio / Console / CSV Record
```

> 注：输出事件中的 `confidence` 为几何方向与检测联合可信度；在未做画面水平镜像的摄像头视角下，算法中 `LEFT` / `RIGHT` 对应相机图像坐标的 `-X` / `+X` 方向（与 LCD 预览屏幕上的左右一致）。

# CPU 预处理优化 v4

## 本轮针对的实际路径

摄像头输入：640x360 NV21（live_media 配置）
模型输入：416x416 FP32 NCHW
主要步骤：NV21 -> RGB -> 双线性缩放 -> 归一化 -> letterbox 填充。

## 修改

1. NV21 增加专用快速路径，避免每个目标像素反复经过通用 pixel-format switch。
2. 每个输出行预先计算 Y/VU 行指针，减少内层循环中的乘法和地址计算。
3. 直接写入 R/G/B 三个 NCHW plane，移除每像素 channel 循环和重复 plane 地址计算。
4. Letterbox 只初始化真正的 padding 区域及 ACL 行 stride 尾部，不再先清空整个 3x416x416 tensor 再覆盖图像区域。
5. CMake 默认构建类型改为 Release；核心预处理库和 CV610 detector 显式启用 -O3。即使旧 build cache 是 Debug，热路径也会获得 -O3。
6. live 状态日志增加 prep_ms / flush_ms / npu_ms / post_ms，便于区分 CPU 预处理、ACL cache flush、NPU 和后处理耗时。

## 正确性检查

对同一份随机 640x360 NV21 输入，优化前后完整 3x416x416 FP32 输出逐元素比较：

- 最大绝对误差：5.96046e-08（浮点运算顺序误差）
- 超过 1e-7 的差异数量：0

原有 CTest：4/4 通过。Debug 配置下也确认核心库编译参数包含 -O3。

## 主机侧微基准（仅作相对参考）

同一 x86 主机、640x360 NV21、200 次预热后平均 200 次：

| 版本 | 编译选项 | 平均预处理耗时 |
|---|---:|---:|
| 原版 | -O3 | 2.235 ms |
| 优化版 | -O3 | 1.390 ms |
| 原版 | -O0 | 6.531 ms |
| 优化版 | -O0 | 4.910 ms |

主机数据不能代替 CV610 实测。板端耗时受 CPU 主频、编译器、内存带宽和 cache 影响；请以日志里的 prep_ms 为准。

## 板端建议

重新编译 CV610 版本并运行，连续观察至少 100 帧的 prep_ms、flush_ms、npu_ms。重点确认 prep_ms 是否从原先 40ms+ 明显下降。如果 prep_ms 仍高，再根据实际 Cortex 核心型号和编译器版本考虑 NEON SIMD 或 VPSS 硬件预处理，不建议在没有测量的情况下直接改变模型输入颜色/插值语义。

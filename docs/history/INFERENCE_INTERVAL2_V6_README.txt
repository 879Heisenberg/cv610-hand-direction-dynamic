推理间隔调整补丁 v6
====================

本补丁基于 v5 NEON 预处理优化版本，仅将：
#define LIVE_INFERENCE_INTERVAL 3U
调整为：
#define LIVE_INFERENCE_INTERVAL 2U

其他推理、预处理、NPU、轨迹窗口及方向识别参数保持不变。

安装：将压缩包内 hand_direction/ 目录覆盖到工程根目录对应位置。
本补丁包含 v5 的相关源码修改，是累积版本。

验证：PC Release 构建通过，CTest 4/4 通过。
CV610 板端性能和手势稳定性仍需实机测试。

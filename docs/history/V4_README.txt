手势识别 CPU 预处理优化补丁 v4（累积版）

本补丁包含此前 v3 轨迹状态/显示修复，并新增 NV21 CPU 预处理优化、CMake -O3 和分阶段耗时日志。
将 hand_direction/ 内文件覆盖到原工程同名路径即可，不需要单独应用 v2/v3。

请先备份原工程。板端性能必须以 CV610 实测日志为准。详细说明见 PREPROCESS_OPTIMIZATION_NOTES.md。

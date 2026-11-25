# 模型目录说明 (`model/`)

本目录存放 Hi3516CV610 NPU 运行时模型与静态图导出工具链：

* [hand_detect_static.om](hand_detect_static.om)：CV610 SVP_NNN 离线模型（已纳入 Git 方便开箱部署，输入 `1x3x416x416` FP32 NCHW，输出 `1x5x3549`）。
* [hand_detect_static_desc.json](hand_detect_static_desc.json)：`.om` 模型结构描述。
* [export_cv610_static.py](export_cv610_static.py) & [hand_yolo26n_416_static.json](hand_yolo26n_416_static.json)：将 YOLO26n `.pt` 权重导出为无 `TopK/GatherElements/Mod` 静态 ONNX（`nms=False, end2end=False`）的脚本与导出元数据。
* `calibration_416/` & `cv610_calibration_images.txt`：ATC/NPU 量化校准图集与列表（12 张 416×416 图像）。

> `.onnx`、`calibration_416_nchw_fp32.bin` 与早期旧模型 `hand_detect.om` 已在 `.gitignore` 中排除。若在仅验证主机侧 `stub` / 离线基准测试，无需加载 `.om` 文件。

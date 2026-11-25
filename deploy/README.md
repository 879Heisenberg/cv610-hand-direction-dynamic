# CV610 LCD 运行部署与镜像打包说明

`cv610_live_hand` 通过 DRM 将 `OS04D10` 相机画面、手部检测框、轨迹尾迹与交互 HUD 渲染到 JD9853 240x320 SPI LCD，并通过独立子进程 `cv610_gesture_sound` 播报 4 方向 WAV 音效。

## 整包烧录镜像打包（WSL）

```bash
cd /mnt/c/Users/32774/Documents/Codex/2026-11-13/https-docs-hisilicon-com-projects-hi/work/hand_direction
chmod +x deploy/package_gui_image.sh deploy/S99hand_direction deploy/soak_monitor.sh
./deploy/package_gui_image.sh
```

脚本会编译 GUI BSP、交叉编译 `cv610_live_hand` 与 `cv610_gesture_sound`、向 rootfs 注入 `/root/hand_direction/{bin,model,audio}` 与开机自启脚本，并在 `outputs/gui_hand_direction_display/` 下输出完整 UBIFS/JFFS2 烧录文件及 `SHA256SUMS`。

## 开机自启顺序 (`/etc/init.d/S99hand_direction`)

1. 拉起 `lo` 与 `eth0`、启动 Dropbear SSH；
2. 加载 MPP 内核模块与管脚复用（`load3516cv610_20s_debug -i`）；
3. 启动 `/root/sdk_server` 并加载 `jd9853.ko` 及可选触摸驱动；
4. 启动 `/root/hand_direction/bin/cv610_live_hand /root/hand_direction/model/hand_detect_static.om`；
5. 配合 `/etc/init.d/S99zz-cv610-net` 在后台配置 `192.168.2.100/24` 并软挂载（`soft`）PC NFS `192.168.2.10:/Share`，确保 PC 关机或网线断开时不阻塞串口与启动。

> 注：原厂 `S99video_ui` 已在打包脚本中被移除，避免抢占 `/dev/dri/card0`。手动调试替换二进制时，请用 `killall cv610_live_hand`（**不要加 `-9`**）或 `Ctrl+C` 退出后再重启进程，确保 kernel ISP 状态干净释放。

## 板端自检与预期 HUD 状态

```sh
cat /tmp/hand_direction.log
cat /tmp/hand_direction_mpp_load.log
lsmod | grep -E 'jd9853|sys_config'
ps | grep -E 'sdk_server|cv610_live_hand'
ls -l /dev/dri/card0
```

* 画面中央带半透明 **`80% × 60%` ROI 引导框**（边框颜色随引擎状态切换）；
* 激活手势准备完成时状态栏显示绿色 **`READY`**；
* 触发四方向手势后，显示 **`UP` / `DOWN` / `LEFT` / `RIGHT` 方向大箭头动画（1000 ms）** 并伴随对应 WAV 音效，同时状态栏显示琥珀色 **`COOL` 进度条（2000 ms）**，此时可将手立即移回框中心准备下一轮。

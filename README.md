# ai-cam · 工业相机采集

> 详细的安装、虚拟相机配置、采集流程、代码说明、问题排查和术语解释见 [docs/cam_demo开发手册.md](docs/cam_demo开发手册.md)。


工业相机（GigE Vision / USB3 Vision）采集链路的完整示例：设备 SDK 接入、触发、帧号与时间戳、
帧池与有界队列、异步写盘、状态机与断线重连、录像校验、分层丢帧统计。没有真实相机时，
用 Aravis 虚拟 GigE 相机、海康虚拟相机或内置模拟相机运行。

## 目录

```
include/frame_queue.h   有界线程安全队列（采集端非阻塞 try_push，消费端带超时 pop）
include/frame.h         帧结构、帧内存池（复用大块内存）、CRC32
include/recorder.h      写盘线程：帧号连续性检查、CRC、按大小滚动文件、写盘耗时统计
src/sim_capture.cpp     模拟相机 + 采集状态机 + 断线重连（无需硬件，可直接跑）
src/aravis_capture.cpp  Aravis（开源 GenICam）真机版
src/mvs_capture.cpp     海康机器人 MVS SDK 真机版
src/uvc_capture.cpp     OpenCV 读普通 USB 摄像头（UVC），可在 Linux / Windows 编译
tools/verify_record.py  录制文件校验：CRC、丢帧、帧间隔 p50/p99、导出 PGM/PPM 图片
```

## 编译运行

```bash
mkdir build && cd build
cmake .. && make -j
./sim_capture 10 100 1280 1024            # 正常
./sim_capture 10 200 2048 1536 8000       # 模拟慢盘：观察队列打满、丢帧计数
python3 ../tools/verify_record.py rec/rec_*.bin
```

Aravis：`sudo apt install libaravis-dev aravis-tools libusb-1.0-0-dev`，`./aravis_capture list` 列出相机。
MVS：官网下载 Linux 版 SDK 装到 /opt/MVS，运行前设置 `LD_LIBRARY_PATH`。
无相机测试：Aravis 运行 `arv-fake-gv-camera-0.8 -i 127.0.0.1` 启动虚拟 GigE 相机；
MVS 用 `/opt/MVS/bin/VirtualDeviceTool.sh`（图形界面）添加虚拟相机，如 MV-CS060-10GM（黑白）/ MV-CS060-10GC-PRO（彩色）；
自定义画面：把与相机同分辨率的 bmp/jpg 放进 `/var/tmp/VirtualCamera/Cameras/<序列号>/Mono8` 或 `RGB24`，采集时循环播放。
`./mvs_capture 5 none 5000 RGB8Packed 10GC`：参数依次为 秒数、触发、曝光、像素格式、相机（序列号或型号片段）。
采集控制界面：`python3 tools/mvs_gui.py`（PyQt5 + MVS Python SDK）——选相机、设置格式/曝光/增益/帧率/触发、实时预览、录制 .bin；
录像查看：`python3 tools/qt_demo.py rec_0000.bin`。需要 `sudo apt install python3-pyqt5`，WSL 下通过 WSLg 显示窗口。
OpenCV：`sudo apt install libopencv-dev`，
`./uvc_capture 10 0 1280 720 30 ./rec_uvc --show`（`--gray` 存灰度）。
没有摄像头时可传视频文件：`./uvc_capture 10 test.mp4`，按视频帧率读取并循环播放。
WSL2 默认看不到 USB 摄像头，需要 usbipd-win 转发 + 带 uvcvideo 驱动的内核；
也可以在 Windows 上用 OpenCV 直接编译运行这个程序。
Aravis / MVS 两个真机版本未在真机上验证，特性名以相机手册 / SDK Samples 为准。

## 数据流

```
相机 ──(SDK 回调/取帧线程)──> 帧池取缓冲 ─ memcpy+元数据 ─ try_push ─┐
                                                                     │ 有界队列(64)
状态机线程: 断开→连接→配置→采集→出错→退避重连                        ▼
统计线程: 每秒打印 fps/MB/s/队列水位/丢帧          写盘线程: 校验帧号→CRC→fwrite→滚动
```

## 设计要点

1. **采集线程永不阻塞**：队列满时直接丢弃并计数，而不是等待。否则相机/SDK 内部缓存溢出，
   丢得更多且不可控。可选策略：丢最新（本例）/ 丢最旧 / 事件期间不丢、平时降采样。
2. **帧池**：高帧率下每帧 new 几 MB 会造成内存碎片和延迟抖动；池子耗尽本身就是"下游积压"的信号。
3. **SDK 缓冲必须及时归还**（`arv_stream_push_buffer` / `MV_CC_FreeImageBuffer`），
   回调或取帧线程里只拷贝，不做算法和 I/O。
4. **三种"丢帧"要分开统计**：网络丢包残帧（SDK 状态/nLostPacket）、帧号跳变（相机侧或传输丢）、
   软件队列满。分开才能定位瓶颈在相机、网络还是磁盘。
5. **时间戳**：用相机硬件时间戳（Chunk / buffer timestamp）对齐，多相机用 PTP 同步；
   主机时间戳只用来算传输延迟。
6. **带宽估算**：2048×1536 Mono8 @200fps ≈ 630MB/s，超过千兆网(~110MB/s)，
   需要万兆、ROI/Binning 或 USB3/CoaXPress；写盘需要 NVMe 或 RAID。
7. **GigE 调优**：巨帧 MTU 9000、增大网卡 ring buffer（`ethtool -G`）、
   增大 socket 接收缓冲（`net.core.rmem_max`）、采集线程绑核/提高优先级。
8. **断线重连**：异常回调只置标志，由状态机线程释放资源并指数退避重连；
   重连后帧号可能从 0 开始，写盘侧要记录"会话号"来区分。

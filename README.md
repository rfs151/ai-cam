# ai-cam · 工业相机采集

工业相机（GigE Vision / USB3 Vision）采集链路的完整示例：设备 SDK 接入、触发、帧号与时间戳、帧池与有界队列、
异步写盘、状态机与断线重连、录像校验、分层丢帧统计。没有真实相机时，用 Aravis 虚拟 GigE 相机、
海康虚拟相机或内置模拟相机运行。

在线演示：<http://47.117.29.74:8098/>

**亮点**

- **分层丢帧统计**：网络包 → 拼帧 → SDK 缓冲 → 帧号 → 程序队列 → 文件，每层单独计数，页面实时显示是哪一层在丢
- **故障注入**：网页上一键拔网线、设置丢包率、让取帧线程或磁盘变慢，看采集端怎么反应
- **实测**：512×512 每帧 193 个包，丢包 2% 时完整帧几乎为 0（0.98¹⁹³ ≈ 2%）；改用 8000 字节巨帧后每帧 132 个包，完整帧回到约 7%
- **断线重连**：2 秒内发现掉线，按 100 ms → 5 s 指数退避重连，相机恢复后约 1 秒继续采集并重新下发参数
- **数据可校验**：录像每帧 48 字节帧头 + CRC32；慢磁盘录 8 秒时队列丢 81 帧，离线校验报告缺约 80 帧，两边对得上
- **C++17 采集引擎** `capd`：取帧线程只拷贝和归还缓冲，录像走帧池 + 有界队列 + 写盘线程，预览 libjpeg 编码

```
浏览器 ◀──WebSocket（状态 / 统计 / JPEG 预览）──▶ web/api（FastAPI）
                                                    │ stdin 命令 / stdout JSON
                                                    ▼
arv-fake-gv-camera ──GVCP 控制 + GVSP 图像（UDP）──▶ capd（C++）──帧池 → 队列 → 写盘线程 → .bin
  GV01 @ 127.0.0.1                                   │
  GV02 @ eth0                                        └─ 录完运行 tools/verify_record.py 校验
```

> 详细的安装、虚拟相机配置、采集流程、代码说明、问题排查和术语解释见 [docs/cam_demo开发手册.md](docs/cam_demo开发手册.md)。

## 网页演示

| 区域 | 内容 |
|---|---|
| 枚举相机 | GVCP 广播发现两台虚拟 GigE 相机（回环和 eth0 网卡各一台） |
| 参数 | 像素格式、分辨率、曝光、帧率、连续 / 软触发、包大小、SDK 缓冲块数 |
| 实时画面 | 预览、帧率、帧号（能看到 16 位帧号 65535 → 1 绕回）、采集状态机 |
| 故障注入 | 拔网线、丢包率 0～5%、取帧线程每帧额外耗时、磁盘每帧额外耗时 |
| 丢帧漏斗 | 六层计数（累计和最近 1 秒），自动给出诊断：比如“每帧 193 个包，完整帧概率 (1−p)^n ≈ …” |
| 录像校验 | 录 3～10 秒，自动校验 CRC、帧号缺口、帧间隔 p50 / p99 |

公网演示同一时间只有一个人能操作（其他人旁观），3 分钟没有操作自动释放；没有访客 10 分钟后恢复默认设置。

### 运行

```bash
docker compose up -d --build          # 页面：http://127.0.0.1:8098（./ctl.sh start / status / logs）
```

国内网络在 `.env` 里设置 `REGISTRY=docker.1ms.run`、`APT_MIRROR=http://mirrors.aliyun.com`（见 `.env.example`）。
不用 Docker 时：`sudo apt install libaravis-dev aravis-tools libjpeg-turbo8-dev python3-fastapi python3-uvicorn python3-websockets`，
编译后 `./ctl.sh dev`。

`capd` 也可以单独用，每行一条命令：`list`、`open GV01`、`start`、`set size 1024x1024`、`record 5 /tmp/rec`……
输出每行一个 JSON（命令和消息格式见 `src/capd.cpp` 开头）。

## 代码结构

```
src/capd.cpp            网页演示用的常驻采集引擎（stdin 命令 / stdout JSON，断线重连，故障注入）
web/api/app.py          网页后端：管理 capd 和虚拟相机进程、WebSocket、控制权
web/static/             演示页面（原生 JS，无外部依赖）
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

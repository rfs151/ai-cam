# cam_demo 工业相机采集开发手册

> 整理自 2026-11-07 ~ 11-08 的开发会话。内容包括：环境安装、四种虚拟/模拟相机的配置与启动、采集流程、各程序的处理逻辑、丢帧统计体系、测试评估方法、常见问题及修复、术语解释。
> 运行环境：Windows + WSL2（Ubuntu 24.04），项目目录 `/root/proj/cam_demo/cam_demo`。

---

## 目录

1. [项目概览](#1-项目概览)
2. [环境安装](#2-环境安装)
3. [编译](#3-编译)
4. [没有真实相机时的五种数据来源](#4-没有真实相机时的五种数据来源)
5. [相机采集的通用流程](#5-相机采集的通用流程)
6. [各程序的处理逻辑与代码说明](#6-各程序的处理逻辑与代码说明)
7. [丢帧统计体系](#7-丢帧统计体系)
8. [测试与评估方法](#8-测试与评估方法)
9. [常见问题与修复](#9-常见问题与修复)
10. [视觉检测系统的搭建](#10-视觉检测系统的搭建)
11. [术语表](#11-术语表)
12. [后续可做的工作](#12-后续可做的工作)

---

## 1. 项目概览

### 1.1 目录结构

```
cam_demo/
├── CMakeLists.txt          按检测结果有条件地编译 4 个采集程序
├── README.md               简要说明
├── docs/
│   └── cam_demo开发手册.md  本文档
├── include/
│   ├── frame.h             帧结构、帧内存池、CRC32、帧号绕回判断 frame_id_missing()
│   ├── frame_queue.h       有界线程安全队列（采集端非阻塞入队，消费端带超时出队）
│   └── recorder.h          写盘线程：帧号检查、CRC、按 512MiB 滚动文件、写盘耗时统计
├── src/
│   ├── sim_capture.cpp     纯软件模拟相机 + 状态机 + 断线重连（不需要任何硬件或 SDK）
│   ├── aravis_capture.cpp  Aravis（开源 GenICam 库）采集，支持 GigE Vision / USB3 Vision
│   ├── mvs_capture.cpp     海康机器人 MVS SDK 采集
│   └── uvc_capture.cpp     OpenCV 采集：普通 USB 摄像头 / 视频文件 / RTSP
├── tools/
│   ├── verify_record.py    录制文件校验：CRC、丢帧、帧间隔；导出单帧图片
│   ├── qt_demo.py          PyQt5 窗口：WSLg 显示测试 + 录像回放
│   └── mvs_gui.py          PyQt5 + 海康 Python SDK 的采集控制界面
└── build/                  CMake 编译目录（含 compile_commands.json）
```

### 1.2 统一的数据流

四个 C++ 采集程序和 `mvs_gui.py` 都采用同一种结构：

```
相机/数据源 ──(SDK 线程或取帧循环)──▶ 帧池取缓冲 ─ memcpy + 元数据 ─ try_push ─┐
                                        │ 池空 → drop pool                        │ 队列满 → drop queue
                                        ▼                                         ▼
                                    帧号检查（链路丢帧）              有界队列（容量 64）
                                                                                  │ pop_for(200ms)
                                                                                  ▼
                                                  写盘线程：帧号检查 → 48 字节帧头 + CRC32 → 写文件（512MiB 滚动）
                                                                                  │
统计线程：每秒打印 fps / MB/s / 队列水位 / 各类丢帧 ◀───────────────────────────────┘
```

设计原则：
- **采集端绝不阻塞**：队列满就丢帧并计数，不让下游的慢拖住相机，否则相机端缓存溢出会整批丢帧。
- **控制与数据分离**：状态机线程只管连接、配置、监视，图像数据不经过它。
- **SDK 回调里只做轻活**：拷贝、入队、设标志；停止、重连这类操作交给控制线程。
- **统一录制格式**：所有程序录下的 `.bin` 都能用同一个工具校验和回放。

### 1.3 程序一览

| 程序 | 数据来源 | 依赖 | 主要用途 |
|---|---|---|---|
| `sim_capture` | 程序内存生成（全黑，首字节为帧号） | 无 | 练习和测试帧池、队列、写盘、丢帧统计、断线重连 |
| `aravis_capture` | GigE/USB3 Vision 相机，测试时用 Aravis 虚拟相机 | `libaravis-dev` | 开源方案采集工业相机 |
| `mvs_capture` | 海康相机，测试时用海康虚拟相机 | 海康 MVS SDK | 海康相机采集 |
| `uvc_capture` | 普通 USB 摄像头 / 视频文件 / RTSP | OpenCV | 原型开发；无相机时用视频文件模拟 |
| `mvs_gui.py` | 海康相机 / 虚拟相机 | PyQt5 + MVS Python SDK | 图形化采集控制 |
| `qt_demo.py` | `.bin` 录像 | PyQt5 | 录像回放、WSLg 显示测试 |
| `verify_record.py` | `.bin` 录像 | Python 3 | 离线校验、导出单帧 |

---

## 2. 环境安装

### 2.1 VS Code 的 C++ 跳转（Ctrl+Click 跳转到定义）

**现象**：在 VS Code 里 Ctrl+Click 方法名无法跳转。

**原因**：VS Code 通过 WSL 远程打开项目，WSL 这一侧没有装 C/C++ 语言扩展；项目也没有 `compile_commands.json`，扩展不知道头文件路径。

**修复**：
1. 在 WSL 侧安装微软 C/C++ 扩展（扩展面板搜 "C/C++"，点 **Install in WSL**），或者命令行：
   ```bash
   ~/.vscode-server/bin/*/bin/code-server --install-extension ms-vscode.cpptools
   ```
2. 生成编译数据库：
   ```bash
   cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
   ```
3. `.vscode/c_cpp_properties.json` 指向它：
   ```json
   {
     "configurations": [{
       "name": "Linux",
       "compileCommands": "${workspaceFolder}/build/compile_commands.json",
       "includePath": ["${workspaceFolder}/include", "/opt/MVS/include"],
       "compilerPath": "/usr/bin/g++",
       "cppStandard": "c++17",
       "intelliSenseMode": "linux-gcc-x64"
     }],
     "version": 4
   }
   ```
4. `Ctrl+Shift+P` → **Developer: Reload Window**。

> 每次新增 cpp 文件或新装 SDK 后，重新运行一次 cmake，`compile_commands.json` 才会包含新文件。

### 2.2 Aravis（开源 GenICam 库）

```bash
sudo apt install libaravis-dev aravis-tools libusb-1.0-0-dev
pkg-config --modversion aravis-0.8       # 应输出 0.8.30
```
- `aravis-tools` 提供 `arv-fake-gv-camera-0.8`（虚拟相机）、`arv-camera-test-0.8`（取流测试）、`arv-viewer-0.8`（图形查看器）。Ubuntu 的这个包里**没有** `arv-tool-0.8`。
- **必须装 `libusb-1.0-0-dev`**：`aravis-0.8.pc` 依赖它，不装的话 CMake 报 "Package 'libusb-1.0' not found"，判定 Aravis 未找到。
- 装完要**清掉 CMake 缓存**再配置：`rm build/CMakeCache.txt`。`pkg_check_modules` 会缓存失败结果。

### 2.3 海康 MVS SDK

1. 从海康机器人官网（服务支持 → 下载中心 → 机器视觉工业相机客户端 MVS）下载 Linux x86_64 版。可能需要登录或同意协议。本次用的是 `MVS-5.1.1_Linux_x86_64_20261016.zip`。
2. 解压（系统没有 `unzip` 时可以用 Python）：
   ```bash
   mkdir -p ~/mvs_pkg && python3 -c "import zipfile,sys;zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" \
       /mnt/c/Users/<用户名>/Downloads/MVS-5.1.1_Linux_x86_64_20261016.zip ~/mvs_pkg
   ```
   里面有 `.deb`、`.tar.gz` 和 `README`。
3. 安装 deb：
   ```bash
   sudo dpkg -i ~/mvs_pkg/MVS-5.1.1_x86_64_20261016.deb
   ```
   安装后：头文件 `/opt/MVS/include`，库 `/opt/MVS/lib/64`，Python 接口 `/opt/MVS/Samples/64/Python/MvImport`，文档 `/opt/MVS/doc`，客户端手册 `/opt/MVS/bin/User_Manual_of_Client_Software_Chinese.pdf`。
4. 卸载：`sudo dpkg -r MVS`（会删除 `/opt/MVS`）。

**安装脚本会改动的系统设置**（海康安装程序的常规操作，建议知晓）：
- 网络反向路径过滤 `rp_filter` 改为不校验，让 GigE 相机能正常收包。
- 调整 USB 和网络 socket 缓冲区大小。
- 往 shell 配置里加入 `MVCAM_SDK_PATH`、`MVCAM_COMMON_RUNENV`、`LD_LIBRARY_PATH` 等环境变量。
- 注册 GigE/PCIe 驱动和日志服务。WSL 里无法加载这些内核驱动，不影响 SDK 使用。
- 在用户主目录创建 `MVS.desktop` 快捷方式。
- 安装时 "Failed to restart network-manager.service" 的报错可以忽略：WSL 里没有这个服务。

### 2.4 OpenCV（用于 uvc_capture）

```bash
sudo apt install libopencv-dev           # 本次为 4.6.0，依赖包很多，安装需要十几分钟
```

### 2.5 PyQt5（图形界面）

```bash
sudo apt install python3-pyqt5           # Qt 5.15 / PyQt 5.15
```
- Ubuntu 24.04 默认禁止用 pip 往系统 Python 装包（PEP 668），所以用 apt 安装。
- 中文字体：系统已有文泉驿微米黑。没有的话装 `fonts-noto-cjk`，否则中文显示成方框。
- Qt6 / PySide6 常常还需要 `libxcb-cursor0`。

### 2.6 其它工具

```bash
sudo apt install ffmpeg                  # 生成测试视频和图片、图片格式转换
```

### 2.7 WSLg（在 Windows 桌面显示 Linux 窗口）

- Windows 11 自带；Windows 10 需要从 Microsoft Store 安装新版 WSL。**不需要**安装 Ubuntu 桌面或 X Server。
- WSL 里的 Qt / PyQt / tkinter / OpenCV `imshow` 窗口都会像普通 Windows 程序一样出现在任务栏上。
- 自己的终端里 `DISPLAY` 通常已经自动设好；某些环境下需要手动 `export DISPLAY=:0`。
- 开始菜单里的 "xxx (Ubuntu)" 快捷方式是 WSLg 根据 Linux 程序自动生成的。

---

## 3. 编译

```bash
cd /root/proj/cam_demo/cam_demo
cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j
```

CMake 会按检测结果决定编译哪些程序：

| 程序 | 条件 | 配置时的提示信息 |
|---|---|---|
| `sim_capture` | 总是编译 | — |
| `aravis_capture` | 找到 `aravis-0.8` | `Aravis found: build aravis_capture` |
| `mvs_capture` | 存在 `/opt/MVS/include/MvCameraControl.h` | `MVS SDK found: build mvs_capture` |
| `uvc_capture` | 找到 OpenCV | `OpenCV 4.6.0 found: build uvc_capture` |

> 新装了 SDK 之后：`rm build/CMakeCache.txt` 再重新运行 cmake。
> 如果你自己另建了编译目录（例如 `/root/proj/cam_demo/build`），代码更新后要在那个目录里重新 `make`。

---

## 4. 没有真实相机时的五种数据来源

| 来源 | 搭配的程序 | 画面内容 | 优点 | 局限 |
|---|---|---|---|---|
| 4.1 内置模拟相机 | `sim_capture` | 全黑（首字节为帧号） | 零依赖，能模拟丢包、断线、慢磁盘 | 画面没有内容 |
| 4.2 Aravis 虚拟相机 | `aravis_capture` | 512×512 斜向渐变，每帧平移 1 像素 | 走真实的 GigE Vision 网络协议 | 不能加载图片；不支持丢包重发 |
| 4.3 海康虚拟相机 | `mvs_capture`、`mvs_gui.py` | 默认测试条纹，或自己的 bmp/jpg 图片（循环播放） | 真实型号的参数表；能加载自己的图片 | 需要图形界面创建；没有硬件时间戳；每次打开都会重置参数 |
| 4.4 视频文件 | `uvc_capture` | 视频内容，按帧率循环播放 | 画面真实、随手可得 | 不是工业相机协议 |
| 4.5 USB 摄像头 | `uvc_capture` | 实时画面 | 真实的实时数据 | WSL2 默认访问不到 |

### 4.1 内置模拟相机（sim_capture）

不需要安装或启动任何东西：

```bash
./build/sim_capture [秒数=10] [fps=100] [宽=1280] [高=1024] [慢盘us=0] [输出目录=./rec]
./build/sim_capture 6                                  # 正常
./build/sim_capture 6 100 1280 1024 15000              # 模拟慢磁盘：每帧写盘多等 15ms
```

模拟相机的行为（`SimCamera` 类）：
- 前 2 次 `open()` 故意失败，用来演示指数退避重试。
- 第一轮采集 3 秒后模拟断线（调用断线回调），用来演示自动重连。
- 每帧有 0.2% 的概率"链路丢帧"：帧号照样加 1，但不调用出帧回调。
- 图像：分配 宽×高 字节的内存并填 0，每帧只把第一个字节改成帧号，让每帧 CRC 都不同。
- 随机数种子固定为 42，每次运行结果相同；重连后会从同一个种子重新开始，所以第二轮的丢帧位置和第一轮相同。

输出：运行命令时所在目录下的 `rec/rec_0000.bin`、`rec_0001.bin`……**每次运行会覆盖同名文件**，上次多出来的旧文件不会被删除。

### 4.2 Aravis 虚拟相机（arv-fake-gv-camera）

**启动**（另开一个终端；没有输出、一直停在前台即表示在线；`Ctrl+C` 关闭）：
```bash
arv-fake-gv-camera-0.8 -i 127.0.0.1
```

| 选项 | 作用 |
|---|---|
| `-i <地址或网卡名>` | 监听地址。必须是某个网卡上实际配置的地址 |
| `-s GV02` | 指定序列号 |
| `-r 20` | 模拟 GVSP 丢包，千分比（20 = 2%） |
| `-d all` | 打印调试日志 |

后台运行：
```bash
arv-fake-gv-camera-0.8 -i 127.0.0.1 &
pkill -x arv-fake-gv-cam                 # 进程名最多显示 15 个字符，所以是截断的
```

**检查是否已经启动**（三种方法，第 3 种最可靠）：
```bash
pgrep -a arv-fake                        # 1. 进程是否在运行
ss -ulnp | grep 3956                     # 2. GVCP 端口是否在监听，应看到 127.0.0.1:3956 和 255.255.255.255:3956 两行
./build/aravis_capture list              # 3. 能否被真正发现：found 1 camera(s)  [0] Aravis Fake  serial=GV01 ...
```

**采集**：
```bash
./build/aravis_capture 5 none                 # 连续采集 5 秒
./build/aravis_capture 5 soft                 # 软触发
./build/aravis_capture 5 none 5000 GV01       # 第 4 个参数选相机
```

**特点与限制**：
- 输出 512×512 Mono8 斜向渐变图案，每帧平移 1 像素。
- 帧号是 16 位的 GVSP block ID，每次启动后从 65400 左右开始，约 4 秒后绕回到 1。正好用来验证绕回处理。
- 不支持丢包重发（调试日志里显示 `Packet resend = no`），所以 `-r` 模拟丢包时坏帧补不回来。
- 没有自动曝光和 chunk 功能，`aravis_capture` 会打印两行 `[err] ... Not found`，属于正常现象。
- 在回环地址上必须禁用 packet socket 收包方式（程序已经自动处理，见 9.4）。
- **多台**：同一个地址上只能运行一台。要再启动一台，需要先给回环网卡加地址（修改系统网络配置，重启 WSL 后失效）：
  ```bash
  sudo ip addr add 127.0.0.2/8 dev lo
  arv-fake-gv-camera-0.8 -i 127.0.0.2 -s GV02
  sudo ip addr del 127.0.0.2/8 dev lo      # 用完删除
  ```
  （这个多相机方案本次会话没有实际验证。）

### 4.3 海康虚拟相机（VirtualDeviceTool）

**创建**：
1. 打开虚拟设备工具（图形界面，需要 WSLg）：
   ```bash
   DISPLAY=:0 /opt/MVS/bin/VirtualDeviceTool.sh
   ```
   也可以在 Windows 开始菜单里点 **VirtualDeviceTool (Ubuntu)**。
2. 在"虚拟相机"页面选择类型和型号，点"添加虚拟相机"。最多 256 台。
   - 本次建了两台：
     - `MV-CS060-10GM`（序列号 `Vir4050648`）：网口、黑白，3072×2048。
     - `MV-CS060-10GC-PRO`（序列号 `Vir4121971`）：网口、彩色，3072×2048。
   - 型号命名：`G` = GigE 网口，`U` = USB3；末尾 `M` = 黑白（Mono），`C` = 彩色（Color）。
3. 配置保存在 `/var/tmp/VirtualCamera/`：
   ```
   /var/tmp/VirtualCamera/
   ├── VirtualCameraConfig.ini       每台相机一节，如 [Vir4050648]：型号、IP、MAC、nTLayerType=1
   └── Cameras/
       └── Vir4050648/
           ├── MV-CS060-10GM.xml     GenICam 参数描述文件
           ├── MV-CS060-10GM_regfile.dat
           ├── Mono8/                自定义黑白图片（自己创建）
           └── RGB24/                自定义彩色图片（自己创建）
   ```
   `VirtualCameraConfig.ini` 的格式没有公开文档，**不要手工编辑**，请用工具创建。

**加载自己的图片**（出自海康客户端手册 6.8.2 节，已实测验证）：
1. 在 `/var/tmp/VirtualCamera/Cameras/<序列号>/` 下创建 `Mono8`（黑白）或 `RGB24`（彩色）文件夹，把图片放进去。
2. **图片分辨率必须和相机完全一致**（这两个型号都是 3072×2048），格式为 `.bmp` 或 `.jpg`。
3. 相机的像素格式要和文件夹对应：`Mono8` 文件夹 ↔ `Mono8`；`RGB24` 文件夹 ↔ `RGB8Packed`。
4. 开始采集后图片按顺序循环播放，但起始的那张不固定。
5. 用 ffmpeg 把普通照片转成需要的格式：
   ```bash
   ffmpeg -i 照片.jpg -vf scale=3072:2048 -pix_fmt gray  1.bmp     # 8 位灰度 BMP
   ffmpeg -i 照片.jpg -vf scale=3072:2048 -pix_fmt bgr24 1.bmp     # 24 位彩色 BMP
   ```

**不放图片时的默认画面**：黑白格式下是 8 条灰度横条；彩色或 Bayer 格式下是彩色横条。

**采集**：
```bash
./build/mvs_capture 5 none 5000 Mono8 10GM           # 黑白相机
./build/mvs_capture 5 none 5000 RGB8Packed 10GC      # 彩色相机，RGB 输出
./build/mvs_capture 5 soft 5000 Mono8 Vir4050648     # 软触发，按序列号选
python3 tools/mvs_gui.py                             # 图形界面
```

**特点与限制**：
- 虚拟相机对外报告的设备类型和真实网口相机一样（`nTLayerType=1`，即 `MV_GIGE_DEVICE`），程序无法区分它是不是虚拟的。
- **不提供硬件时间戳**（值为 0）。
- **每次打开都恢复默认参数**：触发模式回到 Off，触发延时 0，去抖 50μs。真实相机断电前会保留设置。
- 修改曝光、增益不会改变画面亮度（它只是播放图片）。
- 黑白型号的描述文件里虽然列出了 `RGB8Packed`，但标注为未实现，设置时 SDK 返回 `0x80000004`（参数错误）。
- 同一时刻只能被一个程序打开（独占）。
- 删除或重置虚拟相机，可能会清掉自定义图片。

### 4.4 视频文件（uvc_capture）

```bash
ffmpeg -f lavfi -i testsrc=duration=5:size=640x480:rate=30 -pix_fmt yuv420p test.mp4   # 生成测试视频
./build/uvc_capture 10 test.mp4                    # 按视频帧率读取，循环播放
./build/uvc_capture 10 test.mp4 640 480 30 ./rec_uvc --gray
```
- 设备参数不是数字、也不含 `://` 时，就当作本地视频文件。
- 按视频自身的帧率读取（读不到帧率时用命令行参数），读到结尾跳回开头，统计里显示 `file loops`。
- 读文件时，命令行里的宽、高、fps 不起作用，以文件本身为准。
- 时间戳用主机时间，所以校验报告里的"延迟"总是 0。

### 4.5 普通 USB 摄像头

- **Aravis 和 MVS 都读不了普通 USB 摄像头**：它们只支持 GigE Vision / USB3 Vision 工业相机，普通摄像头走的是 UVC 协议。
- 在 WSL2 里读 USB 摄像头需要同时满足：Windows 上装 usbipd-win，用 `usbipd attach --wsl` 转发设备；**并且**自己编译带 `uvcvideo` 驱动的 WSL 内核（当前内核 5.15.153.1 没有这个驱动，转发进来也不会出现 `/dev/video0`）。
- **推荐**：在 Windows 上用 OpenCV 直接编译运行 `uvc_capture`（代码不依赖 Linux 专有接口），设备序号 `0` 通常就是 USB 摄像头。

---

## 5. 相机采集的通用流程

以 GigE 相机为例。一台网口相机和电脑之间有两条通道：

```
         工控机                                         相机
┌────────────────────────┐   GVCP 控制通道（UDP 3956）  ┌──────────────────┐
│ 采集程序                │ ◀─────────────────────────▶ │ 寄存器、GenICam XML│
│ ①枚举 ②打开 ③配置       │  发现、读写参数、心跳         │                  │
│ ④建流 ⑤开始 ⑥取帧 ⑦停止 │   GVSP 图像通道（UDP）        │ 每帧切成很多包    │
│   SDK 缓冲池            │ ◀──────────────────────────── │ 例：1400 字节/包  │
└────────────────────────┘  单向推送                      └──────────────────┘
```

### 5.1 七个步骤

| 步骤 | 做什么 | Aravis | 海康 MVS |
|---|---|---|---|
| 0. 初始化 | SDK 4.x 要求 | — | `MV_CC_Initialize()` |
| ① 枚举 | 广播"谁是相机？"，收集回复（IP、型号、序列号） | `arv_update_device_list()` / `arv_get_n_devices()` / `arv_get_device_*()` | `MV_CC_EnumDevices()` |
| ② 打开 | 获取独占控制权、下载 GenICam XML、开始心跳 | `arv_camera_new(id)` | `MV_CC_CreateHandle()` + `MV_CC_OpenDevice()` |
| ③ 配置 | 写寄存器：包大小、像素格式、曝光、增益、帧率、触发 | `arv_camera_set_*()` / `arv_camera_set_string()` | `MV_CC_SetEnumValueByString()` / `SetFloatValue()` … |
| ④ 建流 | 在电脑上开 UDP 端口、准备缓冲 | `arv_camera_create_stream()` + `arv_stream_push_buffer()` × N | `MV_CC_SetImageNodeNum()` |
| ⑤ 开始 | 写 AcquisitionStart | `arv_camera_start_acquisition()` | `MV_CC_StartGrabbing()` |
| ⑥ 取帧 | 取一帧 → 检查状态 → 拷贝 → **归还缓冲** | `arv_stream_timeout_pop_buffer()` … `arv_stream_push_buffer()` | `MV_CC_GetImageBuffer()` … `MV_CC_FreeImageBuffer()` |
| ⑦ 停止 | 停止采集、关闭、释放 | `arv_camera_stop_acquisition()` + `g_object_unref()` + `arv_shutdown()` | `StopGrabbing` / `CloseDevice` / `DestroyHandle` / `MV_CC_Finalize()` |

### 5.2 关键细节

- **打开时下载 GenICam XML**：XML 描述了相机有哪些参数、每个参数对应哪个寄存器地址。所以 `ExposureTime` 这样的参数名在各家相机上通用。
- **心跳**：SDK 定期给相机发心跳。程序崩溃后，相机在心跳超时后会自动释放控制权；相机掉线时，SDK 会通过回调通知程序（Aravis 的 `control-lost` 信号，MVS 的异常回调）。
- **包大小**：包越大，每帧的包数越少，CPU 开销和丢包机会越小。真实相机建议网卡开巨帧（MTU 9000）。
- **缓冲必须归还**：取帧后不调用 `push_buffer` / `FreeImageBuffer`，SDK 手上的缓冲很快用光，后面的帧就会被丢弃（underrun）。
- **先拷贝再归还**：把数据 memcpy 到自己的帧池后立刻归还 SDK 缓冲，写盘、算法等慢操作在自己的线程里做。
- **主动拉取 vs 回调**：本项目的 Aravis/MVS 程序用主动拉取（在自己的循环里取帧）。如果用 SDK 的出图回调，回调里只能做轻活，**不能在回调里调用 stop**：stop 会等待回调线程结束，而回调正运行在这个线程上，会造成死锁（`sim_capture` 里对应的是 `th_.join()` 等待自身，标准库会抛出 `resource_deadlock_would_occur`）。

---

## 6. 各程序的处理逻辑与代码说明

### 6.1 公共头文件

#### `include/frame.h`

| 内容 | 说明 |
|---|---|
| `enum PixelFormat` | `PIX_MONO8=0x01080001`、`PIX_BAYERRG8=0x01080009`、`PIX_RGB8=0x02180014`（GenICam PFNC 编码） |
| `host_now_ns()` | 主机单调时钟，纳秒 |
| `struct Frame` | 帧号、相机时间戳、主机时间戳、宽高、像素格式、有效字节数、数据（预分配并复用） |
| `class FramePool` | 帧内存池。`create(数量, 每块字节数)` 一次性分配；`acquire()` 返回带自定义析构的 `shared_ptr`，最后一个使用者放手时缓冲自动回到池子；池空时返回 `nullptr` |
| `crc32()` | 标准 CRC32，与 Python `zlib.crc32` 结果一致 |
| `frame_id_missing(prev, cur)` | 计算两帧之间缺了几帧，处理帧号绕回（见 7.4） |

**为什么要用帧池**：每帧 1~18MB，高帧率下频繁 new/delete 既慢又会产生内存碎片，偶尔还会卡顿导致丢帧。

**为什么池子是 `QCAP + 8`**：除了队列里最多 64 帧，写盘线程手上还拿着 1 帧，回调里正在填 1 帧。多留 8 块余量，保证写盘慢时先表现为"队列满"（`drop queue`），而 `drop pool` 不为 0 就说明有缓冲泄漏（程序 bug）。代价是内存：72 × 帧大小（1280×1024 约 94MB；600 万像素 Mono8 约 450MB，RGB8 约 1.3GB）。

#### `include/frame_queue.h` —— `BoundedQueue<T>`

- `try_push()`：**非阻塞**，满了或已关闭就返回 false，由调用方计数丢帧。
- `pop_for(timeout)`：带超时出队。
- `close()`：关闭后不再接受新数据，但消费者仍能把剩余数据取完（排空）再退出。

#### `include/recorder.h` —— 写盘线程

录制文件格式（每帧 = 48 字节帧头 + 图像数据，小端序）：

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | char[4] | magic | `"FRM1"` |
| 4 | uint64 | frame_id | 帧号 |
| 12 | uint64 | dev_ts_ns | 相机时间戳（没有时为 0 或主机时间） |
| 20 | uint64 | host_ts_ns | 主机接收时间 |
| 28 | uint32 | width | 宽 |
| 32 | uint32 | height | 高 |
| 36 | uint32 | pixfmt | PFNC 像素格式 |
| 40 | uint32 | size | 图像数据字节数 |
| 44 | uint32 | crc32 | 图像数据的 CRC32 |

Python 读取方式：`struct.Struct("<4sQQQIIIII")`。

处理流程：从队列取帧 → 帧号连续性检查（`frame_id_missing`）→ 组织帧头 → 写盘并计时 → 每满 512MiB（`512ull << 20` = 536,870,912 字节）换下一个文件。
- 写完一帧后才检查大小，所以每个文件会略大于 512MiB（多出最后一帧）。
- `ull` 表示 64 位整数，防止把上限改大时（比如 `4096 << 20`）在 32 位 int 里溢出。
- `setvbuf` 设置 4MB 缓冲，减少 write 系统调用；偶尔缓冲刷到磁盘时会有毫秒级的写盘耗时（`maxWrite`）。
- `slow_disk_us` 参数用来模拟慢磁盘。

### 6.2 `sim_capture.cpp`

**组成**：`SimCamera`（模拟相机线程）+ `CaptureController`（状态机）+ 写盘线程 + 统计线程。

**SimCamera 的出帧节拍**：
```cpp
next += period;                       // period = 1/fps
std::this_thread::sleep_until(next);  // 睡到"下一帧该出的绝对时刻"
```
用 `sleep_until` 而不是 `sleep_for`，避免处理耗时累积，使帧率不会漂低。

**模拟链路丢包**：
```cpp
++frame_id_;                          // 先加帧号
if (d(rng) < 2) continue;             // 0~999 里抽到 0 或 1（0.2%）就跳过这帧，不调用回调
```

**`run_until(deadline)` 状态机**：
```
             open() 失败：等待 100→200→400…最多 5000ms（指数退避）
                 ┌──────┐
                 ▼      │
Disconnected → Connecting ──成功──▶ Configuring ──▶ Acquiring
     ▲                                                │ 断线回调设置 lost_ = true
     └──────────────── Error（停止相机，重连计数 +1）◀───┘
到达截止时间：退出循环 → cam_.stop() → Stopped
```

| 状态 | 做什么 |
|---|---|
| Disconnected | 进入 Connecting |
| Connecting | `cam_.open()`；失败则 `sleep_for(backoff_ms)`，`backoff_ms = min(backoff_ms*2, 5000)`；成功则重置为 100 |
| Configuring | 设置参数；清除断线标志；清除"上一帧帧号"（新会话）；启动相机并注册出帧回调和断线回调 |
| Acquiring | 每 50ms 检查一次 `lost_`，不碰图像数据 |
| Error | 停止相机、释放资源、重连计数 +1，回到 Disconnected |
| Stopped | 循环里实际走不到（预留） |

**指数退避的意义**：相机掉电或网线被拔时，立刻疯狂重试只会占满 CPU 和网络、日志刷屏；逐步拉长间隔，既能在恢复后很快重连，又不浪费资源。

**`on_frame`（运行在相机线程）**：
1. `received++`
2. **帧号检查**（放在最前面）→ `link_gaps` / `link_missing`：只反映相机或链路丢帧。
3. 从帧池取缓冲，取不到 → `dropped_pool++`
4. memcpy，填元数据
5. `try_push`，失败 → `dropped_queue++`

**输出示例与解读**见 8.2 节。

### 6.3 `aravis_capture.cpp`

```bash
./build/aravis_capture [秒数=10] [触发 none|soft|line1] [曝光us=5000] [相机]
./build/aravis_capture list                       # 只列出相机
```

| 环节 | 实现 |
|---|---|
| 枚举 | `enumerate_cameras()`：`arv_update_device_list()`，打印 序号/厂家/型号/序列号/地址/协议，按第 4 个参数（序列号/型号/IP/设备 ID 的一部分）匹配，标记 `<- selected` |
| 打开 | `arv_camera_new(dev_id)`；连接 `control-lost` 信号，回调里只设标志 |
| 回环处理 | 相机地址是回环地址时，`arv_gv_device_set_stream_options(..., ARV_GV_STREAM_OPTION_PACKET_SOCKET_DISABLED)`，改用普通 UDP socket |
| 配置 | 自动包大小、Mono8、关闭自动曝光、曝光时间、触发（none 时设 30fps；soft 为软件触发；line1 为 Line1 上升沿） |
| Chunk | 尝试开启 `Timestamp,FrameID` chunk，支持的话优先用 chunk 里的时间戳 |
| 建流 | `socket-buffer = AUTO`（自动加大接收缓冲）；32 块 SDK 缓冲 |
| 取帧 | `timeout_pop_buffer(1s)` → 状态为 SUCCESS 则拷贝入队（`ok`），否则 `bad++` → `push_buffer` 归还 |
| 软触发 | 独立线程每 50ms 调用一次 `arv_camera_software_trigger` |
| 统计 | `ok / bad / dropped / written / gaps` + SDK 的 `completed / failures / underruns` |
| 收尾 | 停止采集、释放对象、`arv_shutdown()` |

### 6.4 `mvs_capture.cpp`

```bash
./build/mvs_capture [秒数=10] [触发 none|soft|line0] [曝光us=5000] [像素格式=Mono8] [相机]
```

| 环节 | 实现 |
|---|---|
| 初始化 | `MV_CC_Initialize()`，所有退出路径都调用 `MV_CC_Finalize()` |
| 枚举 | `MV_GIGE_DEVICE \| MV_USB_DEVICE \| MV_VIR_GIGE_DEVICE \| MV_VIR_USB_DEVICE`；列出所有相机，按第 5 个参数选择 |
| 配置 | 真实网口相机协商最优包大小；像素格式（第 4 个参数）；关闭自动曝光/自动增益；触发模式；`SetImageNodeNum(16)` |
| 取帧 | `MV_CC_GetImageBuffer(1000ms)`；帧长优先用 64 位的 `nFrameLenEx`；`nLostPacket > 0` 记为丢包帧；`MV_CC_FreeImageBuffer` 归还 |
| 掉线 | 异常回调收到 `MV_EXCEPTION_DEV_DISCONNECT` 时设标志 |

### 6.5 `uvc_capture.cpp`

```bash
./build/uvc_capture [秒数=10] [设备=0] [宽=1280] [高=720] [fps=30] [输出目录=./rec_uvc] [--show] [--gray]
```

| 环节 | 实现 |
|---|---|
| 设备 | 数字 → 摄像头序号（Linux 上用 V4L2 后端）；含 `://` → 网络流；其它 → 视频文件 |
| 摄像头参数 | 请求 MJPG（USB2 下高分辨率才能跑满帧率）、宽高、fps，`BUFFERSIZE=2` 降低延迟；打印实际生效的值 |
| 文件模式 | 按视频帧率限速（落后超过 5 帧就重新对齐，不补读）；读到结尾 `CAP_PROP_POS_FRAMES=0` 循环播放 |
| 断线重连 | 连续 10 次读取失败视为断线，释放后按指数退避重新打开 |
| 颜色 | OpenCV 给出 BGR，转换为 RGB8 或 Mono8（`--gray`）后再存 |
| 帧号 | UVC 不提供帧号，由程序自己计数（所以链路丢帧检测不出来） |
| 时间戳 | Linux 摄像头用 `CAP_PROP_POS_MSEC`（驱动给的缓冲区时间戳）；文件模式和其它情况用主机时间 |
| 预览 | `--show` 时 `imshow`，按 q 或 Esc 退出（imshow 必须在主线程，所以采集循环放在主线程） |
| 跨平台 | 只用标准 C++17 和 OpenCV（目录创建用 `std::filesystem`），可以在 Windows 上编译 |

### 6.6 `tools/verify_record.py`

```bash
python3 tools/verify_record.py rec/rec_*.bin                 # 校验
python3 tools/verify_record.py --dump 100 rec/rec_0000.bin   # 导出帧号 100 的图像
```
- 逐帧检查：magic、数据是否截断、CRC32、帧号连续性（含绕回）。
- 帧间隔统计（min/p50/p99/max）：优先用相机时间戳；相机时间戳为 0 时退回主机接收时间，并注明 "host time, camera has no timestamp"。
- 延迟统计：相机时间戳为 0 时跳过（只有相机和主机用同一个时钟时才有意义）。
- 导出：Mono8 存 `.pgm`，RGB8 存 `.ppm`；Bayer 等其它格式按灰度导出，看到的是未经去马赛克的原始画面。
- 退出码：没有 CRC 错误返回 0，有则返回 2。
- 包含 Python 版的 `frame_id_missing()`，规则与 C++ 一致，`mvs_gui.py` 从这里导入。

### 6.7 `tools/qt_demo.py`

```bash
python3 tools/qt_demo.py                  # 动画测试画面（看 WSLg 显示是否流畅）
python3 tools/qt_demo.py rec_0000.bin     # 回放录像
```
- 测试画面：移动的彩色方块，显示实际刷新帧率（约 30fps 即为正常）。
- 回放：先扫描建立帧索引（只读帧头），按 30fps 循环播放；支持暂停和拖动进度条逐帧查看；支持 Mono8 / RGB8。
- 大图先缩放到窗口大小再显示，避免卡顿。
- 窗口底部显示 Qt 平台、`DISPLAY`、`WAYLAND_DISPLAY`，便于排查显示问题。

### 6.8 `tools/mvs_gui.py`（采集控制界面）

```bash
python3 tools/mvs_gui.py
```

**界面功能**：

| 区域 | 功能 |
|---|---|
| 相机 | 下拉列出所有相机（含虚拟相机）、刷新、打开/关闭 |
| 参数 | 像素格式（从相机读取可选列表，采集中禁止修改）；曝光、增益、帧率（读回当前值和范围，回车或失焦才下发） |
| 触发 | 连续采集 / 软触发 / 硬触发 Line0；触发边沿（仅硬触发）；触发延时（软/硬触发）；去抖 Line0（仅硬触发）；"触发一次"；按间隔自动软触发 |
| 采集/录制 | 开始/停止采集；选择目录后录制 `.bin` |
| 右侧 | 实时画面（Bayer/YUV 等格式调用 SDK 转成 RGB 显示）；统计：采集 fps、显示 fps、已收帧数、帧号、分辨率和格式名、丢包帧、帧号缺口；录制：写入帧数、MB、MB/s、队列、丢弃、最大写盘耗时 |

**线程结构**：
- **取帧线程**（`Grabber`）：`GetImageBuffer` → `string_at` 拷贝 → 立刻 `FreeImageBuffer` → 统计（含帧号检查）→ 放到"最新帧"槽位（显示只取最新一帧，不排队）→ 录制时非阻塞放入录制队列（满了计数丢弃）。
- **写盘线程**（`Recorder`）：与 `recorder.h` 格式和策略相同，用 `zlib.crc32` 计算 CRC，512MiB 滚动。
- **GUI 线程**：33ms 定时器负责显示和每秒统计；掉线时自动停止并提示。

**实现细节**：
- 启动前设置 `MVCAM_COMMON_RUNENV`（SDK 靠它找动态库），并把 `MvImport` 加入 `sys.path`（可用 `MVS_PY_PATH` 覆盖）。
- 掉线回调对象必须保存引用，否则被 Python 回收后 SDK 再调用会崩溃。
- 用 `MV_XML_GetNodeAccessMode` 判断参数是否**可写**：例如触发边沿设为电平触发时，相机会把触发延时锁成只读，界面输入框随之变灰。
- 去抖时间属于 I/O 口：先设置 `LineSelector=Line0`，再读写 `LineDebouncerTime`。
- 触发边沿只在硬触发时可读（软触发下相机不提供这个参数）。
- 像素格式名由 SDK 的 `PixelType_Gvsp_*` 常量自动生成。

**性能**：600 万像素 RGB8（18MB/帧）在 Python 里约 15fps，原因是每帧要拷贝 18MB。要跑高帧率，取帧和录制应放在 C++ 里，Python 只负责界面。

---

## 7. 丢帧统计体系

### 7.1 从网线到文件的各个关卡

| 关卡 | 丢帧原因 | 统计 | 在哪里看 |
|---|---|---|---|
| ① 网络包 | UDP 包在网线、交换机或网卡上丢了 | `n_missing_packets` / `nLostPacket` | Aravis：`ARV_DEBUG=stream:3`；MVS：每帧信息 |
| ② 拼帧 | 缺包且没有补回来，帧不完整 | `bad` / `failures` / `lostPacketFrames` | 采集程序输出 |
| ③ SDK 缓冲 | 帧到了但 SDK 没有空缓冲 | `underruns` | `aravis_capture` 输出 |
| ④ 帧号（入口） | 整帧都没收到（相机或链路） | `linkGap`（sim_capture）、`帧号缺口`（mvs_gui） | 采集程序输出 |
| ⑤ 程序内部 | 帧池耗尽 / 队列满（下游慢） | `drop pool` / `drop queue` / `dropped` / `丢弃` | 采集程序输出 |
| ⑥ 帧号（写盘） | 文件里最终缺的帧 = ④ + ⑤ | `fileGap` / `gaps` | 采集程序输出 |
| 离线校验 | 同 ⑥，另外校验 CRC | `id_gaps` / `crc_errors` | `verify_record.py` |

### 7.2 排查对照表

| 现象 | 问题在哪 | 处理方法 |
|---|---|---|
| `bad`/`failures` 多 | 网络丢包 | 开巨帧、加大接收缓冲、开启丢包重发；检查网线、交换机带宽、网卡中断合并 |
| `underruns` 多 | SDK 缓冲不够，或者归还太慢 | 增加 SDK 缓冲数；取帧循环里不要做耗时操作 |
| `drop queue`/`dropped` 多 | 下游（写盘/算法）太慢 | 换更快的磁盘、加大队列、算法放到其它线程 |
| `drop pool` 不为 0 | 有缓冲一直没归还（程序 bug） | 检查哪里拿了帧不放 |
| `linkGap` 多但 `bad` 少 | 整帧没发出来 | 相机端问题：触发过快、相机内部缓冲溢出、帧率超过带宽 |
| `fileGap` ≈ `linkGap` + `drop queue` | 正常的对应关系 | — |

### 7.3 为什么少量丢包会导致大量坏帧

一帧要切成很多包，**任何一个包丢失，这一帧就不完整**。实测：512×512 的帧约 188 个包，模拟 2% 丢包，全部帧完整的概率 = 0.98¹⁸⁸ ≈ 2%，结果 145 帧全部损坏。所以 GigE 相机必须依靠：
1. **丢包重发**：SDK 发现缺包，通过控制通道请相机重发。真实相机和海康 SDK 默认开启。
2. **巨帧**：MTU 9000 时每帧的包数大约降到 1/6。

### 7.4 帧号绕回

- GigE Vision 1.x 的帧号（block ID）是 **16 位**，取值 1~65535，**绕回后从 1 开始**（0 保留不用）。30fps 下约 36 分钟绕回一次；Aravis 虚拟相机启动约 4 秒就会绕回。
- 海康 SDK 的 `nFrameNum` 是 32 位，绕回后从 0 开始。
- `frame_id_missing(prev, cur)` 的规则：
  - `cur > prev`：返回 `cur - prev - 1`（0 表示连续）。
  - 前一帧接近 16 位上限、当前帧 ≤ 1000：按 16 位绕回计算缺失数。
  - 前一帧接近 32 位上限、当前帧 ≤ 1000：按 32 位绕回计算。
  - 其它帧号倒退的情况（相机重启后重新计数、重复帧号）：返回 -1，记一次不连续但不计缺失数。
- C++（`frame.h`）和 Python（`verify_record.py`）两个版本规则一致，有 10 个单元测试用例。

---

## 8. 测试与评估方法

### 8.1 测试清单与实测结果

| 场景 | 命令 | 本次实测结果 |
|---|---|---|
| 模拟相机：正常 | `./build/sim_capture 6` | 565~566 帧全部写盘，drop 0，链路丢帧 4，重连 1 次 |
| 模拟相机：慢磁盘 | `./build/sim_capture 6 100 1280 1024 15000` | 收到 567 帧，写入 357，drop queue 210，链路丢帧仍为 4，文件缺约 213 帧 |
| Aravis 连续采集 | `./build/aravis_capture 5 none` | 约 30fps，142~144 帧，帧号无缺口，CRC 全部通过；开始时 1 个半帧（bad=1） |
| Aravis 软触发 | `./build/aravis_capture 5 soft` | 100 帧，帧间隔约 40ms |
| Aravis 帧号绕回 | `./build/aravis_capture 12 none` | 帧号 65534→65535→1→2，gaps=0 |
| Aravis 模拟丢包 | 虚拟相机加 `-r 20` | 145 帧全部 bad（不支持重发），n_missing_packets 21252 |
| MVS 黑白连续 | `./build/mvs_capture 5 none 5000 Mono8 10GM` | 141 帧（约 28.5fps），丢包 0，帧号连续 |
| MVS 软触发 | `./build/mvs_capture 5 soft 5000 Mono8 10GM` | 90 帧，帧间隔约 56ms |
| MVS 自定义黑白图片 | Mono8 文件夹放 3 张图 | 3 张图循环播放（第 4 帧与第 1 帧 md5 相同） |
| MVS 彩色 RGB8 | `./build/mvs_capture 2 none 5000 RGB8Packed 10GC` | 每帧 18MB，约 21fps；红绿蓝色块读回 (253,0,0)/(0,254,0)/(0,0,254)，通道顺序正确 |
| 视频文件（改进前） | `./build/uvc_capture 10 test.mp4` | 6745 帧，丢 3941 帧（读得太快）、重连 44 次（结尾被当成断线） |
| 视频文件（改进后） | 同上 | 稳定 30fps，丢帧 0，帧间隔中位数 33.33ms，循环播放正确 |
| 采集界面 | `python3 tools/mvs_gui.py` | RGB8 约 15fps；录制 3 秒 46 帧 868MB 无丢帧；软触发 100ms 间隔 3 秒 30 帧；Bayer 转换显示正确 |
| WSLg 显示 | `python3 tools/qt_demo.py` | 窗口正常弹出，约 30fps |

### 8.2 读懂 sim_capture 的输出

```
config: 6s, 100 fps, 1280x1024 Mono8, 131.1 MB/s, slow_disk=0us
[state] Disconnected -> Connecting
  open failed, retry in 100 ms           ← 指数退避：100ms
  open failed, retry in 200 ms           ←           200ms（翻倍）
[state] Connecting -> Configuring
[state] Configuring -> Acquiring
[stat] recv=68 write=68 fps=68 89.1MB/s queue=0/64 drop(pool/queue)=0/0 linkGap=1 fileGap=1 maxWrite=5413us
        │       │        │                 │          │                 │         │          └ 单帧最慢写盘耗时
        │       │        │                 │          │                 │         └ 写盘线程看到的帧号缺口
        │       │        │                 │          │                 └ on_frame 入口的链路丢帧
        │       │        │                 │          └ 池耗尽 / 队列满丢帧
        │       │        │                 └ 当前队列水位 / 容量
        │       │        └ 这一秒写了几帧（第一秒有约 300ms 在重试连接，所以只有 68）
        │       └ 累计写盘帧数
        └ 累计收到帧数
[state] Acquiring -> Error               ← 3 秒时模拟断线
[state] Error -> Disconnected            ← 停止相机，重连 +1
...                                       ← 重新连接（这次第一次就成功）
[state] Acquiring -> Stopped             ← 到 6 秒截止时间
[stat] ... fps=3 ...                     ← 停止后统计线程最后一次打印，只覆盖零点几秒
```
**为什么约 566 帧而不是 600 帧**：600 − 开头约 300ms 的重试（约 30 帧）− 断线重连的几十毫秒（约 5 帧）− 4 帧模拟链路丢帧 ≈ 566。

### 8.3 评估方法

1. **完整性**：`received == written` 且各项 drop 为 0，说明程序内部没有丢帧。
2. **正确性**：`verify_record.py` 报告 `crc_errors=0`，说明写盘数据没有损坏。
3. **定位丢帧**：对照 7.1 节各关卡的计数，判断问题出在网络、SDK、程序内部还是相机端。
4. **帧率与稳定性**：看帧间隔的 p50/p99/max。p99 远大于 p50 说明有卡顿。
5. **压力测试**：
   - 慢磁盘参数：观察队列被填满后，丢帧计数是否正确。
   - Aravis 虚拟相机的 `-r` 参数：测试丢包处理。
   - 提高分辨率和帧率：测试带宽和写盘的上限。
6. **画面内容**：`--dump` 导出单帧，或用 `qt_demo.py` 回放，人工确认画面和颜色正确。
   - 用"红、绿、蓝"纯色块图检查通道顺序：在已知位置读取像素值。
   - 用 md5 比较多帧，确认循环播放和帧内容确实在变化。
7. **界面自动化测试**：无界面模式 `QT_QPA_PLATFORM=offscreen` 下用脚本调用界面方法、`grab()` 截图检查。
   - 注意：测试时弹出的模态对话框（比如"打开相机失败"）会让脚本卡住。
   - 脚本输出要加 `python3 -u` 关闭缓冲，否则卡住时看不到任何输出。
8. **参数是否真的写入**：设置后从相机读回比对；关闭再打开，确认读回的是相机自身的值。

---

## 9. 常见问题与修复

| # | 现象 | 原因 | 修复 |
|---|---|---|---|
| 9.1 | VS Code Ctrl+Click 无法跳转 | WSL 侧没装 C/C++ 扩展，没有 `compile_commands.json` | 见 2.1 |
| 9.2 | 装了 Aravis，CMake 仍报 "Aravis not found" | 缺 `libusb-1.0-0-dev`；CMake 缓存了失败结果 | 装 libusb 开发包，`rm build/CMakeCache.txt` 后重新配置 |
| 9.3 | 解压安装包时报 `unzip: command not found` | 系统没有 unzip | 用 Python 的 zipfile 解压（见 2.3），或 `apt install unzip` |
| 9.4 | Aravis 虚拟相机一帧都收不到（`n_ignored_packets` 等于收到的包数） | 默认的 packet socket 收包方式在回环地址上拼不出帧 | 地址为回环时禁用 packet socket（已在代码中自动处理）；`arv-camera-test-0.8 --no-packet-socket` 可以验证 |
| 9.5 | Aravis 约 13% 的帧不完整 | socket 接收缓冲区太小，包溢出 | `g_object_set(stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO)`（已加入代码） |
| 9.6 | `mvs_capture` 提示 "no camera found"，但虚拟相机已经建好 | 枚举时没有加虚拟设备标志；SDK 4.x 没有调用 `MV_CC_Initialize` | 枚举类型加上 `MV_VIR_GIGE_DEVICE \| MV_VIR_USB_DEVICE`；开头 Initialize，退出 Finalize |
| 9.7 | VirtualDeviceTool 窗口不出现，快捷方式也没反应 | WSLg 把窗口送到 Windows 的内部连接断了（`/mnt/wslg/weston.log` 里有 `rdp_peer is not initalized`） | 在 Windows 的 cmd 或 PowerShell 里执行 `wsl --shutdown`，然后重新打开；不行就先 `wsl --update` |
| 9.8 | 黑白相机设置 RGB8Packed 返回 `0x80000004` | 黑白型号不支持彩色格式（描述文件里标注为未实现） | 用彩色型号，如 `MV-CS060-10GC-PRO` |
| 9.9 | 打开相机失败 "被其他程序占用" | 工业相机同一时刻只允许一个程序打开（独占） | 关闭其它占用相机的程序（MVS 客户端、另一个采集界面） |
| 9.10 | 无界面模式下的界面测试脚本卡住、没有输出 | 打开失败时弹出了模态对话框，没人点确定；Python 输出被缓冲 | 先确认相机空闲；用 `python3 -u` |
| 9.11 | 视频文件模式丢帧严重、反复"重连" | 读文件没有帧率限制；读到结尾被当成断线 | 按视频帧率限速 + 结尾跳回开头循环（已修复） |
| 9.12 | 校验报告的延迟约 265 秒，没有帧间隔 | 海康虚拟相机不提供硬件时间戳（为 0） | 时间戳为 0 时跳过延迟，帧间隔改用主机时间（已修复） |
| 9.13 | 采集界面显示 fps 比采集 fps 还高 | 重新开始采集时显示计数没有清零 | 开始采集时清零（已修复） |
| 9.14 | 帧号从 65535 跳到 1 被误判成丢帧 | GigE Vision 16 位帧号绕回 | `frame_id_missing()` 处理绕回（已修复） |
| 9.15 | 第二台 Aravis 虚拟相机启动报 "Failed to start camera" | `-i` 地址不是网卡上实际配置的地址 | `sudo ip addr add 127.0.0.2/8 dev lo` 后再用 `-i 127.0.0.2` |
| 9.16 | `pkill -f <模式>` 在自动化 Bash 里把执行命令的 shell 自己杀了（退出码 144） | `-f` 匹配完整命令行，shell 自己的命令行里也包含这个模式 | 用 `pkill -x <进程名>`（进程名最多 15 个字符，如 `arv-fake-gv-cam`） |
| 9.17 | 重启 WSL 后 `/tmp` 下的测试文件不见了 | `/tmp` 在重启时被清空 | 要保留的文件放在项目目录 |
| 9.18 | 运行时打印 `*******XOpenDisplay Fail *******` | 海康 SDK 尝试连接图形界面失败 | 不影响采集，可以忽略 |
| 9.19 | Qt 报 "could not load the Qt platform plugin xcb" | 缺少 xcb 相关的库 | 例如 `apt install libxcb-cursor0` |
| 9.20 | 界面里的中文显示成方框 | 没有中文字体 | `apt install fonts-noto-cjk`；程序里指定字体 |
| 9.21 | 600 万像素实时显示卡顿 | 每帧缩放和绘制大图开销很大 | 先缩放到窗口大小再显示（已实现），或者降低显示刷新率 |
| 9.22 | Python 采集达不到相机帧率 | 每帧拷贝大块数据加上 Python 本身的开销 | 取帧和录制放到 C++ 里，Python 只做界面 |
| 9.23 | RGB 和 BGR 颜色反了（红显示成蓝） | OpenCV 用 BGR，Qt 和大多数库用 RGB | 显示或存储前转换（`uvc_capture` 已处理） |
| 9.24 | 虚拟相机参数关闭再打开就恢复默认 | 海康虚拟相机每次打开都会重置 | 正常现象；真实相机可以存到 UserSet；软件启动时重新下发参数 |
| 9.25 | 电平触发时触发延时写不进去 | 相机在电平触发时把延时锁成只读 | 界面用"是否可写"判断自动禁用（已实现）；改回边沿触发后解锁 |

---

## 10. 视觉检测系统的搭建

### 10.1 典型拓扑

```
                       ┌──────────────── 工控机（上位机）────────────────┐
                       │  视觉软件：相机控制 + 图像算法 + 结果判断        │
                       │  多口 PoE 网卡        网口/工业总线             │
                       └─────┬─────────────────────────┬─────────────────┘
                             │ ② 图像（网线，PoE 供电）  │ ④ 检测结果 OK/NG
  ┌──────────┐ ①触发   ┌─────┴─────┐                ┌──┴──────┐  ⑤剔除信号  ┌────────┐
  │光电传感器 ├──────▶ │  工业相机   │◀─ 或 PLC 触发 ─┤   PLC   ├───────────▶│剔除气缸 │
  └──────────┘(或经PLC)│ Line0 输入 │                └────┬────┘             └────────┘
                       │ Line1 输出 ├─③频闪同步─▶ 光源控制器 ─▶ 光源      编码器（测速/定位）
                       └───────────┘
            ════════════ 传送带 ════════ 产品 →→→ ════════════════════════════
```

**一次检测的流程**：①产品遮挡传感器 → 相机按设定的边沿识别触发，经过触发延时后曝光 → ②图像传到工控机 → ③曝光时 Line1 输出信号控制频闪 → ④算法判断，把结果发给 PLC → ⑤PLC 根据编码器计算时机，执行剔除。

**关键约束**：结果必须在产品到达剔除工位之前送到 PLC，所以采集加算法的总耗时有硬性上限。

### 10.2 三种触发接法

| 接法 | 信号路径 | 优点 | 缺点 |
|---|---|---|---|
| 传感器直连相机 | 传感器 → 相机 Line0 | 最简单，延迟最低最稳定 | PLC 不知道相机拍了没有 |
| PLC 触发相机 | 传感器 → PLC → 相机 Line0 | PLC 掌握全局节拍 | 多一级 PLC 扫描周期（毫秒级抖动） |
| 软触发 | PLC → 网络 → 工控机软件 → 相机 | 不用额外接线 | 延迟大且不固定，只适合产品停稳后再拍 |

### 10.3 参数由谁设置

- 触发边沿、延时、去抖、曝光等是**相机的参数**，由工控机上的视觉软件通过 SDK 写入相机。PLC 只负责在合适的时刻发出信号。
- 调试方法：按相机手册接好线 → 用手遮挡传感器 → 看相机有没有拍。如果总是在"产品离开时"才拍，说明边沿设反了；一次信号拍了多张，就加大去抖；产品在画面里的位置偏了，就调整延时。
- 参数可以存到相机的**用户参数集（UserSet）**并设为上电默认加载；更稳妥的做法是软件启动时按配置文件重新下发一遍。

---

## 11. 术语表

### 11.1 接口与协议

| 术语 | 解释 |
|---|---|
| **GigE Vision** | 基于千兆以太网的工业相机标准。带宽约 115MB/s，线缆可达 100 米 |
| **GVCP** | GigE Vision 控制协议，UDP 3956 端口。负责发现相机、读写寄存器、心跳 |
| **GVSP** | GigE Vision 流协议（UDP）。相机把每帧切成包推给电脑 |
| **USB3 Vision** | 基于 USB3 的工业相机标准。带宽约 350MB/s，线缆 3~5 米 |
| **10GigE / 25GigE** | 万兆 / 2.5 万兆网口相机，用于高分辨率加高帧率 |
| **CoaXPress（CXP）** | 同轴电缆高速接口，必须配采集卡 |
| **Camera Link** | 老式高速接口，必须配采集卡 |
| **采集卡（Frame Grabber）** | 插在工控机 PCIe 槽里接收相机数据的板卡。MVS 里的 `driver/pcie` 就是海康采集卡的驱动 |
| **GenICam** | 相机参数的通用描述标准。相机内置 XML，把 `ExposureTime` 这类参数名映射到寄存器地址 |
| **PFNC** | GenICam 的像素格式命名和编码标准，例如 Mono8 = 0x01080001 |
| **UVC** | USB 视频类协议，普通 USB 摄像头和笔记本摄像头使用它 |
| **V4L2** | Linux 的视频设备接口，对应 `/dev/video*` |
| **RTSP / H.264 / H.265** | 安防监控常用的推流协议和有损视频编码。工业检测不用：画面有损、延迟不固定、无法按触发拍摄 |
| **MJPG** | 每帧单独做 JPEG 压缩的格式。USB 摄像头用它在高分辨率下跑满帧率 |
| **PoE** | 以太网供电，一根网线同时供电和传数据 |
| **巨帧 / MTU 9000** | 以太网大包。每帧的包数减少，降低 CPU 开销和丢包影响 |
| **丢包重发（Packet Resend）** | SDK 发现缺包时请相机重新发送 |
| **Block ID** | GVSP 帧号，GigE Vision 1.x 为 16 位（1~65535，绕回后从 1 开始） |
| **Payload** | 每帧图像数据的字节数 |
| **Chunk Data** | 相机随图像一起送出的附加数据（时间戳、帧号、曝光值等） |
| **PTP（IEEE 1588）** | 网络时钟同步协议，多台相机的时间戳可以对齐 |
| **心跳（Heartbeat）** | SDK 定期通知相机"我还在"。超时后相机释放控制权 |
| **独占访问** | 同一时刻只允许一个程序打开并控制相机 |
| **Packet socket** | Linux 直接从网卡抓原始包的方式，性能好，但在回环地址上不可用 |

### 11.2 像素格式

| 格式 | 每像素 | 3072×2048 一帧 | 说明 |
|---|---|---|---|
| **Mono8** | 1 字节 | 6MB | 灰度，256 级 |
| **Mono10 / Mono12** | 2 字节 | 12MB | 1024 / 4096 级，高位补 0 |
| **Mono10Packed / Mono12Packed** | 1.5 字节 | 9MB | 去掉补位，2 个像素占 3 字节 |
| **BayerRG8 / GR8 / GB8 / BG8** | 1 字节 | 6MB | 彩色传感器的原始数据，每个像素只有一种颜色，字母表示起始排列 |
| **RGB8Packed** | 3 字节 | 18MB | 每像素 R、G、B |
| **BGR8Packed** | 3 字节 | 18MB | 顺序相反（OpenCV 默认） |
| **YUV422** | 2 字节 | 12MB | 亮度和颜色分开存，颜色只存一半精度 |

- **去马赛克（Demosaic）**：把 Bayer 数据用相邻像素插值成完整 RGB。工业相机通常优先输出 Bayer（数据量只有 RGB 的 1/3），在电脑上做转换。Bayer 起始顺序设错会导致红蓝互换。

### 11.3 成像参数

| 术语 | 解释 |
|---|---|
| **曝光时间（μs）** | 每帧感光的时间。越长越亮，但运动物体会拖影。拖影长度 = 速度 × 曝光时间（1m/s × 5ms = 5mm） |
| **增益（dB）** | 信号放大，画面变亮，噪点也一起放大。约每 6dB 亮度翻倍。原则：先加光，再加曝光，最后才加增益 |
| **帧率（fps）** | 每秒帧数。受曝光时间、传感器读出速度、传输带宽限制。千兆网 + 6MB/帧 ≈ 18fps（虚拟相机默认 18.4fps 就是这个原因） |
| **频闪（Strobe）** | 光源只在曝光期间点亮，画面又亮又不拖影 |

### 11.4 触发

| 术语 | 解释 |
|---|---|
| **连续采集** | 相机按帧率不停地拍 |
| **软触发** | 程序发命令拍一张，延迟为毫秒级且不固定 |
| **硬触发** | 相机 I/O 口收到电信号就拍，微秒级且稳定 |
| **Line0 / Line1** | 相机 I/O 口。Line0 常用作光耦隔离输入，Line1 常用作输出（如控制频闪） |
| **触发边沿（TriggerActivation）** | 上升沿（低→高）、下降沿（高→低）、任意边沿、高电平、低电平 |
| **触发延时（TriggerDelay）** | 收到触发后等多久再曝光，让产品走到画面中间 |
| **去抖（LineDebouncerTime）** | 信号要稳定保持这么久才算有效，用来过滤毛刺 |
| **NPN / PNP** | 传感器输出类型：NPN 检测到物体时输出低电平，PNP 输出高电平。接线和边沿设置要与之匹配 |
| **UserSet** | 相机内部的用户参数集，可以设为上电默认加载 |

### 11.5 系统与软件

| 术语 | 解释 |
|---|---|
| **上位机 / 下位机** | 上位机一般指工控机及其上的软件；下位机指 PLC 等控制器 |
| **PLC** | 可编程逻辑控制器，控制产线节拍、电机、气缸 |
| **编码器** | 测量传送带位移，用于计算剔除时机或触发线阵相机 |
| **帧池（Frame Pool）** | 预先分配并反复使用的帧缓冲 |
| **有界队列** | 容量固定的队列，满了就拒绝入队（产生丢帧计数），不阻塞生产者 |
| **背压** | 下游变慢导致上游积压的现象，本项目里表现为队列水位上升 |
| **指数退避** | 重试间隔按倍数增长（100→200→400…→5000ms） |
| **Underrun** | SDK 没有空缓冲可用，导致帧被丢弃 |
| **CRC32** | 32 位循环冗余校验，用来发现数据损坏 |
| **WSLg** | WSL 的图形支持，把 Linux 窗口显示到 Windows 桌面 |
| **usbipd-win** | 把 Windows 的 USB 设备转发给 WSL 的工具 |
| **离屏模式（offscreen）** | `QT_QPA_PLATFORM=offscreen`，Qt 程序不显示窗口，可以用于自动化测试和截图 |

---

## 12. 后续可做的工作

1. `aravis_capture` 和 `mvs_capture` 的取帧循环里也加上入口帧号检查（区分链路丢帧和文件缺帧），并加上 `sim_capture` 那样的状态机断线重连。
2. `sim_capture` 生成会动的测试图案（并画上帧号），让回放能看到画面内容。
3. `mvs_gui.py` 增加参数配置的保存和加载（启动时自动下发）；Bayer 录像回放时做去马赛克。
4. 采集界面加上和 PLC 的通信（Modbus TCP 等）：接收触发指令、上报 OK/NG 结果。
5. 高帧率场景：C++ 采集录制 + Python/Qt 界面，两者通过共享内存或网络通信。
6. 接入真实相机后验证：曝光和增益效果、硬件触发（边沿、延时、去抖）、相机时间戳、PTP 同步、丢包重发、巨帧。

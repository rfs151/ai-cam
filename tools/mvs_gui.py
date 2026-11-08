#!/usr/bin/env python3
"""海康 MVS 相机采集控制界面（PyQt5 + MVS Python SDK）

功能: 枚举/打开相机，设置像素格式、曝光、增益、帧率、触发方式，实时预览，
      录制为 .bin（与 recorder.h 相同的 FRM1 格式，可用 verify_record.py 校验、qt_demo.py 回放）
线程: 取帧线程只做 取帧->拷贝->统计；显示只取最新一帧；录制走有界队列，满了丢帧计数，不阻塞取帧
用法: python3 tools/mvs_gui.py
依赖: sudo apt install python3-pyqt5；海康 MVS SDK（默认 /opt/MVS）
无相机测试: 用 /opt/MVS/bin/VirtualDeviceTool.sh 添加虚拟相机
"""
import ctypes
import os
import queue
import struct
import sys
import threading
import time
import zlib

# ---------- 加载 MVS Python SDK ----------
MVS_ROOT = os.environ.get("MVCAM_SDK_PATH", "/opt/MVS")
os.environ.setdefault("MVCAM_COMMON_RUNENV", MVS_ROOT + "/lib")  # SDK 靠它找 libMvCameraControl.so
sys.path.append(os.environ.get("MVS_PY_PATH", MVS_ROOT + "/Samples/64/Python/MvImport"))
try:
    from MvCameraControl_class import *  # noqa: F401,F403  (SDK 的 MvCamera 和各种结构体/常量)
except Exception as e:  # SDK 不在时给出明确提示
    sys.exit(f"无法加载 MVS Python SDK: {e}\n请确认已安装 MVS（默认 /opt/MVS），或设置 MVS_PY_PATH")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_record import frame_id_missing   # 同目录，帧号绕回规则与 C++ 一致

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QComboBox,
                             QDoubleSpinBox, QFileDialog, QFormLayout, QGridLayout,
                             QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
                             QMessageBox, QPushButton, QRadioButton, QSpinBox,
                             QVBoxLayout, QWidget)

PIX_MONO8 = PixelType_Gvsp_Mono8
PIX_RGB8 = PixelType_Gvsp_RGB8_Packed
HDR = struct.Struct("<4sQQQIIIII")   # 与 recorder.h 中 FrameRecordHeader 一致
# 像素格式数值 -> 名称（取自 SDK 的 PixelType_Gvsp_xxx 常量），如 0x01080009 -> BayerRG8
PIX_NAMES = {v: k[len("PixelType_Gvsp_"):] for k, v in list(globals().items())
             if k.startswith("PixelType_Gvsp_") and isinstance(v, int)}
# 硬触发信号的有效条件（TriggerActivation 的取值）
EDGE_NAMES = {"RisingEdge": "上升沿", "FallingEdge": "下降沿", "LevelHigh": "高电平",
              "LevelLow": "低电平", "AnyEdge": "任意边沿"}


def c_str(arr):
    return bytes(arr).split(b"\0", 1)[0].decode(errors="replace")


# ---------------------------------------------------------------------------
# 相机：对 SDK 的薄封装
# ---------------------------------------------------------------------------
class Camera:
    TYPES = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_VIR_GIGE_DEVICE | MV_VIR_USB_DEVICE

    def __init__(self):
        self.cam = MvCamera()
        self.is_open = False
        self.lost = False
        # 回调对象必须保存引用，否则被 Python 回收后 SDK 调用会崩溃
        self._exc_cb = CFUNCTYPE(None, c_uint, c_void_p)(self._on_exception)

    def _on_exception(self, msg_type, user):
        if msg_type == MV_EXCEPTION_DEV_DISCONNECT:
            self.lost = True   # SDK 内部线程里只置标志

    @staticmethod
    def enum_devices():
        lst = MV_CC_DEVICE_INFO_LIST()
        if MvCamera.MV_CC_EnumDevices(Camera.TYPES, lst) != 0:
            return [], lst
        devs = []
        for i in range(lst.nDeviceNum):
            d = cast(lst.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            gige = d.nTLayerType in (MV_GIGE_DEVICE, MV_VIR_GIGE_DEVICE)
            info = d.SpecialInfo.stGigEInfo if gige else d.SpecialInfo.stUsb3VInfo
            devs.append((c_str(info.chModelName), c_str(info.chSerialNumber), d))
        return devs, lst   # lst 要一起保留，d 指向它内部的内存

    def open(self, dev):
        ret = self.cam.MV_CC_CreateHandle(dev)
        if ret != 0:
            return f"CreateHandle 失败 0x{ret:08x}"
        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            self.cam.MV_CC_DestroyHandle()
            return f"打开失败 0x{ret:08x}（被其他程序占用？IP 不在同网段？）"
        self.is_open, self.lost = True, False
        self.cam.MV_CC_RegisterExceptionCallBack(self._exc_cb, None)
        if dev.nTLayerType == MV_GIGE_DEVICE:   # 真实网口相机协商最大包
            pkt = self.cam.MV_CC_GetOptimalPacketSize()
            if pkt > 0:
                self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", pkt)
        self.cam.MV_CC_SetEnumValue("ExposureAuto", 0)
        self.cam.MV_CC_SetEnumValue("GainAuto", 0)
        self.cam.MV_CC_SetImageNodeNum(16)
        return None

    def close(self):
        if self.is_open:
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
            self.is_open = False

    # ---- 参数读写 ----
    def enum_entries(self, key):
        """返回 (当前符号名, [可选符号名...])"""
        v = MVCC_ENUMVALUE()
        if self.cam.MV_CC_GetEnumValue(key, v) != 0:
            return None, []
        names, cur = [], None
        for i in range(v.nSupportedNum):
            e = MVCC_ENUMENTRY()
            e.nValue = v.nSupportValue[i]
            if self.cam.MV_CC_GetEnumEntrySymbolic(key, e) == 0:
                n = e.chSymbolic.decode()
                names.append(n)
                if e.nValue == v.nCurValue:
                    cur = n
        return cur, names

    def writable(self, key):
        """参数当前是否可写（不存在、不可用、被锁定成只读都返回 False）"""
        mode = MV_XML_AccessMode()
        return (self.cam.MV_XML_GetNodeAccessMode(key, mode) == 0
                and mode.value in (AM_RW, AM_WO))

    def set_enum(self, key, name):
        return self.cam.MV_CC_SetEnumValueByString(key, name)

    def get_float(self, key):
        v = MVCC_FLOATVALUE()
        if self.cam.MV_CC_GetFloatValue(key, v) != 0:
            return None
        return v.fCurValue, v.fMin, v.fMax

    def set_float(self, key, val):
        return self.cam.MV_CC_SetFloatValue(key, float(val))

    def get_int(self, key):
        v = MVCC_INTVALUE_EX()
        if self.cam.MV_CC_GetIntValueEx(key, v) != 0:
            return None
        return v.nCurValue, v.nMin, v.nMax

    def set_int(self, key, val):
        return self.cam.MV_CC_SetIntValueEx(key, int(val))

    # 去抖时间属于某个 I/O 口，先用 LineSelector 选中 Line0 再读写
    def get_debounce(self, line="Line0"):
        if self.set_enum("LineSelector", line) != 0:
            return None
        return self.get_int("LineDebouncerTime")

    def set_debounce(self, us, line="Line0"):
        r = self.set_enum("LineSelector", line)
        return r if r != 0 else self.set_int("LineDebouncerTime", us)

    def set_trigger(self, mode):
        """mode: 'off' 连续 / 'soft' 软触发 / 'line0' 硬触发"""
        if mode == "off":
            return self.set_enum("TriggerMode", "Off")
        r = self.set_enum("TriggerMode", "On")
        r |= self.set_enum("TriggerSource", "Software" if mode == "soft" else "Line0")
        return r   # 硬触发的边沿（TriggerActivation）由界面单独设置

    def soft_trigger(self):
        return self.cam.MV_CC_SetCommandValue("TriggerSoftware")

    def to_rgb(self, data, w, h, pix):
        """非 Mono8/RGB8 格式（Bayer、YUV、10/12 位等）用 SDK 转成 RGB8 供显示"""
        n = w * h * 3
        if getattr(self, "_cvt_len", 0) < n:
            self._cvt_buf, self._cvt_len = (c_ubyte * n)(), n
        src = (c_ubyte * len(data)).from_buffer_copy(data)
        p = MV_CC_PIXEL_CONVERT_PARAM_EX()
        p.nWidth, p.nHeight = w, h
        p.pSrcData = cast(src, POINTER(c_ubyte))
        p.nSrcDataLen = len(data)
        p.enSrcPixelType = pix
        p.enDstPixelType = PIX_RGB8
        p.pDstBuffer = cast(self._cvt_buf, POINTER(c_ubyte))
        p.nDstBufferSize = self._cvt_len
        if self.cam.MV_CC_ConvertPixelTypeEx(p) != 0:
            return None
        return ctypes.string_at(self._cvt_buf, n)


# ---------------------------------------------------------------------------
# 写盘线程：与 recorder.h 相同的格式和滚动策略
# ---------------------------------------------------------------------------
class Recorder(threading.Thread):
    def __init__(self, out_dir, roll_bytes=512 << 20):
        super().__init__(daemon=True)
        os.makedirs(out_dir, exist_ok=True)
        self.dir, self.roll = out_dir, roll_bytes
        self.q = queue.Queue(maxsize=64)   # 有界队列
        self.written = self.bytes = self.max_write_us = 0
        self.files = []
        self._fp, self._cur, self._idx = None, 0, 0

    def push(self, frame):
        try:
            self.q.put_nowait(frame)   # 不阻塞取帧线程
            return True
        except queue.Full:
            return False

    def stop(self):
        self.q.put(None)   # 哨兵：排空后退出
        self.join()

    def _open_next(self):
        if self._fp:
            self._fp.close()
        path = os.path.join(self.dir, f"rec_{self._idx:04d}.bin")
        self._idx += 1
        self._fp = open(path, "wb", buffering=4 << 20)
        self.files.append(path)
        self._cur = 0

    def run(self):
        self._open_next()
        while True:
            f = self.q.get()
            if f is None:
                break
            t0 = time.perf_counter()
            data = f["data"]
            self._fp.write(HDR.pack(b"FRM1", f["id"], f["dev_ts"], f["host_ts"], f["w"],
                                    f["h"], f["pix"], len(data), zlib.crc32(data)))
            self._fp.write(data)
            us = int((time.perf_counter() - t0) * 1e6)
            self.max_write_us = max(self.max_write_us, us)
            n = HDR.size + len(data)
            self.written += 1
            self.bytes += n
            self._cur += n
            if self._cur >= self.roll:
                self._open_next()
        self._fp.close()


# ---------------------------------------------------------------------------
# 取帧线程
# ---------------------------------------------------------------------------
class Grabber(threading.Thread):
    def __init__(self, camera):
        super().__init__(daemon=True)
        self.camera = camera
        self.running = True
        self.lock = threading.Lock()
        self.latest = None          # 最新一帧（给显示用，只保留一帧）
        self.recorder = None
        self.received = self.lost_pkt = self.gaps = self.rec_drop = 0
        self._last_id = None

    def run(self):
        cam = self.camera.cam
        out = MV_FRAME_OUT()
        while self.running and not self.camera.lost:
            ctypes.memset(byref(out), 0, sizeof(out))
            if cam.MV_CC_GetImageBuffer(out, 500) != 0:
                continue                      # 超时：触发模式下没信号属正常
            fi = out.stFrameInfo
            n = fi.nFrameLenEx or fi.nFrameLen
            frame = dict(data=ctypes.string_at(out.pBufAddr, n), id=fi.nFrameNum,
                         dev_ts=(fi.nDevTimeStampHigh << 32) | fi.nDevTimeStampLow,
                         host_ts=time.monotonic_ns(), w=fi.nWidth, h=fi.nHeight,
                         pix=int(fi.enPixelType))
            lost = fi.nLostPacket
            cam.MV_CC_FreeImageBuffer(out)    # 尽快还给 SDK

            self.received += 1
            if lost:
                self.lost_pkt += 1
            if self._last_id is not None and frame_id_missing(self._last_id, frame["id"]) != 0:
                self.gaps += 1
            self._last_id = frame["id"]
            with self.lock:
                self.latest = frame
            rec = self.recorder
            if rec and not rec.push(frame):
                self.rec_drop += 1

    def take_latest(self):
        with self.lock:
            f, self.latest = self.latest, None
        return f


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("海康相机采集控制")
        self.resize(1200, 760)
        self.camera = Camera()
        self.grabber = None
        self.recorder = None
        self.devs, self._dev_list = [], None
        self._has_delay = self._has_debounce = False   # 打开相机后按相机实际能力更新
        self._build_ui()
        self._stat_t, self._stat_recv, self._stat_bytes = time.monotonic(), 0, 0
        self._disp_count, self.cam_fps, self.disp_fps, self.rec_mbps = 0, 0.0, 0.0, 0.0

        self.view_timer = QTimer(self, timeout=self._refresh_view)   # 显示 + 统计
        self.view_timer.start(33)
        self.trig_timer = QTimer(self, timeout=self._do_soft_trigger)  # 自动软触发
        self.refresh_devices()
        self._update_state()

    # ---------- 界面 ----------
    def _build_ui(self):
        # 相机
        self.cb_dev = QComboBox()
        self.btn_refresh = QPushButton("刷新")
        self.btn_open = QPushButton("打开相机")
        g_dev = QGroupBox("相机")
        l = QGridLayout(g_dev)
        l.addWidget(self.cb_dev, 0, 0, 1, 2)
        l.addWidget(self.btn_refresh, 1, 0)
        l.addWidget(self.btn_open, 1, 1)

        # 参数
        self.cb_pix = QComboBox()
        self.sp_exp = QDoubleSpinBox(decimals=0, suffix=" us", singleStep=500)
        self.sp_gain = QDoubleSpinBox(decimals=1, suffix=" dB", singleStep=1)
        self.sp_fps = QDoubleSpinBox(decimals=1, suffix=" fps", singleStep=5)
        for sp in (self.sp_exp, self.sp_gain, self.sp_fps):
            sp.setKeyboardTracking(False)   # 输完回车/失焦才下发，避免每敲一位都设置
        g_par = QGroupBox("参数")
        f = QFormLayout(g_par)
        f.addRow("像素格式", self.cb_pix)
        f.addRow("曝光", self.sp_exp)
        f.addRow("增益", self.sp_gain)
        f.addRow("帧率", self.sp_fps)

        # 触发
        self.rb_off = QRadioButton("连续采集")
        self.rb_soft = QRadioButton("软触发")
        self.rb_line = QRadioButton("硬触发 Line0")
        self.rb_off.setChecked(True)
        self.trig_group = QButtonGroup(self)
        for i, rb in enumerate((self.rb_off, self.rb_soft, self.rb_line)):
            self.trig_group.addButton(rb, i)
        self.btn_trig = QPushButton("触发一次")
        self.ck_auto = QCheckBox("自动触发，间隔")
        self.sp_interval = QSpinBox(minimum=10, maximum=10000, value=100, suffix=" ms")
        self.sp_delay = QDoubleSpinBox(decimals=0, suffix=" us", singleStep=100)
        self.sp_delay.setToolTip("触发延时：收到触发信号后等待多久再曝光。\n"
                                 "用于传感器和相机视野之间有距离时，让产品走到画面中间再拍。")
        self.sp_debounce = QSpinBox(suffix=" us", singleStep=10)
        self.sp_debounce.setToolTip("去抖时间（Line0）：输入信号必须保持稳定这么久才算有效，\n"
                                    "用来过滤毛刺，防止一次信号触发多次。只影响硬触发。")
        for sp in (self.sp_delay, self.sp_debounce):
            sp.setKeyboardTracking(False)
        self.cb_edge = QComboBox()
        self.cb_edge.setToolTip("硬触发信号怎样算一次触发：\n"
                                "上升沿：低→高的瞬间；下降沿：高→低的瞬间（低电平有效的传感器用这个）；\n"
                                "任意边沿：两种变化都触发；高/低电平：电平保持期间有效（部分相机会因此锁定触发延时）。")
        g_trig = QGroupBox("触发")
        v = QVBoxLayout(g_trig)
        for rb in (self.rb_off, self.rb_soft, self.rb_line):
            v.addWidget(rb)
        ft = QFormLayout()
        ft.addRow("触发边沿", self.cb_edge)
        ft.addRow("触发延时", self.sp_delay)
        ft.addRow("去抖 (Line0)", self.sp_debounce)
        v.addLayout(ft)
        h = QHBoxLayout()
        h.addWidget(self.ck_auto)
        h.addWidget(self.sp_interval)
        v.addWidget(self.btn_trig)
        v.addLayout(h)

        # 采集 / 录制
        self.btn_grab = QPushButton("开始采集")
        self.ed_dir = QLineEdit(os.path.join(os.getcwd(), "rec_gui"))
        self.btn_dir = QPushButton("…")
        self.btn_rec = QPushButton("开始录制")
        g_rec = QGroupBox("采集 / 录制")
        v2 = QVBoxLayout(g_rec)
        v2.addWidget(self.btn_grab)
        h2 = QHBoxLayout()
        h2.addWidget(QLabel("目录"))
        h2.addWidget(self.ed_dir, 1)
        h2.addWidget(self.btn_dir)
        v2.addLayout(h2)
        v2.addWidget(self.btn_rec)

        left = QVBoxLayout()
        for g in (g_dev, g_par, g_trig, g_rec):
            left.addWidget(g)
        left.addStretch(1)
        left_w = QWidget()
        left_w.setLayout(left)
        left_w.setFixedWidth(320)

        # 右侧：预览 + 统计
        self.view = QLabel("未打开相机", alignment=Qt.AlignCenter)
        self.view.setMinimumSize(480, 360)
        self.view.setStyleSheet("background:#202020; color:#aaa; font-size:16px;")
        self.lb_stat = QLabel()
        self.lb_stat.setStyleSheet("font-family: monospace;")
        self.lb_rec = QLabel()
        self.lb_rec.setStyleSheet("font-family: monospace;")
        right = QVBoxLayout()
        right.addWidget(self.view, 1)
        right.addWidget(self.lb_stat)
        right.addWidget(self.lb_rec)

        root = QWidget()
        hl = QHBoxLayout(root)
        hl.addWidget(left_w)
        hl.addLayout(right, 1)
        self.setCentralWidget(root)

        # 信号
        self.btn_refresh.clicked.connect(self.refresh_devices)
        self.btn_open.clicked.connect(self.toggle_open)
        self.cb_pix.activated.connect(self._set_pixfmt)
        self.sp_exp.valueChanged.connect(lambda x: self._set_float("ExposureTime", x))
        self.sp_gain.valueChanged.connect(lambda x: self._set_float("Gain", x))
        self.sp_fps.valueChanged.connect(self._set_fps)
        self.trig_group.idClicked.connect(self._set_trigger)
        self.cb_edge.activated.connect(self._set_edge)
        self.sp_delay.valueChanged.connect(lambda x: self._set_float("TriggerDelay", x))
        self.sp_debounce.valueChanged.connect(self._set_debounce)
        self.btn_trig.clicked.connect(self._do_soft_trigger)
        self.ck_auto.toggled.connect(self._toggle_auto_trigger)
        self.sp_interval.valueChanged.connect(
            lambda ms: self.trig_timer.setInterval(ms))
        self.btn_grab.clicked.connect(self.toggle_grab)
        self.btn_dir.clicked.connect(self._choose_dir)
        self.btn_rec.clicked.connect(self.toggle_record)

    def _update_state(self):
        opened = self.camera.is_open
        grabbing = self.grabber is not None
        recording = self.recorder is not None
        soft = self.rb_soft.isChecked()
        self.cb_dev.setEnabled(not opened)
        self.btn_refresh.setEnabled(not opened)
        self.btn_open.setText("关闭相机" if opened else "打开相机")
        self.btn_open.setEnabled(bool(self.devs) or opened)
        self.cb_pix.setEnabled(opened and not grabbing)   # 改格式会改变帧大小，采集中不允许
        for w in (self.sp_exp, self.sp_gain, self.sp_fps, self.rb_off, self.rb_soft,
                  self.rb_line):
            w.setEnabled(opened)
        self.sp_fps.setEnabled(opened and self.rb_off.isChecked())
        # 触发延时：软/硬触发都有效；去抖：只作用于硬件输入口。相机不支持的保持禁用
        self.cb_edge.setEnabled(opened and self.rb_line.isChecked() and self.cb_edge.count() > 0)
        self.sp_delay.setEnabled(opened and not self.rb_off.isChecked() and self._has_delay)
        self.sp_debounce.setEnabled(opened and self.rb_line.isChecked() and self._has_debounce)
        self.btn_trig.setEnabled(grabbing and soft)
        self.ck_auto.setEnabled(grabbing and soft)
        self.btn_grab.setEnabled(opened)
        self.btn_grab.setText("停止采集" if grabbing else "开始采集")
        self.btn_rec.setEnabled(grabbing)
        self.btn_rec.setText("停止录制" if recording else "开始录制")
        self.ed_dir.setEnabled(not recording)
        self.btn_dir.setEnabled(not recording)

    def _error(self, msg):
        self.statusBar().showMessage(msg, 8000)

    # ---------- 相机 ----------
    def refresh_devices(self):
        self.devs, self._dev_list = Camera.enum_devices()
        self.cb_dev.clear()
        for model, serial, _ in self.devs:
            self.cb_dev.addItem(f"{model}  ({serial})")
        if not self.devs:
            self.cb_dev.addItem("未找到相机")
        self.statusBar().showMessage(f"找到 {len(self.devs)} 台相机", 5000)
        self._update_state()

    def toggle_open(self):
        if self.camera.is_open:
            self.stop_grab()
            self.camera.close()
            self.view.setText("未打开相机")
        else:
            idx = self.cb_dev.currentIndex()
            if not (0 <= idx < len(self.devs)):
                return
            err = self.camera.open(self.devs[idx][2])
            if err:
                QMessageBox.warning(self, "打开相机", err)
                return
            self._load_params()
            self.view.setText("已打开，点击「开始采集」")
        self._update_state()

    def _load_params(self):
        """从相机读回当前值和范围，填到界面上"""
        cur, names = self.camera.enum_entries("PixelFormat")
        self.cb_pix.clear()
        self.cb_pix.addItems(names)
        if cur:
            self.cb_pix.setCurrentText(cur)
        for sp, key in ((self.sp_exp, "ExposureTime"), (self.sp_gain, "Gain"),
                        (self.sp_fps, "AcquisitionFrameRate")):
            v = self.camera.get_float(key)
            sp.blockSignals(True)
            if v:
                sp.setRange(v[1], v[2])
                sp.setValue(v[0])
            sp.setEnabled(v is not None)
            sp.blockSignals(False)
        v = self.camera.get_float("TriggerDelay")
        self._has_delay = v is not None and self.camera.writable("TriggerDelay")
        self.sp_delay.blockSignals(True)
        if v:
            self.sp_delay.setRange(v[1], v[2])
            self.sp_delay.setValue(v[0])
        self.sp_delay.blockSignals(False)
        v = self.camera.get_debounce()
        self._has_debounce = v is not None and self.camera.writable("LineDebouncerTime")
        self.sp_debounce.blockSignals(True)
        if v:
            self.sp_debounce.setRange(int(v[1]), int(min(v[2], 2**31 - 1)))
            self.sp_debounce.setValue(int(v[0]))
        self.sp_debounce.blockSignals(False)

        cur_trig, _ = self.camera.enum_entries("TriggerMode")
        self.rb_off.setChecked(cur_trig != "On")
        if cur_trig == "On":
            src, _ = self.camera.enum_entries("TriggerSource")
            (self.rb_soft if src == "Software" else self.rb_line).setChecked(True)
        self._load_edges()

    def _load_edges(self):
        """读触发边沿选项。软触发下相机不提供这个参数，所以只在硬触发时读"""
        self.cb_edge.clear()
        if not self.rb_line.isChecked():
            return
        cur, names = self.camera.enum_entries("TriggerActivation")
        for n in names:
            self.cb_edge.addItem(EDGE_NAMES.get(n, n), n)
        if cur:
            self.cb_edge.setCurrentIndex(self.cb_edge.findData(cur))

    def _set_edge(self):
        name = self.cb_edge.currentData()
        r = self.camera.set_enum("TriggerActivation", name)
        if r != 0:
            self._error(f"设置触发边沿 {EDGE_NAMES.get(name, name)} 失败 0x{r:08x}")
        self._load_edges()   # 以相机实际值为准
        # 电平触发时部分相机会锁定触发延时，刷新一下它的可用状态
        self._has_delay = self.camera.writable("TriggerDelay")
        self._update_state()

    def _set_pixfmt(self):
        name = self.cb_pix.currentText()
        r = self.camera.set_enum("PixelFormat", name)
        if r != 0:
            self._error(f"设置像素格式 {name} 失败 0x{r:08x}（黑白相机不支持彩色格式）")
            self._load_params()

    def _set_float(self, key, val):
        r = self.camera.set_float(key, val)
        if r != 0:
            self._error(f"设置 {key}={val} 失败 0x{r:08x}")

    def _set_debounce(self, val):
        r = self.camera.set_debounce(val)
        if r != 0:
            self._error(f"设置去抖时间 {val} us 失败 0x{r:08x}")

    def _set_fps(self, val):
        self.camera.cam.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True)
        self._set_float("AcquisitionFrameRate", val)

    def _set_trigger(self, idx):
        mode = ("off", "soft", "line0")[idx]
        r = self.camera.set_trigger(mode)
        if r != 0:
            self._error(f"设置触发模式失败 0x{r:08x}")
        if mode != "soft":
            self.ck_auto.setChecked(False)
        self._load_edges()
        self._has_delay = self.camera.writable("TriggerDelay")
        self._update_state()

    def _do_soft_trigger(self):
        r = self.camera.soft_trigger()
        if r != 0:
            self._error(f"软触发失败 0x{r:08x}")

    def _toggle_auto_trigger(self, on):
        if on:
            self.trig_timer.start(self.sp_interval.value())
        else:
            self.trig_timer.stop()

    # ---------- 采集 / 录制 ----------
    def toggle_grab(self):
        if self.grabber:
            self.stop_grab()
        else:
            r = self.camera.cam.MV_CC_StartGrabbing()
            if r != 0:
                self._error(f"开始采集失败 0x{r:08x}")
                return
            self.grabber = Grabber(self.camera)
            self.grabber.start()
            self._stat_t, self._stat_recv, self._disp_count = time.monotonic(), 0, 0
        self._update_state()

    def stop_grab(self):
        self.ck_auto.setChecked(False)
        if self.recorder:
            self.toggle_record()
        if self.grabber:
            self.grabber.running = False
            self.grabber.join()
            self.camera.cam.MV_CC_StopGrabbing()
            self.grabber = None
        self._update_state()

    def _choose_dir(self):
        d = QFileDialog.getExistingDirectory(self, "录制目录", self.ed_dir.text())
        if d:
            self.ed_dir.setText(d)

    def toggle_record(self):
        if self.recorder:
            rec, self.recorder = self.recorder, None
            if self.grabber:
                self.grabber.recorder = None
            rec.stop()   # 排空队列后退出
            self.statusBar().showMessage(
                f"录制完成：{rec.written} 帧，{rec.bytes / 1e6:.0f} MB → {rec.dir}", 10000)
            self._last_rec = rec
        else:
            try:
                self.recorder = Recorder(self.ed_dir.text())
            except OSError as e:
                self._error(f"无法创建录制目录: {e}")
                return
            self.recorder.start()
            self.grabber.rec_drop = 0
            self._stat_bytes = 0
            self.grabber.recorder = self.recorder
        self._update_state()

    # ---------- 显示 + 统计（GUI 线程，约 30 次/秒） ----------
    def _refresh_view(self):
        g = self.grabber
        if g:
            if self.camera.lost:
                self._error("相机掉线！")
                self.stop_grab()
                self.camera.close()
                self.view.setText("相机掉线，请刷新后重新打开")
                self._update_state()
                return
            f = g.take_latest()
            if f:
                self._show(f)
                self._disp_count += 1

        now = time.monotonic()
        if now - self._stat_t >= 1.0:   # 每秒刷新一次统计
            dt = now - self._stat_t
            if g:
                self.cam_fps = (g.received - self._stat_recv) / dt
                self._stat_recv = g.received
            rec = self.recorder
            if rec:
                self.rec_mbps = (rec.bytes - self._stat_bytes) / dt / 1e6
                self._stat_bytes = rec.bytes
            self.disp_fps = self._disp_count / dt
            self._disp_count, self._stat_t = 0, now
            self._update_stats()

    def _show(self, f):
        w, h, pix, data = f["w"], f["h"], f["pix"], f["data"]
        if pix == PIX_MONO8:
            img = QImage(data, w, h, w, QImage.Format_Grayscale8)
        elif pix == PIX_RGB8:
            img = QImage(data, w, h, w * 3, QImage.Format_RGB888)
        else:
            rgb = self.camera.to_rgb(data, w, h, pix)
            if rgb is None:
                self.view.setText(f"无法显示像素格式 0x{pix:08x}")
                return
            img, data = QImage(rgb, w, h, w * 3, QImage.Format_RGB888), rgb
        # 先缩放到窗口大小再显示：600 万像素原图直接显示会很卡
        pm = QPixmap.fromImage(img).scaled(self.view.size(), Qt.KeepAspectRatio,
                                           Qt.FastTransformation)
        self.view.setPixmap(pm)
        self._last_info = (f["id"], w, h, pix)

    def _update_stats(self):
        g = self.grabber
        if g:
            fid, w, h, pix = getattr(self, "_last_info", (0, 0, 0, 0))
            fmt = PIX_NAMES.get(pix, f"0x{pix:08x}")
            self.lb_stat.setText(
                f"采集 {self.cam_fps:5.1f} fps  显示 {self.disp_fps:5.1f} fps  "
                f"已收 {g.received} 帧  帧号 {fid}  {w}x{h} {fmt}  "
                f"丢包帧 {g.lost_pkt}  帧号缺口 {g.gaps}")
        else:
            self.lb_stat.setText("未采集")
        rec = self.recorder
        if rec:
            self.lb_rec.setText(
                f"● 录制中  写入 {rec.written} 帧  {rec.bytes / 1e6:.0f} MB  "
                f"{self.rec_mbps:.1f} MB/s  队列 {rec.q.qsize()}/64  "
                f"丢弃 {g.rec_drop if g else 0}  最大写盘 {rec.max_write_us} us")
            self.lb_rec.setStyleSheet("font-family: monospace; color: #d33;")
        else:
            self.lb_rec.setText("未录制")
            self.lb_rec.setStyleSheet("font-family: monospace;")

    def closeEvent(self, e):
        self.stop_grab()
        self.camera.close()
        e.accept()


def main():
    MvCamera.MV_CC_Initialize()        # SDK 4.x 要求先初始化
    app = QApplication(sys.argv)
    app.setFont(QFont("WenQuanYi Micro Hei", 10))
    win = MainWindow()
    win.show()
    ret = app.exec_()
    MvCamera.MV_CC_Finalize()
    return ret


if __name__ == "__main__":
    sys.exit(main())

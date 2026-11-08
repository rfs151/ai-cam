#!/usr/bin/env python3
"""PyQt5 小窗口：验证 WSL 图形界面（WSLg），并可回放采集程序录制的 .bin 文件。

用法: python3 tools/qt_demo.py [rec_0000.bin]
  不带参数：播放程序生成的动画测试画面，显示实际刷新帧率
  带 .bin：按录制格式（recorder.h 的 FRM1 帧头）逐帧回放，支持 Mono8 / RGB8
安装: sudo apt install python3-pyqt5
"""
import os
import struct
import sys

from PyQt5.QtCore import QElapsedTimer, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPixmap
from PyQt5.QtWidgets import (QApplication, QFileDialog, QHBoxLayout, QLabel,
                             QPushButton, QSlider, QVBoxLayout, QWidget)

HDR = struct.Struct("<4sQQQIIIII")   # 与 recorder.h 中 FrameRecordHeader 一致
PIX_MONO8, PIX_RGB8 = 0x01080001, 0x02180014


def index_frames(path):
    """扫描 .bin，返回每帧 (文件偏移, 帧头字典)，不读图像数据。"""
    frames = []
    with open(path, "rb") as f:
        while True:
            pos = f.tell()
            raw = f.read(HDR.size)
            if len(raw) < HDR.size:
                break
            magic, fid, dts, hts, w, h, pix, size, crc = HDR.unpack(raw)
            if magic != b"FRM1":
                break
            frames.append((pos, dict(id=fid, w=w, h=h, pix=pix, size=size)))
            f.seek(size, 1)
    return frames


class Window(QWidget):
    def __init__(self, path=None):
        super().__init__()
        self.setWindowTitle("cam_demo PyQt 测试窗口")
        self.resize(960, 720)

        self.view = QLabel(alignment=Qt.AlignCenter)
        self.view.setMinimumSize(320, 240)
        self.view.setStyleSheet("background:#202020; color:#ccc;")
        self.info = QLabel()
        self.env = QLabel()
        self.env.setStyleSheet("color:gray;")

        self.btn_play = QPushButton("暂停")
        self.btn_open = QPushButton("打开录制文件 (.bin)…")
        self.btn_demo = QPushButton("测试画面")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setEnabled(False)

        bar = QHBoxLayout()
        for w in (self.btn_play, self.btn_demo, self.btn_open):
            bar.addWidget(w)
        bar.addWidget(self.slider, 1)
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel("你好，WSLg！这个窗口由 WSL 里的 Python/Qt 程序创建。"))
        lay.addWidget(self.view, 1)
        lay.addLayout(bar)
        lay.addWidget(self.info)
        lay.addWidget(self.env)

        self.btn_play.clicked.connect(self.toggle_play)
        self.btn_open.clicked.connect(self.choose_file)
        self.btn_demo.clicked.connect(self.use_demo)
        self.slider.valueChanged.connect(self.seek)

        self.env.setText(
            f"Qt 平台={QApplication.platformName()}  DISPLAY={os.environ.get('DISPLAY', '-')}  "
            f"WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY', '-')}")

        self.file = None        # 回放文件句柄
        self.frames = []        # 帧索引
        self.cur = 0            # 当前帧 / 动画计数
        self.timer = QTimer(self, timeout=self.tick)
        self.fps_clock = QElapsedTimer()
        self.fps_count, self.fps = 0, 0.0

        if path:
            self.load(path)
        else:
            self.use_demo()

    # ---------- 数据源 ----------
    def use_demo(self):
        if self.file:
            self.file.close()
        self.file, self.frames, self.cur = None, [], 0
        self.slider.setEnabled(False)
        self.start(30)

    def choose_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择录制文件", os.getcwd(),
                                              "录制文件 (*.bin);;所有文件 (*)")
        if path:
            self.load(path)

    def load(self, path):
        frames = index_frames(path)
        if not frames:
            self.info.setText(f"不是有效的录制文件: {path}")
            return
        if self.file:
            self.file.close()
        self.file, self.frames, self.cur = open(path, "rb"), frames, 0
        self.slider.blockSignals(True)
        self.slider.setRange(0, len(frames) - 1)
        self.slider.setValue(0)
        self.slider.blockSignals(False)
        self.slider.setEnabled(True)
        self.setWindowTitle(f"cam_demo 回放 - {os.path.basename(path)}")
        self.start(30)

    # ---------- 播放控制 ----------
    def start(self, fps):
        self.timer.start(int(1000 / fps))
        self.btn_play.setText("暂停")
        self.fps_clock.start()
        self.fps_count = 0

    def toggle_play(self):
        if self.timer.isActive():
            self.timer.stop()
            self.btn_play.setText("播放")
        else:
            self.start(30)

    def seek(self, i):
        self.cur = i
        self.show_frame()

    def tick(self):
        if self.frames:
            self.cur = (self.cur + 1) % len(self.frames)   # 播到结尾循环
            self.slider.blockSignals(True)
            self.slider.setValue(self.cur)
            self.slider.blockSignals(False)
        else:
            self.cur += 1
        self.show_frame()

        # 每秒统计一次实际刷新帧率（反映 WSLg 显示性能）
        self.fps_count += 1
        if self.fps_clock.elapsed() >= 1000:
            self.fps = self.fps_count * 1000.0 / self.fps_clock.restart()
            self.fps_count = 0

    # ---------- 画面 ----------
    def demo_image(self):
        w, h = 640, 480
        img = QImage(w, h, QImage.Format_RGB888)
        img.fill(QColor("#1e3a5f"))
        p = QPainter(img)
        x = (self.cur * 8) % (w + 200) - 200      # 移动的色块，看是否流畅
        p.fillRect(x, 160, 200, 160, QColor.fromHsv((self.cur * 3) % 360, 200, 255))
        p.setPen(Qt.white)
        p.setFont(QFont("WenQuanYi Micro Hei", 28))
        p.drawText(img.rect(), Qt.AlignTop | Qt.AlignHCenter, f"测试画面  第 {self.cur} 帧")
        p.end()
        return img, f"测试画面 640x480"

    def file_image(self):
        pos, hd = self.frames[self.cur]
        self.file.seek(pos + HDR.size)
        data = self.file.read(hd["size"])
        w, h = hd["w"], hd["h"]
        if hd["pix"] == PIX_RGB8:
            img = QImage(data, w, h, w * 3, QImage.Format_RGB888)
        else:  # Mono8，其它格式（如 Bayer）也按灰度显示
            img = QImage(data, w, h, w, QImage.Format_Grayscale8)
        img = img.copy()   # QImage 不持有 data，复制一份防止 data 被回收
        fmt = {PIX_RGB8: "RGB8", PIX_MONO8: "Mono8"}.get(hd["pix"], hex(hd["pix"]))
        return img, f"帧号 {hd['id']}  {w}x{h} {fmt}  [{self.cur + 1}/{len(self.frames)}]"

    def show_frame(self):
        img, desc = self.file_image() if self.frames else self.demo_image()
        # 先缩放到窗口大小再显示：大图（如 600 万像素）直接显示会很卡
        pix = QPixmap.fromImage(img).scaled(self.view.size(), Qt.KeepAspectRatio,
                                            Qt.FastTransformation)
        self.view.setPixmap(pix)
        self.info.setText(f"{desc}    显示帧率 {self.fps:.1f} fps")


def main():
    app = QApplication(sys.argv)
    app.setFont(QFont("WenQuanYi Micro Hei", 10))   # 指定中文字体，避免方框
    win = Window(sys.argv[1] if len(sys.argv) > 1 else None)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

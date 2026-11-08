"""ai-cam 网页演示后端。

    浏览器 ──WebSocket /ws──▶ 本服务 ──stdin 文本命令──▶ capd（C++ 采集引擎）
           ◀── 状态/统计 JSON + 预览 JPEG ──   ◀── stdout JSON ──┘
                              └─ 启停 arv-fake-gv-camera（虚拟 GigE 相机：拔网线、丢包率）

公网演示的约束：
- 同一时间只有一个人能操作（控制权），其他人旁观；3 分钟没有操作自动释放。
- 访客输入只按白名单转成命令，不直接拼进命令行。
- 录像最长 10 秒，只保留最近 3 段；没有人访问 10 分钟后恢复默认设置。

运行：uvicorn app:app --host 0.0.0.0 --port 8098（工作目录 web/api），或见 ctl.sh / docker-compose.yml
"""
import asyncio
import base64
import contextlib
import itertools
import json
import os
import re
import shutil
import socket
import struct
import sys
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

ROOT = Path(__file__).resolve().parents[2]
CAPD = os.environ.get("CAPD", str(ROOT / "build" / "capd"))
VERIFY = os.environ.get("VERIFY", str(ROOT / "tools" / "verify_record.py"))
FAKE_CAM = os.environ.get("FAKE_CAM", "arv-fake-gv-camera-0.8")
REC_ROOT = Path(os.environ.get("REC_ROOT", "/tmp/ai-cam-rec"))
STATIC = Path(__file__).resolve().parents[1] / "static"

CONTROL_IDLE_S = 180      # 控制权空闲多久自动释放
RESET_IDLE_S = 600        # 没有访客多久恢复默认
REC_MAX_S = 10
REC_KEEP = 3

DEFAULTS = {"exposure": "5000", "fps": "30", "size": "512x512", "pixfmt": "Mono8", "trigger": "none",
            "auto_trigger": "0", "packet_size": "1400", "buffers": "16", "process_ms": "0", "slow_disk_us": "0"}


def iface_addr(name):
    """网卡的 IPv4 地址（Linux），取不到返回 None。"""
    try:
        import fcntl
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        packed = fcntl.ioctl(s.fileno(), 0x8915, struct.pack("256s", name[:15].encode()))  # SIOCGIFADDR
        return socket.inet_ntoa(packed[20:24])
    except OSError:
        return None


def camera_plan():
    """要启动的虚拟相机：GV01 在回环地址；有 eth0 时再在 eth0 上起一台 GV02（不用改系统网络配置）。
    可用环境变量 FAKE_CAMS="GV01@127.0.0.1,GV02@eth0" 覆盖。"""
    spec = os.environ.get("FAKE_CAMS")
    if spec:
        return [tuple(x.split("@", 1)) for x in spec.split(",") if "@" in x]
    plan = [("GV01", "127.0.0.1")]
    if iface_addr("eth0"):
        plan.append(("GV02", "eth0"))
    return plan


# ---------------------------------------------------------------------------
# 虚拟相机进程
# ---------------------------------------------------------------------------
class FakeCam:
    def __init__(self, serial, iface):
        self.serial, self.iface = serial, iface
        self.loss = 0            # 丢包率，千分比
        self.plugged = True      # False = 模拟拔网线（进程结束）
        self.proc = None

    async def start(self):
        await self.stop()
        args = [FAKE_CAM, "-i", self.iface, "-s", self.serial]
        if self.loss:
            args += ["-r", str(self.loss)]
        self.proc = await asyncio.create_subprocess_exec(
            *args, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)

    async def stop(self):
        if self.proc and self.proc.returncode is None:
            self.proc.terminate()
            try:
                await asyncio.wait_for(self.proc.wait(), 3)
            except asyncio.TimeoutError:
                self.proc.kill()
                await self.proc.wait()
        self.proc = None

    async def apply(self):
        if self.plugged:
            await self.start()
        else:
            await self.stop()

    def view(self):
        alive = bool(self.proc and self.proc.returncode is None)
        return {"serial": self.serial, "iface": self.iface, "loss": self.loss, "plugged": self.plugged, "running": alive}


# ---------------------------------------------------------------------------
# 演示服务：capd 进程 + 访客 + 控制权
# ---------------------------------------------------------------------------
class Client:
    ids = itertools.count(1)

    def __init__(self, ws):
        self.ws = ws
        self.id = next(Client.ids)
        self.q = asyncio.Queue(maxsize=32)
        self.task = None

    def offer(self, kind, data):
        """放进发送队列；队列满时丢掉预览帧，状态消息挤掉最旧的一条。"""
        try:
            self.q.put_nowait((kind, data))
        except asyncio.QueueFull:
            if kind == "bytes":
                return
            with contextlib.suppress(asyncio.QueueEmpty):
                self.q.get_nowait()
            with contextlib.suppress(asyncio.QueueFull):
                self.q.put_nowait((kind, data))

    async def pump(self):
        while True:
            kind, data = await self.q.get()
            if kind == "bytes":
                await self.ws.send_bytes(data)
            else:
                await self.ws.send_text(data)


class Demo:
    def __init__(self):
        self.cams = [FakeCam(s, i) for s, i in camera_plan()]
        self.capd = None
        self.clients = {}
        self.controller = None
        self.control_until = 0.0
        self.params = dict(DEFAULTS)
        self.selected = self.cams[0].serial
        # 最近一次的各类消息，新访客连上时先补发
        self.last = {"cameras": None, "info": None, "state": None, "stats": None, "record": None, "verify": None}
        self.logs = []
        self.last_seen = time.monotonic()
        self.recordings = []

    # ---------- 启动 ----------
    async def startup(self):
        REC_ROOT.mkdir(parents=True, exist_ok=True)
        for c in self.cams:
            await c.start()
        await asyncio.sleep(0.8)
        await self.start_capd()
        await self.apply_defaults()
        asyncio.create_task(self.watchdog())

    async def shutdown(self):
        if self.capd and self.capd.returncode is None:
            with contextlib.suppress(Exception):
                await self.send("quit")
                await asyncio.wait_for(self.capd.wait(), 3)
            if self.capd.returncode is None:
                self.capd.kill()
        for c in self.cams:
            await c.stop()

    async def start_capd(self):
        self.capd = await asyncio.create_subprocess_exec(
            CAPD, stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, limit=16 * 1024 * 1024)
        asyncio.create_task(self.read_capd(self.capd))

    async def apply_defaults(self):
        """恢复默认：所有相机插上、不丢包，打开第一台并开始连续采集。"""
        for c in self.cams:
            changed = c.loss or not c.plugged
            c.loss, c.plugged = 0, True
            if changed or not c.proc:
                await c.apply()
        self.params = dict(DEFAULTS)
        self.selected = self.cams[0].serial
        await self.send("list")
        for k, v in self.params.items():
            await self.send(f"set {k} {v}")
        await self.send(f"open {self.selected}")
        await self.send("start")
        self.broadcast_meta()

    async def send(self, line):
        if not self.capd or self.capd.returncode is not None:
            return
        self.capd.stdin.write((line + "\n").encode())
        await self.capd.stdin.drain()

    # ---------- capd 输出 ----------
    async def read_capd(self, proc):
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            try:
                m = json.loads(line)
            except ValueError:
                continue
            t = m.get("type")
            if t == "preview":
                jpeg = base64.b64decode(m.pop("jpeg"))
                for c in list(self.clients.values()):
                    c.offer("bytes", jpeg)
                continue
            if t in self.last:
                self.last[t] = m
            if t == "log":
                self.logs = (self.logs + [m])[-50:]
            self.broadcast(m)
            if t == "record" and m.get("state") == "done":
                asyncio.create_task(self.verify(m))
        await proc.wait()
        if proc is self.capd:   # 意外退出：1 秒后重启并恢复
            self.log("error", f"capd 退出（返回码 {proc.returncode}），1 秒后重启")
            await asyncio.sleep(1)
            await self.start_capd()
            await self.restore_params()

    async def restore_params(self):
        await self.send("list")
        for k, v in self.params.items():
            await self.send(f"set {k} {v}")
        await self.send(f"open {self.selected}")
        await self.send("start")

    async def verify(self, rec):
        files = rec.get("files") or []
        if not files:
            return
        p = await asyncio.create_subprocess_exec(sys.executable, VERIFY, "--json", *files,
                                                 stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        out, _ = await p.communicate()
        try:
            v = json.loads(out.decode().strip().splitlines()[-1])
        except (ValueError, IndexError):
            v = {"error": "校验失败"}
        v["type"] = "verify"
        v["record"] = {k: rec.get(k) for k in ("secs", "written", "drop_pool", "drop_queue", "file_gaps", "max_write_us")}
        self.last["verify"] = v
        self.broadcast(v)

    def log(self, level, msg):
        m = {"type": "log", "level": level, "msg": msg}
        self.logs = (self.logs + [m])[-50:]
        self.broadcast(m)

    # ---------- 广播 ----------
    def broadcast(self, m):
        s = json.dumps(m, ensure_ascii=False)
        for c in list(self.clients.values()):
            c.offer("text", s)

    def meta(self):
        left = max(0, int(self.control_until - time.monotonic())) if self.controller else 0
        return {"type": "meta", "controller": self.controller, "control_left": left, "viewers": len(self.clients),
                "cams": [c.view() for c in self.cams], "params": self.params, "selected": self.selected,
                "limits": {"rec_max_s": REC_MAX_S, "control_idle_s": CONTROL_IDLE_S}}

    def broadcast_meta(self):
        self.broadcast(self.meta())

    # ---------- 访客 ----------
    async def join(self, ws):
        c = Client(ws)
        self.clients[c.id] = c
        self.last_seen = time.monotonic()
        c.task = asyncio.create_task(c.pump())
        c.offer("text", json.dumps({"type": "welcome", "id": c.id}))
        for k in ("cameras", "info", "state", "stats", "record", "verify"):
            if self.last[k]:
                c.offer("text", json.dumps(self.last[k], ensure_ascii=False))
        for m in self.logs[-10:]:
            c.offer("text", json.dumps(m, ensure_ascii=False))
        self.broadcast_meta()
        return c

    def leave(self, c):
        self.clients.pop(c.id, None)
        if c.task:
            c.task.cancel()
        if self.controller == c.id:
            self.controller = None
        self.last_seen = time.monotonic()
        self.broadcast_meta()

    async def watchdog(self):
        while True:
            await asyncio.sleep(1)
            now = time.monotonic()
            if self.controller and now > self.control_until:
                self.controller = None
                self.log("info", "控制权空闲超时，已释放")
            if self.clients:
                self.last_seen = now
            elif now - self.last_seen > RESET_IDLE_S:
                self.last_seen = now
                await self.apply_defaults()
            if self.clients:
                self.broadcast_meta()
            for c in self.cams:   # 插着网线的虚拟相机意外退出时拉起来
                if c.plugged and (not c.proc or c.proc.returncode is not None):
                    await c.start()

    # ---------- 访客操作 ----------
    async def handle(self, c, m):
        op = m.get("op")
        now = time.monotonic()
        if op == "take":
            if self.controller in (None, c.id):
                self.controller = c.id
                self.control_until = now + CONTROL_IDLE_S
            self.broadcast_meta()
            return
        if op == "release":
            if self.controller == c.id:
                self.controller = None
            self.broadcast_meta()
            return
        if self.controller != c.id:
            c.offer("text", json.dumps({"type": "log", "level": "warn", "msg": "请先点击「获取控制权」"}))
            return
        self.control_until = now + CONTROL_IDLE_S
        try:
            await self.do(op, m)
        except ValueError as e:
            c.offer("text", json.dumps({"type": "log", "level": "error", "msg": str(e)}, ensure_ascii=False))
        self.broadcast_meta()

    def cam(self, serial):
        for x in self.cams:
            if x.serial == serial:
                return x
        raise ValueError(f"没有相机 {serial}")

    async def do(self, op, m):
        if op == "list":
            await self.send("list")
        elif op == "open":
            cam = self.cam(str(m.get("serial")))
            self.selected = cam.serial
            for k, v in self.params.items():
                await self.send(f"set {k} {v}")
            await self.send(f"open {cam.serial}")
        elif op in ("close", "start", "stop", "trigger", "reset", "record_stop"):
            await self.send(op)
        elif op == "set":
            k, v = self.check_param(str(m.get("key")), str(m.get("value")))
            self.params[k] = v
            await self.send(f"set {k} {v}")
        elif op == "record":
            secs = float(m.get("secs", 5))
            if not 1 <= secs <= REC_MAX_S:
                raise ValueError(f"录像时长 1～{REC_MAX_S} 秒")
            d = REC_ROOT / time.strftime("rec_%Y%m%d_%H%M%S")
            self.recordings.append(d)
            while len(self.recordings) > REC_KEEP:
                shutil.rmtree(self.recordings.pop(0), ignore_errors=True)
            await self.send(f"record {secs:g} {d}")
        elif op == "cable":
            cam = self.cam(str(m.get("serial")))
            cam.plugged = bool(m.get("plugged"))
            await cam.apply()
            self.log("info", f"{cam.serial} 网线{'已插回' if cam.plugged else '已拔掉'}")
        elif op == "loss":
            cam = self.cam(str(m.get("serial")))
            loss = int(m.get("permille", 0))
            if not 0 <= loss <= 100:
                raise ValueError("丢包率 0～100‰")
            cam.loss = loss
            if cam.plugged:
                await cam.start()   # 虚拟相机只能在启动时设丢包率，重启后 capd 自动重连
            self.log("info", f"{cam.serial} 丢包率设为 {loss}‰（虚拟相机重启，采集端自动重连）")
        elif op == "defaults":
            await self.apply_defaults()
        else:
            raise ValueError(f"未知操作 {op}")

    @staticmethod
    def check_param(k, v):
        nums = {"exposure": (10, 1_000_000), "fps": (1, 120), "auto_trigger": (0, 10_000), "packet_size": (576, 9000),
                "buffers": (1, 64), "process_ms": (0, 500), "slow_disk_us": (0, 200_000)}
        if k in nums:
            try:
                x = float(v)
            except ValueError:
                raise ValueError(f"{k} 需要数字")
            lo, hi = nums[k]
            if not lo <= x <= hi:
                raise ValueError(f"{k} 范围 {lo}～{hi}")
            return k, f"{x:g}"
        if k == "size":
            mm = re.fullmatch(r"(\d{2,4})x(\d{2,4})", v)
            if not mm or not (16 <= int(mm[1]) <= 2048 and 16 <= int(mm[2]) <= 2048):
                raise ValueError("分辨率最大 2048x2048")
            return k, v
        if k == "pixfmt":
            if v not in ("Mono8", "Mono16", "RGB8", "BayerRG8", "BayerGR8", "BayerGB8", "BayerBG8"):
                raise ValueError("不支持的像素格式")
            return k, v
        if k == "trigger":
            if v not in ("none", "soft"):
                raise ValueError("触发方式只能是 none 或 soft")
            return k, v
        raise ValueError(f"不能修改参数 {k}")


demo = Demo()
app = FastAPI(title="ai-cam")


@app.on_event("startup")
async def _startup():
    await demo.startup()


@app.on_event("shutdown")
async def _shutdown():
    await demo.shutdown()


@app.get("/api/health")
async def health():
    s = demo.last.get("stats") or {}
    ok = bool(demo.capd and demo.capd.returncode is None)
    return JSONResponse({"ok": ok, "state": s.get("state"), "fps": s.get("fps"), "viewers": len(demo.clients)},
                        status_code=200 if ok else 503)


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    c = await demo.join(ws)
    try:
        while True:
            try:
                m = json.loads(await ws.receive_text())
            except ValueError:
                continue
            if isinstance(m, dict):
                await demo.handle(c, m)
    except WebSocketDisconnect:
        pass
    finally:
        demo.leave(c)


@app.get("/")
async def index():
    return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-cache"})


app.mount("/static", StaticFiles(directory=STATIC), name="static")

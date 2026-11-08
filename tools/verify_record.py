#!/usr/bin/env python3
"""校验录制文件：逐帧检查头部 magic、CRC32、帧号连续性，统计帧间隔和传输延迟。

用法: python3 verify_record.py rec_0000.bin [rec_0001.bin ...]
      python3 verify_record.py --dump 100 rec_0000.bin   # 导出帧号 100：Mono8 存 PGM，RGB8 存 PPM
"""
import struct
import sys
import zlib

HDR = struct.Struct("<4sQQQIIIII")   # 与 recorder.h 中 FrameRecordHeader 一致
assert HDR.size == 48


def frame_id_missing(prev, cur):
    """prev 之后到 cur 之间缺了几帧，处理帧号绕回（与 frame.h 中的 C++ 版本规则一致）。
    返回 0 连续（含正常绕回），>0 缺失帧数，-1 帧号倒退但不像绕回（如相机重启后重新计数）。
    16 位（GigE Vision 1.x block ID）绕回后从 1 开始；32 位绕回后从 0 开始。"""
    if cur > prev:
        return cur - prev - 1
    tol = 1000
    if prev <= 0xFFFF and prev + tol >= 0xFFFF and cur <= tol:
        return (0xFFFF - prev) + (cur - 1 if cur >= 1 else 0)
    if prev <= 0xFFFFFFFF and prev + tol >= 0xFFFFFFFF and cur <= tol:
        return (0xFFFFFFFF - prev) + cur
    return -1


def iter_frames(path):
    with open(path, "rb") as f:
        while True:
            raw = f.read(HDR.size)
            if not raw:
                return
            if len(raw) < HDR.size:
                yield ("truncated_header", None, None)
                return
            magic, fid, dts, hts, w, h, pix, size, crc = HDR.unpack(raw)
            data = f.read(size)
            if magic != b"FRM1":
                yield ("bad_magic", None, None)
                return
            if len(data) < size:
                yield ("truncated_data", None, None)
                return
            hdr = dict(id=fid, dev_ts=dts, host_ts=hts, w=w, h=h, pix=pix, size=size, crc=crc)
            yield ("ok", hdr, data)


def main(argv):
    dump_id = None
    if len(argv) > 2 and argv[1] == "--dump":
        dump_id = int(argv[2])
        argv = argv[2:]
    files = argv[1:]
    if not files:
        print(__doc__)
        return 1

    total = crc_err = gaps = missing = 0
    last_id = last_dts = last_hts = None
    intervals, host_intervals, latency = [], [], []

    for path in files:
        for status, hdr, data in iter_frames(path):
            if status != "ok":
                print(f"[{path}] {status}")
                continue
            total += 1
            if zlib.crc32(data) & 0xFFFFFFFF != hdr["crc"]:
                crc_err += 1
            if last_id is not None:
                miss = frame_id_missing(last_id, hdr["id"])
                if miss != 0:
                    gaps += 1
                if miss > 0:
                    missing += miss
            if last_dts is not None and hdr["dev_ts"] > last_dts:
                intervals.append((hdr["dev_ts"] - last_dts) / 1e6)
            if last_hts is not None and hdr["host_ts"] > last_hts:
                host_intervals.append((hdr["host_ts"] - last_hts) / 1e6)
            # 相机时间戳为 0 表示设备不提供（如 MVS 虚拟相机），算延迟没有意义
            if hdr["dev_ts"] and hdr["host_ts"] >= hdr["dev_ts"]:
                latency.append((hdr["host_ts"] - hdr["dev_ts"]) / 1e6)
            last_id, last_dts, last_hts = hdr["id"], hdr["dev_ts"], hdr["host_ts"]

            if dump_id is not None and hdr["id"] == dump_id:
                # Mono8 存 PGM，RGB8 存 PPM，常见看图软件可打开
                rgb = hdr["pix"] == 0x02180014
                out = f"frame_{dump_id}.{'ppm' if rgb else 'pgm'}"
                with open(out, "wb") as o:
                    o.write(f"{'P6' if rgb else 'P5'} {hdr['w']} {hdr['h']} 255\n".encode())
                    o.write(data[: hdr["w"] * hdr["h"] * (3 if rgb else 1)])
                print(f"dumped {out}")

    print(f"frames={total} crc_errors={crc_err} id_gaps={gaps} missing~{missing}")
    # 优先用相机时间戳算帧间隔；相机不给时间戳时退回主机接收时间
    label = "frame interval ms"
    if not intervals and host_intervals:
        intervals, label = host_intervals, "frame interval ms (host time, camera has no timestamp)"
    if intervals:
        intervals.sort()
        p = lambda q: intervals[min(len(intervals) - 1, int(q * len(intervals)))]
        print(f"{label}: min={intervals[0]:.2f} p50={p(0.5):.2f} "
              f"p99={p(0.99):.2f} max={intervals[-1]:.2f}")
    if latency:
        latency.sort()
        print(f"dev->host latency ms (同一时钟才有意义): p50={latency[len(latency)//2]:.3f} "
              f"max={latency[-1]:.3f}")
    return 0 if crc_err == 0 else 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))

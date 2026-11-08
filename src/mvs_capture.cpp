// 海康机器人 MVS SDK 版采集（MV-CA / MV-CS / MV-CU 等系列工业相机）
// 安装: 海康机器人官网下载 MVS Linux 版，默认装到 /opt/MVS
//       头文件 /opt/MVS/include，库 /opt/MVS/lib/64 (x86_64) 或 aarch64
// 运行前: export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
//
// 用法: ./mvs_capture [秒数=10] [触发 none|soft|line0] [曝光us=5000] [像素格式=Mono8] [相机]
//   像素格式用 GenICam 名称，如 Mono8 / RGB8Packed / BayerRG8（彩色相机）
//   相机: 序列号或型号的一部分，如 Vir4050648 / 10GC；不给则用枚举到的第一台
// 无相机测试: 用 /opt/MVS/bin/VirtualDeviceTool.sh 创建一台虚拟相机后运行本程序
//
// 本示例按 MVS SDK 常用接口编写，未在真机上编译验证；
// 不同 SDK 版本结构体字段可能略有差异（如 nFrameLen / nFrameLenEx），以随 SDK 附带的 Samples 为准。
#include "MvCameraControl.h"

#include <atomic>
#include <cinttypes>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "frame.h"
#include "frame_queue.h"
#include "recorder.h"

static std::atomic<bool> g_stop{false};
static std::atomic<bool> g_lost{false};

static void on_sigint(int) { g_stop = true; }

// 异常回调：掉线时 SDK 调用（在 SDK 内部线程，只置标志，不做重活）
static void __stdcall on_exception(unsigned int msg_type, void*) {
    if (msg_type == MV_EXCEPTION_DEV_DISCONNECT) g_lost = true;
}

#define CHECK(expr)                                                                   \
    do {                                                                              \
        int _r = (expr);                                                              \
        if (_r != MV_OK) std::fprintf(stderr, "[err] %s = 0x%08x\n", #expr, _r);       \
    } while (0)

int main(int argc, char** argv) {
    int secs = argc > 1 ? std::atoi(argv[1]) : 10;
    std::string trig = argc > 2 ? argv[2] : "none";
    float exposure_us = argc > 3 ? float(std::atof(argv[3])) : 5000.f;
    std::string pixfmt = argc > 4 ? argv[4] : "Mono8";
    std::string select = argc > 5 ? argv[5] : "";
    std::signal(SIGINT, on_sigint);

    // SDK 4.x 起要求先初始化，退出前 MV_CC_Finalize
    if (MV_CC_Initialize() != MV_OK) {
        std::fprintf(stderr, "MV_CC_Initialize failed\n");
        return 1;
    }

    // ---------- 1. 枚举并打开设备 ----------
    // 真实 GigE/USB3 相机 + MVS 自带 VirtualDeviceTool 创建的虚拟相机（无硬件时测试用）
    MV_CC_DEVICE_INFO_LIST list;
    std::memset(&list, 0, sizeof(list));
    const unsigned int types =
        MV_GIGE_DEVICE | MV_USB_DEVICE | MV_VIR_GIGE_DEVICE | MV_VIR_USB_DEVICE;
    if (MV_CC_EnumDevices(types, &list) != MV_OK || list.nDeviceNum == 0) {
        std::fprintf(stderr, "no camera found\n");
        MV_CC_Finalize();
        return 1;
    }
    // 列出所有相机；按第 5 个参数（序列号或型号的一部分）选择，不给就用第一台
    auto model_of = [](MV_CC_DEVICE_INFO* d) {
        bool gige = d->nTLayerType == MV_GIGE_DEVICE || d->nTLayerType == MV_VIR_GIGE_DEVICE;
        return std::string(gige ? (const char*)d->SpecialInfo.stGigEInfo.chModelName
                                : (const char*)d->SpecialInfo.stUsb3VInfo.chModelName);
    };
    auto serial_of = [](MV_CC_DEVICE_INFO* d) {
        bool gige = d->nTLayerType == MV_GIGE_DEVICE || d->nTLayerType == MV_VIR_GIGE_DEVICE;
        return std::string(gige ? (const char*)d->SpecialInfo.stGigEInfo.chSerialNumber
                                : (const char*)d->SpecialInfo.stUsb3VInfo.chSerialNumber);
    };
    std::printf("found %u camera(s)\n", list.nDeviceNum);
    MV_CC_DEVICE_INFO* di = nullptr;
    for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO* d = list.pDeviceInfo[i];
        bool match = select.empty() ? i == 0
                                    : serial_of(d).find(select) != std::string::npos ||
                                          model_of(d).find(select) != std::string::npos;
        if (match && !di) di = d;
        std::printf("  [%u] %s (%s)%s\n", i, model_of(d).c_str(), serial_of(d).c_str(),
                    d == di ? "  <- selected" : "");
    }
    if (!di) {
        std::fprintf(stderr, "no camera matches '%s'\n", select.c_str());
        MV_CC_Finalize();
        return 1;
    }

    void* handle = nullptr;
    if (MV_CC_CreateHandle(&handle, di) != MV_OK) {
        MV_CC_Finalize();
        return 1;
    }
    if (MV_CC_OpenDevice(handle) != MV_OK) {
        std::fprintf(stderr, "open failed (被其他程序占用? IP 不在同网段?)\n");
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return 1;
    }
    CHECK(MV_CC_RegisterExceptionCallBack(handle, on_exception, nullptr));

    // ---------- 2. 配置 ----------
    if (di->nTLayerType == MV_GIGE_DEVICE) {  // 虚拟相机不走网络，不需要协商包大小
        int pkt = MV_CC_GetOptimalPacketSize(handle);   // 需开巨帧才能拿到大包
        if (pkt > 0) CHECK(MV_CC_SetIntValue(handle, "GevSCPSPacketSize", pkt));
    }
    CHECK(MV_CC_SetEnumValueByString(handle, "PixelFormat", pixfmt.c_str()));
    CHECK(MV_CC_SetEnumValue(handle, "ExposureAuto", 0));     // 关自动曝光
    CHECK(MV_CC_SetFloatValue(handle, "ExposureTime", exposure_us));
    CHECK(MV_CC_SetEnumValue(handle, "GainAuto", 0));

    if (trig == "none") {
        CHECK(MV_CC_SetEnumValue(handle, "TriggerMode", 0));  // Off：连续采集
        CHECK(MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true));
        CHECK(MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", 30.f));
    } else {
        CHECK(MV_CC_SetEnumValue(handle, "TriggerMode", 1));  // On
        CHECK(MV_CC_SetEnumValueByString(handle, "TriggerSource",
                                         trig == "soft" ? "Software" : "Line0"));
        if (trig != "soft")
            CHECK(MV_CC_SetEnumValueByString(handle, "TriggerActivation", "RisingEdge"));
    }
    // SDK 内部缓存节点数
    CHECK(MV_CC_SetImageNodeNum(handle, 16));

    MVCC_INTVALUE payload{};
    CHECK(MV_CC_GetIntValue(handle, "PayloadSize", &payload));
    std::printf("payload %u bytes\n", payload.nCurValue);

    // ---------- 3. 下游 ----------
    const size_t QCAP = 64;
    auto pool = FramePool::create(QCAP + 8, payload.nCurValue);
    BoundedQueue<FramePtr> q(QCAP);
    Recorder rec(q, ".", 512ull << 20);
    rec.start();

    CHECK(MV_CC_StartGrabbing(handle));

    std::thread trig_th;
    if (trig == "soft") {
        trig_th = std::thread([&] {
            while (!g_stop && !g_lost) {
                MV_CC_SetCommandValue(handle, "TriggerSoftware");
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
    }

    // ---------- 4. 主动取帧（也可用 MV_CC_RegisterImageCallBackEx 回调方式） ----------
    uint64_t ok = 0, lost_pkt_frames = 0, dropped = 0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
    while (!g_stop && !g_lost && std::chrono::steady_clock::now() < deadline) {
        MV_FRAME_OUT out;
        std::memset(&out, 0, sizeof(out));
        int r = MV_CC_GetImageBuffer(handle, &out, 1000);   // 1s 超时(ms)
        if (r != MV_OK) continue;                           // 超时

        const MV_FRAME_OUT_INFO_EX& info = out.stFrameInfo;
        if (info.nLostPacket > 0) lost_pkt_frames++;         // 网络丢包导致的残帧

        FramePtr f = pool->acquire();
        if (f) {
            // SDK 4.x 用 64 位 nFrameLenEx（旧字段 nFrameLen 只到 4GB），老版本 SDK 这里为 0
            size_t n = info.nFrameLenEx ? size_t(info.nFrameLenEx) : info.nFrameLen;
            if (n > f->data.size()) n = f->data.size();
            std::memcpy(f->data.data(), out.pBufAddr, n);
            f->frame_id = info.nFrameNum;
            // 设备时间戳是高低 32 位拆开的，单位依型号而定（常见为 ns 或 tick），需查手册换算
            f->dev_ts_ns = (uint64_t(info.nDevTimeStampHigh) << 32) | info.nDevTimeStampLow;
            f->host_ts_ns = host_now_ns();
            f->width = info.nWidth;
            f->height = info.nHeight;
            f->pixfmt = uint32_t(info.enPixelType);
            f->size = uint32_t(n);
            if (!q.try_push(std::move(f))) dropped++;
        } else {
            dropped++;
        }
        ok++;
        MV_CC_FreeImageBuffer(handle, &out);   // 必须归还，否则 SDK 缓存耗尽
    }

    if (g_lost) std::fprintf(stderr, "[warn] device disconnected, need reconnect\n");

    // ---------- 5. 收尾 ----------
    g_stop = true;
    if (trig_th.joinable()) trig_th.join();
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();

    q.close();
    rec.join();
    std::printf("ok=%" PRIu64 " lostPacketFrames=%" PRIu64 " dropped=%" PRIu64
                " written=%" PRIu64 " gaps=%" PRIu64 "\n",
                ok, lost_pkt_frames, dropped, rec.stats().written.load(),
                rec.stats().gaps.load());
    return 0;
}

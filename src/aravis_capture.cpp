// Aravis 版采集：开源 GenICam 库，支持绝大多数 GigE Vision / USB3 Vision 相机
// 安装: sudo apt install libaravis-dev aravis-tools   (Ubuntu 22.04+ 为 0.8 版本)
// 调试: arv-tool-0.8 列出相机; arv-viewer-0.8 图形界面看图; arv-camera-test-0.8 测试取流
// 无相机测试: 另开终端运行 arv-fake-gv-camera-0.8 -i 127.0.0.1（虚拟 GigE 相机），再运行本程序
// 编译: 见 CMakeLists.txt，或
//   g++ -std=c++17 -O2 -pthread -Iinclude src/aravis_capture.cpp -o aravis_capture
//       $(pkg-config --cflags --libs aravis-0.8)      (两行写在同一行)
//
// 用法: ./aravis_capture [秒数=10] [触发 none|soft|line1] [曝光us=5000] [相机]
//       ./aravis_capture list            只列出所有相机
//   相机: 序列号/型号/IP/设备ID 的一部分，如 GV02 / 192.168.1.10；不给则用枚举到的第一台
//
// 本示例未在真机上编译验证，不同 Aravis 版本 / 相机型号的特性名可能有差异，
// 以 arv-tool-0.8 features 输出和相机手册为准。
#include <arv.h>

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

// GigE 控制通道丢失（拔网线、相机掉电）时 Aravis 发出的信号
static void on_control_lost(ArvGvDevice*, gpointer) { g_lost = true; }

static bool check(GError*& err, const char* what) {
    if (!err) return true;
    std::fprintf(stderr, "[err] %s: %s\n", what, err->message);
    g_clear_error(&err);
    return false;
}

// 枚举所有相机并打印；select 非空时选第一台 序列号/型号/地址/设备ID 包含它的，否则选第 0 台。
// 返回选中相机的设备 ID（传给 arv_camera_new），没有匹配返回空串。
static std::string enumerate_cameras(const std::string& select) {
    arv_update_device_list();   // GigE：在各网卡上广播发现请求；USB3：扫描 USB 总线
    unsigned int n = arv_get_n_devices();
    std::printf("found %u camera(s)\n", n);
    std::string chosen;
    auto has = [&](const char* s) { return s && std::strstr(s, select.c_str()); };
    for (unsigned int i = 0; i < n; ++i) {
        const char* id = arv_get_device_id(i);
        bool match = select.empty() ? i == 0
                                    : has(arv_get_device_serial_nbr(i)) ||
                                          has(arv_get_device_model(i)) ||
                                          has(arv_get_device_address(i)) || has(id);
        bool pick = match && chosen.empty();
        if (pick) chosen = id ? id : "";
        std::printf("  [%u] %s %s  serial=%s  addr=%s  %s%s\n", i, arv_get_device_vendor(i),
                    arv_get_device_model(i), arv_get_device_serial_nbr(i),
                    arv_get_device_address(i), arv_get_device_protocol(i),
                    pick ? "  <- selected" : "");
    }
    return chosen;
}

int main(int argc, char** argv) {
    // ./aravis_capture list：只列出相机就退出
    if (argc > 1 && std::strcmp(argv[1], "list") == 0) {
        enumerate_cameras("");
        arv_shutdown();
        return 0;
    }
    int secs = argc > 1 ? std::atoi(argv[1]) : 10;
    std::string trig = argc > 2 ? argv[2] : "none";
    double exposure_us = argc > 3 ? std::atof(argv[3]) : 5000;
    std::string select = argc > 4 ? argv[4] : "";
    std::signal(SIGINT, on_sigint);

    GError* err = nullptr;

    // ---------- 1. 枚举并打开相机 ----------
    std::string dev_id = enumerate_cameras(select);
    if (dev_id.empty()) {
        std::fprintf(stderr, select.empty() ? "no camera found\n" : "no camera matches '%s'\n",
                     select.c_str());
        arv_shutdown();
        return 1;
    }
    ArvCamera* cam = arv_camera_new(dev_id.c_str(), &err);
    if (!check(err, "open camera") || !cam) return 1;
    std::printf("camera: %s %s (%s)\n", arv_camera_get_vendor_name(cam, nullptr),
                arv_camera_get_model_name(cam, nullptr),
                arv_camera_get_device_serial_number(cam, nullptr));

    ArvDevice* dev = arv_camera_get_device(cam);
    if (ARV_IS_GV_DEVICE(dev)) {
        g_signal_connect(dev, "control-lost", G_CALLBACK(on_control_lost), nullptr);

        // Aravis 默认用 packet socket 直接抓网卡原始包（性能好），但在回环地址上收不到完整帧：
        // 相机在 127.0.0.1（arv-fake-gv-camera 虚拟相机）时改用普通 UDP socket
        GSocketAddress* sa = arv_gv_device_get_device_address(ARV_GV_DEVICE(dev));
        if (sa && G_IS_INET_SOCKET_ADDRESS(sa) &&
            g_inet_address_get_is_loopback(
                g_inet_socket_address_get_address(G_INET_SOCKET_ADDRESS(sa)))) {
            arv_gv_device_set_stream_options(ARV_GV_DEVICE(dev),
                                             ARV_GV_STREAM_OPTION_PACKET_SOCKET_DISABLED);
            std::printf("loopback camera: packet socket disabled\n");
        }
    }

    // ---------- 2. 配置参数（GenICam 标准特性名） ----------
    if (arv_camera_is_gv_device(cam)) {
        // 自动协商最大包大小（需交换机/网卡开巨帧 MTU 9000 才能用到 8K 包）
        arv_camera_gv_auto_packet_size(cam, &err);
        check(err, "auto packet size");
    }
    arv_camera_set_pixel_format(cam, ARV_PIXEL_FORMAT_MONO_8, &err);
    check(err, "pixel format");
    arv_camera_set_exposure_time_auto(cam, ARV_AUTO_OFF, &err);
    check(err, "exposure auto off");
    arv_camera_set_exposure_time(cam, exposure_us, &err);
    check(err, "exposure");
    // ROI 示例：arv_camera_set_region(cam, x, y, w, h, &err);

    if (trig == "none") {
        arv_camera_clear_triggers(cam, &err);            // 自由运行
        check(err, "clear triggers");
        arv_camera_set_frame_rate(cam, 30.0, &err);
        check(err, "frame rate");
    } else if (trig == "soft") {
        arv_camera_set_trigger(cam, "Software", &err);   // 软件触发
        check(err, "software trigger");
    } else {
        // 硬件触发：等价于 TriggerSelector=FrameStart, TriggerMode=On, TriggerSource=Line1
        arv_camera_set_trigger(cam, "Line1", &err);
        check(err, "hw trigger");
        arv_camera_set_string(cam, "TriggerActivation", "RisingEdge", &err);
        check(err, "trigger activation");
    }

    // Chunk 数据：让相机把时间戳、帧号随图像一起送出来（相机支持才生效）
    arv_camera_set_chunks(cam, "Timestamp,FrameID", &err);
    bool chunk_ok = check(err, "enable chunks");
    ArvChunkParser* parser = chunk_ok ? arv_camera_create_chunk_parser(cam) : nullptr;

    // ---------- 3. 创建流并预压缓冲 ----------
    ArvStream* stream = arv_camera_create_stream(cam, nullptr, nullptr, &err);
    if (!check(err, "create stream") || !stream) return 1;
    // 让 Aravis 按帧大小自动调大 socket 接收缓冲，否则高速流下内核缓冲溢出会大量丢包
    if (ARV_IS_GV_STREAM(stream))
        g_object_set(stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, nullptr);
    guint payload = arv_camera_get_payload(cam, &err);
    check(err, "payload");
    const int NBUF = 32;   // SDK 侧缓冲数；太少容易 underrun
    for (int i = 0; i < NBUF; ++i) arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));

    gint x, y, w, h;
    arv_camera_get_region(cam, &x, &y, &w, &h, nullptr);
    std::printf("region %dx%d, payload %u bytes\n", w, h, payload);

    // ---------- 4. 下游：帧池 + 队列 + 写盘线程 ----------
    const size_t QCAP = 64;
    auto pool = FramePool::create(QCAP + 8, payload);
    BoundedQueue<FramePtr> q(QCAP);
    Recorder rec(q, ".", 512ull << 20);
    rec.start();

    // 软触发时用一个线程定时发触发命令（真实系统里通常由运动控制或外部信号驱动）
    std::thread trig_th;
    if (trig == "soft") {
        trig_th = std::thread([&] {
            while (!g_stop && !g_lost) {
                arv_camera_software_trigger(cam, nullptr);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
    }

    arv_camera_start_acquisition(cam, &err);
    if (!check(err, "start acquisition")) return 1;

    // ---------- 5. 取帧循环 ----------
    uint64_t ok = 0, bad = 0, dropped = 0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
    while (!g_stop && !g_lost && std::chrono::steady_clock::now() < deadline) {
        ArvBuffer* buf = arv_stream_timeout_pop_buffer(stream, 1000000);  // 1s 超时(us)
        if (!buf) continue;  // 超时：触发模式下没触发信号时属正常

        if (arv_buffer_get_status(buf) == ARV_BUFFER_STATUS_SUCCESS) {
            size_t n = 0;
            const void* data = arv_buffer_get_data(buf, &n);
            FramePtr f = pool->acquire();
            if (f) {
                if (n > f->data.size()) n = f->data.size();
                std::memcpy(f->data.data(), data, n);
                f->frame_id = arv_buffer_get_frame_id(buf);
                f->dev_ts_ns = arv_buffer_get_timestamp(buf);          // 相机时间戳
                f->host_ts_ns = arv_buffer_get_system_timestamp(buf);  // 主机接收时间
                if (parser) {  // 优先用 chunk 里的值
                    GError* e2 = nullptr;
                    int64_t ts = arv_chunk_parser_get_integer_value(parser, buf, "ChunkTimestamp", &e2);
                    if (!e2) f->dev_ts_ns = uint64_t(ts); else g_clear_error(&e2);
                }
                f->width = arv_buffer_get_image_width(buf);
                f->height = arv_buffer_get_image_height(buf);
                f->pixfmt = arv_buffer_get_image_pixel_format(buf);
                f->size = uint32_t(n);
                if (!q.try_push(std::move(f))) dropped++;
            } else {
                dropped++;
            }
            ok++;
        } else {
            bad++;   // MISSING_PACKETS / TIMEOUT 等：网络或带宽问题
        }
        arv_stream_push_buffer(stream, buf);   // 缓冲还给 SDK，必须做，否则很快耗尽
    }

    if (g_lost) std::fprintf(stderr, "[warn] control lost, need reconnect\n");
    // 真实项目：这里进入 Error 状态，释放资源后按 sim_capture 中的状态机重连

    // ---------- 6. 收尾 ----------
    g_stop = true;
    if (trig_th.joinable()) trig_th.join();
    arv_camera_stop_acquisition(cam, nullptr);

    guint64 completed = 0, failures = 0, underruns = 0;
    arv_stream_get_statistics(stream, &completed, &failures, &underruns);

    q.close();
    rec.join();
    std::printf("ok=%" PRIu64 " bad=%" PRIu64 " dropped=%" PRIu64 " written=%" PRIu64
                " gaps=%" PRIu64 " | sdk completed=%" G_GUINT64_FORMAT
                " failures=%" G_GUINT64_FORMAT " underruns=%" G_GUINT64_FORMAT "\n",
                ok, bad, dropped, rec.stats().written.load(), rec.stats().gaps.load(),
                completed, failures, underruns);

    if (parser) g_object_unref(parser);
    g_object_unref(stream);
    g_object_unref(cam);
    arv_shutdown();   // 释放 Aravis 全局资源（设备列表等）
    return 0;
}

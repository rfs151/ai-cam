// capd：常驻采集引擎，给网页演示用（由 web/api 启动并通过管道控制）
//
//   stdin  每行一条文本命令          stdout 每行一个 JSON 消息
//   ─────────────────────────────    ─────────────────────────────────────────────
//   list                             {"type":"cameras","items":[...]}
//   open <序列号> / close             {"type":"state","state":"Connecting",...}
//   start / stop                     {"type":"info",...}    打开后：相机型号、可选格式、参数范围
//   trigger                          {"type":"stats",...}   每 200ms：各层计数
//   set <参数> <值>                   {"type":"preview",...} 每 100ms：缩小后的 JPEG（base64）
//   record <秒数> <目录> / record_stop {"type":"record",...}  录像开始 / 结束
//   reset / quit                     {"type":"log",...}
//
//   set 参数：exposure(us) fps size(宽x高) width height pixfmt trigger(none|soft) auto_trigger(ms，0 关)
//            packet_size buffers(SDK 缓冲块数) process_ms(取帧线程每帧额外耗时) slow_disk_us
//
// 线程：主线程只读 stdin；引擎线程负责相机控制 + 取帧 + 统计（Aravis 对象只在这个线程里用）；
//       录像时另有写盘线程（recorder.h）。断线后按 100ms → 5s 指数退避自动重连，并恢复采集。
#include <arv.h>

#include <cstdio>   // jpeglib.h 用到 FILE，要先引入
#include <jpeglib.h>

#include <atomic>
#include <cinttypes>
#include <cmath>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "frame.h"
#include "frame_queue.h"
#include "recorder.h"

using namespace std::chrono;
using Clock = steady_clock;

// ---------------------------------------------------------------------------
// 输出：每行一个 JSON，多线程共用一把锁
// ---------------------------------------------------------------------------
static std::mutex g_out_mtx;
static void emit(const std::string& json) {
    std::lock_guard<std::mutex> lk(g_out_mtx);
    std::fwrite(json.data(), 1, json.size(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
}

static std::string jstr(const std::string& s) {
    std::string o = "\"";
    for (unsigned char c : s) {
        switch (c) {
            case '"': o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n"; break;
            case '\r': o += "\\r"; break;
            case '\t': o += "\\t"; break;
            default:
                if (c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); o += b; }
                else o += char(c);
        }
    }
    return o + "\"";
}
static std::string jstr(const char* s) { return jstr(std::string(s ? s : "")); }

static std::string fmt(const char* f, ...) {
    char buf[4096];
    va_list ap;
    va_start(ap, f);
    int n = std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    if (n < 0) return {};
    if (size_t(n) < sizeof buf) return std::string(buf, size_t(n));
    std::string big(size_t(n) + 1, '\0');   // 超长时再格式化一次
    va_start(ap, f);
    std::vsnprintf(big.data(), big.size(), f, ap);
    va_end(ap);
    big.resize(size_t(n));
    return big;
}

static void log_msg(const char* level, const std::string& msg) {
    emit("{\"type\":\"log\",\"level\":" + jstr(level) + ",\"msg\":" + jstr(msg) + "}");
}

// ---------------------------------------------------------------------------
// 预览：缩小 + JPEG 编码 + base64
// ---------------------------------------------------------------------------
static std::string base64(const unsigned char* p, size_t n) {
    static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string o;
    o.reserve((n + 2) / 3 * 4);
    size_t i = 0;
    for (; i + 2 < n; i += 3) {
        uint32_t v = (p[i] << 16) | (p[i + 1] << 8) | p[i + 2];
        o += T[v >> 18]; o += T[(v >> 12) & 63]; o += T[(v >> 6) & 63]; o += T[v & 63];
    }
    if (i < n) {
        uint32_t v = p[i] << 16;
        if (i + 1 < n) v |= p[i + 1] << 8;
        o += T[v >> 18]; o += T[(v >> 12) & 63];
        o += (i + 1 < n) ? T[(v >> 6) & 63] : '=';
        o += '=';
    }
    return o;
}

// 把一帧缩小到最长边 <= max_side，转成 8 位灰度或 RGB，编码为 JPEG
static bool make_preview(const uint8_t* src, size_t n, int w, int h, ArvPixelFormat pf,
                         int max_side, std::vector<unsigned char>& jpeg, int& ow, int& oh) {
    if (w <= 0 || h <= 0) return false;
    int step = std::max(1, int(std::ceil(double(std::max(w, h)) / max_side)));
    ow = w / step;
    oh = h / step;
    bool rgb = pf == ARV_PIXEL_FORMAT_RGB_8_PACKED;
    int bpp = ARV_PIXEL_FORMAT_BIT_PER_PIXEL(pf);
    int bytes_pp = rgb ? 3 : (bpp > 8 ? 2 : 1);
    if (size_t(w) * h * bytes_pp > n) return false;
    int shift = pf == ARV_PIXEL_FORMAT_MONO_10 ? 2 : pf == ARV_PIXEL_FORMAT_MONO_12 ? 4 : 8;

    std::vector<unsigned char> img(size_t(ow) * oh * (rgb ? 3 : 1));
    for (int y = 0; y < oh; ++y) {
        const uint8_t* row = src + size_t(y) * step * w * bytes_pp;
        for (int x = 0; x < ow; ++x) {
            const uint8_t* px = row + size_t(x) * step * bytes_pp;
            if (rgb) {
                unsigned char* d = &img[(size_t(y) * ow + x) * 3];
                d[0] = px[0]; d[1] = px[1]; d[2] = px[2];
            } else if (bytes_pp == 2) {
                uint16_t v = uint16_t(px[0] | (px[1] << 8));   // 小端
                img[size_t(y) * ow + x] = uint8_t(std::min(255, v >> shift));
            } else {
                img[size_t(y) * ow + x] = px[0];               // Mono8 / Bayer8 原样当灰度
            }
        }
    }

    jpeg_compress_struct c{};
    jpeg_error_mgr err{};
    c.err = jpeg_std_error(&err);
    jpeg_create_compress(&c);
    unsigned char* out = nullptr;
    unsigned long out_n = 0;
    jpeg_mem_dest(&c, &out, &out_n);
    c.image_width = ow;
    c.image_height = oh;
    c.input_components = rgb ? 3 : 1;
    c.in_color_space = rgb ? JCS_RGB : JCS_GRAYSCALE;
    jpeg_set_defaults(&c);
    jpeg_set_quality(&c, 75, TRUE);
    jpeg_start_compress(&c, TRUE);
    while (c.next_scanline < c.image_height) {
        JSAMPROW r = &img[size_t(c.next_scanline) * ow * (rgb ? 3 : 1)];
        jpeg_write_scanlines(&c, &r, 1);
    }
    jpeg_finish_compress(&c);
    jpeg.assign(out, out + out_n);
    jpeg_destroy_compress(&c);
    std::free(out);
    return true;
}

// ---------------------------------------------------------------------------
// 命令队列：主线程读 stdin 放进来，引擎线程取
// ---------------------------------------------------------------------------
class CommandQueue {
public:
    void push(std::string s) {
        std::lock_guard<std::mutex> lk(m_);
        q_.push_back(std::move(s));
    }
    bool pop(std::string& s) {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.empty()) return false;
        s = std::move(q_.front());
        q_.pop_front();
        return true;
    }
private:
    std::mutex m_;
    std::deque<std::string> q_;
};

// ---------------------------------------------------------------------------
// 引擎
// ---------------------------------------------------------------------------
enum class St { Idle, Connecting, Ready, Acquiring, Error };
static const char* st_name(St s) {
    switch (s) {
        case St::Idle: return "Idle";
        case St::Connecting: return "Connecting";
        case St::Ready: return "Ready";
        case St::Acquiring: return "Acquiring";
        case St::Error: return "Error";
    }
    return "?";
}

// GigE 相机的 IP（Aravis 返回的是 IP + 端口的套接字地址）
static GInetAddress* device_ip(ArvDevice* dev) {
    GSocketAddress* sa = arv_gv_device_get_device_address(ARV_GV_DEVICE(dev));
    return sa ? g_inet_socket_address_get_address(G_INET_SOCKET_ADDRESS(sa)) : nullptr;
}

static std::atomic<bool> g_control_lost{false};
static void on_control_lost(ArvGvDevice*) { g_control_lost = true; }   // 只设标志

struct Params {
    double exposure_us = 5000;
    double fps = 30;
    int width = 0, height = 0;          // 0 = 保持相机当前值
    std::string pixfmt;                 // 空 = 保持相机当前值
    std::string trigger = "none";       // none | soft
    int auto_trigger_ms = 0;
    int packet_size = 1400;
    int buffers = 16;
    int process_ms = 0;                 // 故障注入：取帧线程每帧额外耗时
    int slow_disk_us = 0;               // 故障注入：写盘每帧额外耗时
};

// 累计计数。SDK 的计数跟着 stream 走，重连后 stream 重建会清零，所以先加到 base 里
struct Counters {
    uint64_t ok = 0, bad = 0, link_gaps = 0, link_missing = 0, bytes = 0;
    std::map<std::string, uint64_t> bad_by;
    uint64_t sdk_completed = 0, sdk_failures = 0, sdk_underruns = 0;
    uint64_t pkt_received = 0, pkt_missing = 0, pkt_resend_req = 0, pkt_resent = 0, pkt_ignored = 0;
};

class Engine {
public:
    explicit Engine(CommandQueue& cq) : cq_(cq) {}

    void run() {
        auto next_stats = Clock::now();
        while (!quit_) {
            std::string line;
            while (cq_.pop(line)) handle(line);
            if (quit_) break;
            tick();
            auto now = Clock::now();
            if (now >= next_stats) {
                emit_stats();
                next_stats = now + milliseconds(200);
            }
        }
        record_stop(true);
        teardown();
        if (rec_thread_.joinable()) rec_thread_.join();
    }

private:
    // ---------------- 命令 ----------------
    void handle(const std::string& line) {
        std::istringstream is(line);
        std::string cmd;
        is >> cmd;
        if (cmd.empty()) return;
        if (cmd == "quit") { quit_ = true; return; }
        if (cmd == "list") { list_cameras(); return; }
        if (cmd == "open") {
            std::string serial;
            is >> serial;
            if (serial.empty()) { log_msg("error", "open 需要序列号"); return; }
            teardown();
            want_serial_ = serial;
            want_acquire_ = false;
            backoff_ms_ = 100;
            reset_counters();
            set_state(St::Connecting);
            next_try_ = Clock::now();
            return;
        }
        if (cmd == "close") {
            record_stop(false);
            teardown();
            want_serial_.clear();
            want_acquire_ = false;
            set_state(St::Idle);
            return;
        }
        if (cmd == "start") {
            want_acquire_ = true;
            if (state_ == St::Ready) start_acquisition();
            return;
        }
        if (cmd == "stop") {
            want_acquire_ = false;
            record_stop(false);
            if (state_ == St::Acquiring) stop_acquisition();
            return;
        }
        if (cmd == "trigger") { soft_trigger(); return; }
        if (cmd == "reset") { reset_counters(); return; }
        if (cmd == "set") {
            std::string k, v;
            is >> k >> v;
            set_param(k, v);
            return;
        }
        if (cmd == "record") {
            double secs = 10;
            std::string dir;
            is >> secs >> dir;
            record_start(secs, dir);
            return;
        }
        if (cmd == "record_stop") { record_stop(false); return; }
        log_msg("error", "未知命令: " + cmd);
    }

    void set_param(const std::string& k, const std::string& v) {
        auto num = [&](double lo, double hi, double& out) {
            char* end = nullptr;
            double x = std::strtod(v.c_str(), &end);
            if (v.empty() || *end || !(x >= lo && x <= hi)) {
                log_msg("error", fmt("参数 %s 的值 %s 超出范围 [%g, %g]", k.c_str(), v.c_str(), lo, hi));
                return false;
            }
            out = x;
            return true;
        };
        double x = 0;
        // 需要重建 stream 的参数（改变每帧字节数或缓冲块数）
        bool rebuild = false;
        if (k == "exposure") { if (!num(10, 1e6, x)) return; p_.exposure_us = x; apply_exposure(); }
        else if (k == "fps") { if (!num(1, 1000, x)) return; p_.fps = x; apply_trigger(); }
        else if (k == "width") { if (!num(16, 16384, x)) return; p_.width = int(x); rebuild = true; }
        else if (k == "height") { if (!num(16, 16384, x)) return; p_.height = int(x); rebuild = true; }
        else if (k == "size") {
            int w = 0, h = 0;
            if (std::sscanf(v.c_str(), "%dx%d", &w, &h) != 2 || w < 16 || h < 16 || w > 16384 || h > 16384) {
                log_msg("error", "size 格式是 宽x高，例如 1024x1024"); return;
            }
            p_.width = w; p_.height = h; rebuild = true;
        }
        else if (k == "pixfmt") { p_.pixfmt = v; rebuild = true; }
        else if (k == "trigger") {
            if (v != "none" && v != "soft") { log_msg("error", "trigger 只能是 none 或 soft"); return; }
            p_.trigger = v; apply_trigger();
        }
        else if (k == "auto_trigger") { if (!num(0, 60000, x)) return; p_.auto_trigger_ms = int(x); next_auto_ = Clock::now(); }
        else if (k == "packet_size") { if (!num(576, 9000, x)) return; p_.packet_size = int(x); rebuild = true; }
        else if (k == "buffers") { if (!num(1, 256, x)) return; p_.buffers = int(x); rebuild = true; }
        else if (k == "process_ms") { if (!num(0, 1000, x)) return; p_.process_ms = int(x); }
        else if (k == "slow_disk_us") {
            if (!num(0, 1e6, x)) return;
            p_.slow_disk_us = int(x);
            if (recorder_) recorder_->set_slow_disk_us(p_.slow_disk_us);
        }
        else { log_msg("error", "未知参数: " + k); return; }

        if (rebuild && cam_) {
            if (rec_active_ || rec_busy_) { log_msg("error", "录像中不能修改分辨率、格式、包大小或缓冲数"); return; }
            bool was = state_ == St::Acquiring;
            if (was) stop_acquisition();
            destroy_stream();
            if (!configure() || !create_stream()) { fail("重新配置失败"); return; }
            emit_info();
            if (was) start_acquisition();
        }
    }

    // ---------------- 枚举 ----------------
    void list_cameras() {
        arv_update_device_list();   // GigE：在每块网卡上广播发现请求
        unsigned n = arv_get_n_devices();
        std::string items;
        for (unsigned i = 0; i < n; ++i) {
            if (i) items += ",";
            items += "{\"id\":" + jstr(arv_get_device_id(i)) +
                     ",\"vendor\":" + jstr(arv_get_device_vendor(i)) +
                     ",\"model\":" + jstr(arv_get_device_model(i)) +
                     ",\"serial\":" + jstr(arv_get_device_serial_nbr(i)) +
                     ",\"address\":" + jstr(arv_get_device_address(i)) +
                     ",\"protocol\":" + jstr(arv_get_device_protocol(i)) + "}";
        }
        emit("{\"type\":\"cameras\",\"items\":[" + items + "]}");
    }

    std::string find_device_id(const std::string& serial) {
        arv_update_device_list();
        for (unsigned i = 0, n = arv_get_n_devices(); i < n; ++i) {
            const char* s = arv_get_device_serial_nbr(i);
            if (s && serial == s) return arv_get_device_id(i);
        }
        return {};
    }

    // ---------------- 状态机 ----------------
    void set_state(St s, int retry_ms = 0) {
        state_ = s;
        emit(fmt("{\"type\":\"state\",\"state\":\"%s\",\"serial\":%s,\"retry_in_ms\":%d,\"reconnects\":%" PRIu64 "}",
                 st_name(s), jstr(want_serial_).c_str(), retry_ms, reconnects_));
    }

    // 出错：释放资源，转入 Connecting，按退避间隔重试（想要采集的话重连后自动恢复）
    void fail(const std::string& why) {
        log_msg("warn", why);
        reconnects_++;
        set_state(St::Error);   // 先通知，再释放（释放可能要几十毫秒）
        teardown(false);        // 相机已经联系不上：不再发停止命令，否则要等好几秒超时
        next_try_ = Clock::now() + milliseconds(backoff_ms_);
        set_state(St::Connecting, backoff_ms_);
    }

    void tick() {
        auto now = Clock::now();
        switch (state_) {
            case St::Idle:
            case St::Error:
                std::this_thread::sleep_for(milliseconds(20));
                break;
            case St::Connecting:
                if (now < next_try_) { std::this_thread::sleep_for(milliseconds(20)); break; }
                if (connect()) {
                    backoff_ms_ = 100;
                    set_state(St::Ready);
                    emit_info();
                    if (want_acquire_) start_acquisition();
                } else {
                    next_try_ = Clock::now() + milliseconds(backoff_ms_);
                    set_state(St::Connecting, backoff_ms_);
                    backoff_ms_ = std::min(backoff_ms_ * 2, 5000);   // 指数退避
                }
                break;
            case St::Ready:
                std::this_thread::sleep_for(milliseconds(20));
                check_link(now);
                break;
            case St::Acquiring:
                grab(now);
                check_link(now);
                break;
        }
        if (rec_active_ && now >= rec_deadline_) record_stop(false);
    }

    // 掉线检测：Aravis 心跳线程发现控制通道丢失；或者连续采集时 2 秒没有任何帧。
    // 不在这里主动读寄存器：相机死掉时一次读操作要等好几秒超时，会卡住取帧和统计。
    void check_link(Clock::time_point now) {
        if (!cam_) return;
        // 相机收回了控制权之后就会停止推流，所以立即重连
        if (g_control_lost.exchange(false)) { fail("相机收回了控制权（心跳超时）"); return; }
        if (state_ == St::Acquiring && p_.trigger == "none" &&
            now - last_frame_ > seconds(2)) { fail("2 秒没有收到图像"); return; }
    }

    bool connect() {
        GError* e = nullptr;
        // 重连时先用上次的设备 ID 直接打开，省掉约 1 秒的广播发现；打不开再重新发现
        if (!last_dev_id_.empty() && last_dev_serial_ == want_serial_) {
            cam_ = arv_camera_new(last_dev_id_.c_str(), &e);
            if (e) { g_clear_error(&e); if (cam_) { g_object_unref(cam_); cam_ = nullptr; } }
        }
        std::string id = last_dev_id_;
        if (!cam_) {
            id = find_device_id(want_serial_);
            if (id.empty()) { log_msg("warn", "没有发现相机 " + want_serial_); return false; }
            cam_ = arv_camera_new(id.c_str(), &e);
        }
        if (e || !cam_) {
            log_msg("warn", std::string("打开失败: ") + (e ? e->message : "?"));
            if (e) g_error_free(e);
            if (cam_) { g_object_unref(cam_); cam_ = nullptr; }
            return false;
        }
        g_control_lost = false;
        ArvDevice* dev = arv_camera_get_device(cam_);
        // 相机端多久收不到心跳就收回控制权（GigE Vision 默认 3 秒），支持的话放宽到 10 秒
        if (ARV_IS_GV_DEVICE(dev) && arv_device_is_feature_available(dev, "GevHeartbeatTimeout", nullptr)) {
            GError* he = nullptr;
            arv_device_set_integer_feature_value(dev, "GevHeartbeatTimeout", 10000, &he);
            if (he) g_error_free(he);
        }
        if (ARV_IS_GV_DEVICE(dev)) {
            g_signal_connect(dev, "control-lost", G_CALLBACK(on_control_lost), nullptr);
            // 回环地址上 packet socket 收不到完整帧，改用普通 UDP socket
            GInetAddress* a = device_ip(dev);
            if (a && g_inet_address_get_is_loopback(a))
                arv_gv_device_set_stream_options(ARV_GV_DEVICE(dev),
                                                 ARV_GV_STREAM_OPTION_PACKET_SOCKET_DISABLED);
        }
        if (!configure() || !create_stream()) { teardown(); return false; }
        last_dev_id_ = id;
        last_dev_serial_ = want_serial_;
        return true;
    }

    // 下发参数。每一步失败只记日志，不中断（虚拟相机没有的功能会报 Not found）
    bool configure() {
        GError* e = nullptr;
        auto ok = [&](const char* what) {
            if (!e) return true;
            log_msg("warn", std::string(what) + ": " + e->message);
            g_clear_error(&e);
            return false;
        };
        if (arv_camera_is_gv_device(cam_)) {
            arv_camera_gv_set_packet_size(cam_, p_.packet_size, &e);
            ok("包大小");
        }
        if (!p_.pixfmt.empty()) { arv_camera_set_pixel_format_from_string(cam_, p_.pixfmt.c_str(), &e); ok("像素格式"); }
        if (p_.width > 0 || p_.height > 0) {
            gint x, y, w, h;
            arv_camera_get_region(cam_, &x, &y, &w, &h, &e);
            if (ok("读取分辨率")) {
                arv_camera_set_region(cam_, 0, 0, p_.width > 0 ? p_.width : w, p_.height > 0 ? p_.height : h, &e);
                ok("分辨率");
            }
        }
        if (arv_camera_is_exposure_auto_available(cam_, nullptr)) {
            arv_camera_set_exposure_time_auto(cam_, ARV_AUTO_OFF, &e);
            ok("关闭自动曝光");
        }
        apply_exposure();
        apply_trigger();
        payload_ = arv_camera_get_payload(cam_, &e);
        return ok("读取 payload") && payload_ > 0;
    }

    void apply_exposure() {
        if (!cam_) return;
        GError* e = nullptr;
        arv_camera_set_exposure_time(cam_, p_.exposure_us, &e);
        if (e) { log_msg("warn", std::string("曝光: ") + e->message); g_error_free(e); }
    }

    void apply_trigger() {
        if (!cam_) return;
        GError* e = nullptr;
        if (p_.trigger == "soft") {
            arv_camera_set_trigger(cam_, "Software", &e);
        } else {
            arv_camera_clear_triggers(cam_, &e);
            if (!e) arv_camera_set_frame_rate(cam_, p_.fps, &e);
        }
        if (e) { log_msg("warn", std::string("触发/帧率: ") + e->message); g_error_free(e); }
        last_frame_ = Clock::now();
    }

    bool create_stream() {
        GError* e = nullptr;
        stream_ = arv_camera_create_stream(cam_, nullptr, nullptr, &e);
        if (e || !stream_) {
            log_msg("error", std::string("创建 stream 失败: ") + (e ? e->message : "?"));
            if (e) g_error_free(e);
            return false;
        }
        if (ARV_IS_GV_STREAM(stream_))
            g_object_set(stream_, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, nullptr);
        for (int i = 0; i < p_.buffers; ++i)
            arv_stream_push_buffer(stream_, arv_buffer_new(payload_, nullptr));
        // SDK 统计项名称（不同版本、不同协议可能不同，先查有哪些）
        info_names_.clear();
        for (guint i = 0, n = arv_stream_get_n_infos(stream_); i < n; ++i)
            info_names_.insert(arv_stream_get_info_name(stream_, i));
        // 录像用的帧池按每帧字节数分配
        pool_ = FramePool::create(QCAP + 8, payload_);
        return true;
    }

    uint64_t info(const char* name) {
        if (!stream_ || !info_names_.count(name)) return 0;
        return arv_stream_get_info_uint64_by_name(stream_, name);
    }

    // 读取当前 stream 的 SDK 计数（累计值 = base_ + 当前 stream）
    Counters sdk_now() {
        Counters c = base_;
        if (stream_) {
            guint64 comp = 0, fail = 0, under = 0;
            arv_stream_get_statistics(stream_, &comp, &fail, &under);
            c.sdk_completed += comp; c.sdk_failures += fail; c.sdk_underruns += under;
            c.pkt_received += info("n_received_packets");
            c.pkt_missing += info("n_missing_packets");
            c.pkt_resend_req += info("n_resend_requests");
            c.pkt_resent += info("n_resent_packets");
            c.pkt_ignored += info("n_ignored_packets");
        }
        return c;
    }

    void destroy_stream() {
        if (!stream_) return;
        save_sdk_counters();
        g_object_unref(stream_);
        stream_ = nullptr;
    }

    // 把当前 stream 的 SDK 计数并入 base_（stream 销毁后计数就没了）
    void save_sdk_counters() {
        if (!stream_) return;
        Counters c = sdk_now();
        base_.sdk_completed = c.sdk_completed; base_.sdk_failures = c.sdk_failures;
        base_.sdk_underruns = c.sdk_underruns; base_.pkt_received = c.pkt_received;
        base_.pkt_missing = c.pkt_missing; base_.pkt_resend_req = c.pkt_resend_req;
        base_.pkt_resent = c.pkt_resent; base_.pkt_ignored = c.pkt_ignored;
    }

    // camera_alive=false 时只释放本地资源，不给相机发命令（断线时发命令会卡在超时上）
    void teardown(bool camera_alive = true) {
        if (camera_alive && cam_ && state_ == St::Acquiring) {
            GError* e = nullptr;
            arv_camera_stop_acquisition(cam_, &e);
            if (e) g_error_free(e);
        }
        if (cam_) {   // 旧设备的掉线信号不能再触发，否则会误伤新连接
            ArvDevice* dev = arv_camera_get_device(cam_);
            if (dev) g_signal_handlers_disconnect_by_func(dev, (gpointer)on_control_lost, nullptr);
        }
        if (camera_alive) {
            destroy_stream();
            if (cam_) g_object_unref(cam_);
        } else {
            // 释放 Aravis 对象时会尝试通知相机交还控制权，相机已断开就要等几秒超时，
            // 交给后台线程去等，引擎线程马上进入重连
            save_sdk_counters();
            ArvStream* s = stream_;
            ArvCamera* c = cam_;
            std::thread([s, c] {
                if (s) g_object_unref(s);
                if (c) g_object_unref(c);
            }).detach();
        }
        stream_ = nullptr;
        cam_ = nullptr;
        have_last_id_ = false;
    }

    void start_acquisition() {
        GError* e = nullptr;
        arv_camera_start_acquisition(cam_, &e);
        if (e) { std::string m = e->message; g_error_free(e); fail("开始采集失败: " + m); return; }
        last_frame_ = Clock::now();
        next_auto_ = Clock::now();
        have_last_id_ = false;   // 新的采集会话
        set_state(St::Acquiring);
    }

    void stop_acquisition() {
        GError* e = nullptr;
        arv_camera_stop_acquisition(cam_, &e);
        if (e) g_error_free(e);
        set_state(St::Ready);
    }

    void soft_trigger() {
        if (state_ != St::Acquiring || p_.trigger != "soft") { log_msg("warn", "需要先切到软触发并开始采集"); return; }
        GError* e = nullptr;
        arv_camera_software_trigger(cam_, &e);
        if (e) { log_msg("warn", std::string("软触发: ") + e->message); g_error_free(e); }
    }

    // ---------------- 取帧 ----------------
    void grab(Clock::time_point now) {
        if (p_.trigger == "soft" && p_.auto_trigger_ms > 0 && now >= next_auto_) {
            next_auto_ = now + milliseconds(p_.auto_trigger_ms);
            soft_trigger();
        }
        ArvBuffer* buf = arv_stream_timeout_pop_buffer(stream_, 20000);   // 最多等 20ms，保证能及时处理命令
        auto budget = Clock::now() + milliseconds(50);   // 每轮最多处理 50ms，处理不完的留给下一轮
        while (buf) {
            on_buffer(buf);
            arv_stream_push_buffer(stream_, buf);    // ★ 用完立刻还给 SDK
            if (p_.process_ms > 0)                   // 故障注入：取帧线程太慢 → SDK 缓冲用光（underrun）
                std::this_thread::sleep_for(milliseconds(p_.process_ms));
            if (Clock::now() >= budget) break;
            buf = arv_stream_try_pop_buffer(stream_);
        }
        if (preview_pending_ && Clock::now() >= next_preview_) send_preview();
    }

    void on_buffer(ArvBuffer* buf) {
        last_frame_ = Clock::now();   // 坏帧也说明链路还通（全部丢包时不能误判成断线）
        ArvBufferStatus status = arv_buffer_get_status(buf);
        if (status != ARV_BUFFER_STATUS_SUCCESS) {
            cnt_.bad++;
            const char* name = status == ARV_BUFFER_STATUS_MISSING_PACKETS ? "missing_packets"
                             : status == ARV_BUFFER_STATUS_TIMEOUT ? "timeout"
                             : status == ARV_BUFFER_STATUS_SIZE_MISMATCH ? "size_mismatch"
                             : status == ARV_BUFFER_STATUS_ABORTED ? "aborted" : "other";
            cnt_.bad_by[name]++;
            return;
        }
        size_t n = 0;
        const void* data = arv_buffer_get_data(buf, &n);
        uint64_t id = arv_buffer_get_frame_id(buf);
        cnt_.ok++;
        cnt_.bytes += n;
        last_id_shown_ = id;

        // 帧号连续性（入口处，只反映相机/链路丢帧），处理 16 位 block ID 绕回
        if (have_last_id_) {
            int64_t miss = frame_id_missing(last_id_, id);
            if (miss != 0) cnt_.link_gaps++;
            if (miss > 0) cnt_.link_missing += uint64_t(miss);
        }
        last_id_ = id;
        have_last_id_ = true;

        int w = arv_buffer_get_image_width(buf), h = arv_buffer_get_image_height(buf);
        ArvPixelFormat pf = arv_buffer_get_image_pixel_format(buf);

        // 预览：只在到点时拷贝一份，编码放到归还缓冲之后
        if (Clock::now() >= next_preview_ && !preview_pending_) {
            preview_.assign((const uint8_t*)data, (const uint8_t*)data + n);
            prev_w_ = w; prev_h_ = h; prev_pf_ = pf; prev_id_ = id;
            preview_pending_ = true;
        }

        // 录像：帧池取缓冲 → 拷贝 → 非阻塞入队（满了就计数丢弃）
        if (rec_active_ && rec_q_) {
            FramePtr f = pool_->acquire();
            if (!f) { rec_drop_pool_++; return; }
            size_t m = std::min(n, f->data.size());
            std::memcpy(f->data.data(), data, m);
            f->frame_id = id;
            guint64 ts = arv_buffer_get_timestamp(buf);
            f->dev_ts_ns = ts ? ts : arv_buffer_get_system_timestamp(buf);
            f->host_ts_ns = host_now_ns();
            f->width = uint32_t(w);
            f->height = uint32_t(h);
            f->pixfmt = uint32_t(pf);
            f->size = uint32_t(m);
            if (!rec_q_->try_push(std::move(f))) rec_drop_queue_++;
        }
    }

    void send_preview() {
        preview_pending_ = false;
        next_preview_ = Clock::now() + milliseconds(100);
        std::vector<unsigned char> jpeg;
        int ow = 0, oh = 0;
        if (!make_preview(preview_.data(), preview_.size(), prev_w_, prev_h_, prev_pf_, 640, jpeg, ow, oh)) return;
        emit(fmt("{\"type\":\"preview\",\"frame_id\":%" PRIu64 ",\"w\":%d,\"h\":%d,\"src_w\":%d,\"src_h\":%d,\"jpeg\":\"",
                 prev_id_, ow, oh, prev_w_, prev_h_) + base64(jpeg.data(), jpeg.size()) + "\"}");
    }

    // ---------------- 录像 ----------------
    void record_start(double secs, const std::string& dir) {
        if (state_ != St::Acquiring) { log_msg("warn", "需要先开始采集"); return; }
        if (rec_active_ || rec_busy_) { log_msg("warn", "上一段录像还没结束"); return; }
        if (dir.empty() || !(secs > 0 && secs <= 3600)) { log_msg("error", "record <秒数> <目录>"); return; }
        ::mkdir(dir.c_str(), 0755);
        rec_q_ = std::make_shared<BoundedQueue<FramePtr>>(QCAP);
        recorder_ = std::make_shared<Recorder>(*rec_q_, dir, 512ull << 20, p_.slow_disk_us);
        recorder_->start();
        rec_dir_ = dir;
        rec_drop_pool_ = rec_drop_queue_ = 0;
        rec_t0_ = Clock::now();
        rec_deadline_ = rec_t0_ + duration_cast<Clock::duration>(duration<double>(secs));
        rec_active_ = true;
        rec_busy_ = true;
        emit(fmt("{\"type\":\"record\",\"state\":\"started\",\"dir\":%s,\"secs\":%g}", jstr(dir).c_str(), secs));
    }

    // 停止录像：关队列，在另一个线程里等写盘线程排空，避免卡住取帧
    void record_stop(bool wait) {
        if (!rec_active_) return;
        rec_active_ = false;
        rec_q_->close();
        auto rec = recorder_;
        auto q = rec_q_;
        std::string dir = rec_dir_;
        double secs = duration<double>(Clock::now() - rec_t0_).count();
        uint64_t dp = rec_drop_pool_, dq = rec_drop_queue_;
        if (rec_thread_.joinable()) rec_thread_.join();
        rec_thread_ = std::thread([this, rec, q, dir, secs, dp, dq] {
            rec->join();
            auto& s = rec->stats();
            std::string files;
            for (int i = 0; i < rec->file_count(); ++i)
                files += (i ? "," : "") + jstr(fmt("%s/rec_%04d.bin", dir.c_str(), i));
            emit(fmt("{\"type\":\"record\",\"state\":\"done\",\"dir\":%s,\"files\":[%s],\"secs\":%.2f,"
                     "\"written\":%" PRIu64 ",\"bytes\":%" PRIu64 ",\"drop_pool\":%" PRIu64 ",\"drop_queue\":%" PRIu64
                     ",\"file_gaps\":%" PRIu64 ",\"file_missing\":%" PRIu64 ",\"max_write_us\":%" PRIu64 "}",
                     jstr(dir).c_str(), files.c_str(), secs, s.written.load(), s.bytes.load(), dp, dq,
                     s.gaps.load(), s.missing.load(), s.max_write_us.load()));
            rec_busy_ = false;
        });
        if (wait) rec_thread_.join();
    }

    // ---------------- 统计 ----------------
    void reset_counters() {
        cnt_ = Counters{};
        base_ = Counters{};
        if (stream_) {   // 当前 stream 的 SDK 计数不能清零，记下来做减法
            Counters c = sdk_now();
            base_.sdk_completed = 0 - c.sdk_completed; base_.sdk_failures = 0 - c.sdk_failures;
            base_.sdk_underruns = 0 - c.sdk_underruns; base_.pkt_received = 0 - c.pkt_received;
            base_.pkt_missing = 0 - c.pkt_missing; base_.pkt_resend_req = 0 - c.pkt_resend_req;
            base_.pkt_resent = 0 - c.pkt_resent; base_.pkt_ignored = 0 - c.pkt_ignored;
        }
        reconnects_ = 0;
        rate_hist_.clear();
    }

    void emit_info() {
        if (!cam_) return;
        GError* e = nullptr;
        gint x = 0, y = 0, w = 0, h = 0, sw = 0, sh = 0;
        arv_camera_get_region(cam_, &x, &y, &w, &h, &e); g_clear_error(&e);
        arv_camera_get_sensor_size(cam_, &sw, &sh, &e); g_clear_error(&e);
        double emin = 0, emax = 0, fmin = 0, fmax = 0;
        arv_camera_get_exposure_time_bounds(cam_, &emin, &emax, &e); g_clear_error(&e);
        arv_camera_get_frame_rate_bounds(cam_, &fmin, &fmax, &e); g_clear_error(&e);
        double exposure = arv_camera_get_exposure_time(cam_, &e); g_clear_error(&e);
        double fps = arv_camera_get_frame_rate(cam_, &e); g_clear_error(&e);
        const char* pf = arv_camera_get_pixel_format_as_string(cam_, &e); g_clear_error(&e);
        guint n_pf = 0;
        const char** pfs = arv_camera_dup_available_pixel_formats_as_strings(cam_, &n_pf, &e); g_clear_error(&e);
        std::string pf_list;
        for (guint i = 0; pfs && i < n_pf; ++i) pf_list += (i ? "," : "") + jstr(pfs[i]);
        g_free(pfs);
        int packet = arv_camera_is_gv_device(cam_) ? int(arv_camera_gv_get_packet_size(cam_, &e)) : 0;
        g_clear_error(&e);
        // GVSP 每包有效数据 = 包大小 - IP(20) - UDP(8) - GVSP(8) 头
        int per_packet = packet > 36 ? packet - 36 : 0;
        int packets = per_packet ? int((payload_ + per_packet - 1) / per_packet) : 0;
        ArvDevice* dev = arv_camera_get_device(cam_);
        std::string addr;
        if (ARV_IS_GV_DEVICE(dev)) {
            GInetAddress* a = device_ip(dev);
            if (a) { char* s = g_inet_address_to_string(a); addr = s; g_free(s); }
        }
        emit(fmt("{\"type\":\"info\",\"vendor\":%s,\"model\":%s,\"serial\":%s,\"address\":%s,"
                 "\"width\":%d,\"height\":%d,\"sensor_w\":%d,\"sensor_h\":%d,\"pixfmt\":%s,\"pixfmts\":[%s],"
                 "\"exposure\":%.1f,\"exposure_min\":%.1f,\"exposure_max\":%.1f,"
                 "\"fps\":%.2f,\"fps_min\":%.2f,\"fps_max\":%.2f,\"packet_size\":%d,\"payload\":%u,"
                 "\"packets_per_frame\":%d,\"buffers\":%d,\"trigger\":%s}",
                 jstr(arv_camera_get_vendor_name(cam_, nullptr)).c_str(),
                 jstr(arv_camera_get_model_name(cam_, nullptr)).c_str(),
                 jstr(arv_camera_get_device_serial_number(cam_, nullptr)).c_str(), jstr(addr).c_str(),
                 w, h, sw, sh, jstr(pf).c_str(), pf_list.c_str(), exposure, emin, emax, fps, fmin, fmax,
                 packet, payload_, packets, p_.buffers, jstr(p_.trigger).c_str()));
    }

    void emit_stats() {
        Counters c = sdk_now();
        auto now = Clock::now();
        // 最近 1 秒的帧率和吞吐
        rate_hist_.push_back({now, cnt_.ok, cnt_.bytes});
        while (rate_hist_.size() > 1 && now - rate_hist_.front().t > milliseconds(1050)) rate_hist_.pop_front();
        double fps = 0, mbps = 0;
        if (rate_hist_.size() > 1) {
            double dt = duration<double>(rate_hist_.back().t - rate_hist_.front().t).count();
            if (dt > 0) {
                fps = (rate_hist_.back().ok - rate_hist_.front().ok) / dt;
                mbps = (rate_hist_.back().bytes - rate_hist_.front().bytes) / dt / 1e6;
            }
        }
        std::string bad_by;
        for (auto& [k, v] : cnt_.bad_by) bad_by += (bad_by.empty() ? "" : ",") + jstr(k) + fmt(":%" PRIu64, v);

        std::string rec = "null";
        if (recorder_ && (rec_active_ || rec_busy_)) {
            auto& s = recorder_->stats();
            rec = fmt("{\"active\":%s,\"elapsed\":%.1f,\"written\":%" PRIu64 ",\"bytes\":%" PRIu64
                      ",\"queue\":%zu,\"qcap\":%zu,\"drop_pool\":%" PRIu64 ",\"drop_queue\":%" PRIu64
                      ",\"file_gaps\":%" PRIu64 ",\"max_write_us\":%" PRIu64 "}",
                      rec_active_ ? "true" : "false", duration<double>(now - rec_t0_).count(),
                      s.written.load(), s.bytes.load(), rec_q_->size(), rec_q_->capacity(),
                      rec_drop_pool_, rec_drop_queue_, s.gaps.load(), s.max_write_us.load());
        }
        emit(fmt("{\"type\":\"stats\",\"state\":\"%s\",\"fps\":%.2f,\"mbps\":%.2f,\"frame_id\":%" PRIu64
                 ",\"ok\":%" PRIu64 ",\"bad\":%" PRIu64 ",\"bad_by\":{%s},\"link_gaps\":%" PRIu64
                 ",\"link_missing\":%" PRIu64 ",\"sdk\":{\"completed\":%" PRIu64 ",\"failures\":%" PRIu64
                 ",\"underruns\":%" PRIu64 "},\"pkt\":{\"received\":%" PRIu64 ",\"missing\":%" PRIu64
                 ",\"resend_requests\":%" PRIu64 ",\"resent\":%" PRIu64 ",\"ignored\":%" PRIu64
                 "},\"reconnects\":%" PRIu64 ",\"process_ms\":%d,\"slow_disk_us\":%d,\"rec\":%s}",
                 st_name(state_), fps, mbps, last_id_shown_, cnt_.ok, cnt_.bad, bad_by.c_str(),
                 cnt_.link_gaps, cnt_.link_missing, c.sdk_completed, c.sdk_failures, c.sdk_underruns,
                 c.pkt_received, c.pkt_missing, c.pkt_resend_req, c.pkt_resent, c.pkt_ignored,
                 reconnects_, p_.process_ms, p_.slow_disk_us, rec.c_str()));
    }

    static constexpr size_t QCAP = 64;
    CommandQueue& cq_;
    bool quit_ = false;
    St state_ = St::Idle;
    Params p_;
    std::string want_serial_;
    std::string last_dev_id_, last_dev_serial_;   // 上次成功打开的设备，重连时直接用
    bool want_acquire_ = false;
    int backoff_ms_ = 100;
    uint64_t reconnects_ = 0;
    Clock::time_point next_try_, last_frame_, next_auto_, next_preview_;

    ArvCamera* cam_ = nullptr;
    ArvStream* stream_ = nullptr;
    guint payload_ = 0;
    std::set<std::string> info_names_;
    std::shared_ptr<FramePool> pool_;

    Counters cnt_, base_;
    uint64_t last_id_ = 0, last_id_shown_ = 0;
    bool have_last_id_ = false;
    struct Rate { Clock::time_point t; uint64_t ok, bytes; };
    std::deque<Rate> rate_hist_;

    std::vector<uint8_t> preview_;
    bool preview_pending_ = false;
    int prev_w_ = 0, prev_h_ = 0;
    ArvPixelFormat prev_pf_ = 0;
    uint64_t prev_id_ = 0;

    std::shared_ptr<BoundedQueue<FramePtr>> rec_q_;
    std::shared_ptr<Recorder> recorder_;
    std::thread rec_thread_;
    std::atomic<bool> rec_busy_{false};
    bool rec_active_ = false;
    std::string rec_dir_;
    Clock::time_point rec_t0_, rec_deadline_;
    uint64_t rec_drop_pool_ = 0, rec_drop_queue_ = 0;
};

int main() {
    std::ios::sync_with_stdio(false);
    CommandQueue cq;
    Engine eng(cq);
    std::thread th([&] { eng.run(); });
    emit("{\"type\":\"hello\",\"version\":1}");
    std::string line;
    while (std::getline(std::cin, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        cq.push(line);
        if (line == "quit") break;
    }
    cq.push("quit");   // stdin 关闭（父进程退出）时也退出
    th.join();
    arv_shutdown();
    return 0;
}

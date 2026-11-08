// OpenCV 版采集：读普通 USB 摄像头（UVC 协议，笔记本自带摄像头 / USB 网络摄像头）
//   OpenCV 采集(主线程) -> 帧池 + 有界队列 -> 写盘线程(CRC/滚动) -> 统计线程
//   和 sim_capture 共用 frame.h / frame_queue.h / recorder.h，录制文件可用 verify_record.py 校验
//
// 安装: sudo apt install libopencv-dev
// 用法: ./uvc_capture [秒数=10] [设备=0] [宽=1280] [高=720] [fps=30] [输出目录=./rec_uvc] [--show] [--gray]
//   设备: 数字是摄像头序号(0 = /dev/video0)，也可以是设备路径、视频文件或 rtsp:// 地址
//         视频文件按文件自身帧率读取并循环播放，用来在没有摄像头时模拟采集
//   --show: 弹窗预览（WSL 需要 WSLg），按 q 或 Esc 提前退出
//   --gray: 存灰度 Mono8（数据量为 RGB 的 1/3），默认存 RGB8
// 例:   ./uvc_capture 10 0 1280 720 30 ./rec_uvc --show
//
// 注意：WSL2 默认看不到 USB 摄像头，需要 usbipd-win 转发 + 带 uvcvideo 驱动的内核；
//       这个程序不依赖 Linux 专有接口，在 Windows 上用 OpenCV 编译后可直接读摄像头。
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "frame.h"
#include "frame_queue.h"
#include "recorder.h"

using namespace std::chrono;

struct CaptureStats {
    std::atomic<uint64_t> received{0};
    std::atomic<uint64_t> dropped_pool{0};   // 帧池耗尽导致的丢弃
    std::atomic<uint64_t> dropped_queue{0};  // 队列满导致的丢弃
    std::atomic<uint64_t> dropped_size{0};   // 重连后分辨率变大、放不进帧池缓冲
    std::atomic<uint64_t> read_fail{0};      // read() 失败次数
    std::atomic<uint64_t> reconnects{0};
    std::atomic<uint64_t> loops{0};          // 视频文件循环播放次数
};

struct Options {
    int secs = 10;
    std::string device = "0";
    int width = 1280, height = 720;
    double fps = 30;
    std::string dir = "./rec_uvc";
    bool show = false;
    bool gray = false;
};

static bool is_number(const std::string& s) {
    return !s.empty() && s.find_first_not_of("0123456789") == std::string::npos;
}

// 本地视频文件：不是摄像头序号，也不是 rtsp:// / http:// 这类网络流
static bool is_video_file(const std::string& s) {
    return !is_number(s) && s.find("://") == std::string::npos;
}

// 打开摄像头并设置参数；摄像头不一定接受请求的值，以实际读回的为准
static bool open_camera(cv::VideoCapture& cap, const Options& o) {
    if (is_number(o.device)) {
#ifdef __linux__
        cap.open(std::atoi(o.device.c_str()), cv::CAP_V4L2);
#else
        cap.open(std::atoi(o.device.c_str()), cv::CAP_ANY);
#endif
    } else {
        cap.open(o.device);
    }
    if (!cap.isOpened()) return false;

    if (is_video_file(o.device)) {  // 文件的分辨率/帧率由文件决定，设置无效
        std::printf("[open] video file %dx%d @ %.1f fps, %d frames (loop playback)\n",
                    int(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
                    int(cap.get(cv::CAP_PROP_FRAME_HEIGHT)), cap.get(cv::CAP_PROP_FPS),
                    int(cap.get(cv::CAP_PROP_FRAME_COUNT)));
        return true;
    }

    // MJPG 压缩格式通常能在高分辨率下跑满帧率；YUYV 在 USB2 上 720p 往往只有 10fps
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, o.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, o.height);
    cap.set(cv::CAP_PROP_FPS, o.fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 2);  // 驱动侧少缓存，降低延迟

    int fourcc = int(cap.get(cv::CAP_PROP_FOURCC));
    char fcc[5] = {char(fourcc & 0xFF), char((fourcc >> 8) & 0xFF),
                   char((fourcc >> 16) & 0xFF), char((fourcc >> 24) & 0xFF), 0};
    std::printf("[open] backend=%s actual %dx%d @ %.1f fps, fourcc=%s\n",
                cap.getBackendName().c_str(), int(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
                int(cap.get(cv::CAP_PROP_FRAME_HEIGHT)), cap.get(cv::CAP_PROP_FPS), fcc);
    return true;
}

int main(int argc, char** argv) {
    Options o;
    int pos = 0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--show") { o.show = true; continue; }
        if (a == "--gray") { o.gray = true; continue; }
        if (a == "-h" || a == "--help") {
            std::printf("usage: %s [secs=10] [device=0] [w=1280] [h=720] [fps=30] "
                        "[dir=./rec_uvc] [--show] [--gray]\n", argv[0]);
            return 0;
        }
        switch (pos++) {
            case 0: o.secs = std::atoi(a.c_str()); break;
            case 1: o.device = a; break;
            case 2: o.width = std::atoi(a.c_str()); break;
            case 3: o.height = std::atoi(a.c_str()); break;
            case 4: o.fps = std::atof(a.c_str()); break;
            case 5: o.dir = a; break;
            default: std::fprintf(stderr, "ignored arg: %s\n", a.c_str());
        }
    }
    std::error_code ec;
    std::filesystem::create_directories(o.dir, ec);

    cv::VideoCapture cap;
    int backoff_ms = 100;
    auto deadline = steady_clock::now() + seconds(o.secs);
    while (!open_camera(cap, o)) {
        if (steady_clock::now() >= deadline) {
            std::fprintf(stderr, "cannot open camera '%s'\n", o.device.c_str());
#ifdef __linux__
            std::fprintf(stderr, "hint: check that /dev/video%s exists "
                                 "(WSL2 needs usbipd-win + uvcvideo kernel driver)\n",
                         is_number(o.device) ? o.device.c_str() : "*");
#endif
            return 1;
        }
        std::printf("  open failed, retry in %d ms\n", backoff_ms);
        std::this_thread::sleep_for(milliseconds(backoff_ms));
        backoff_ms = std::min(backoff_ms * 2, 5000);  // 指数退避
    }

    // 帧池按首次打开的实际分辨率分配
    const uint32_t ch = o.gray ? 1 : 3;
    const uint32_t pixfmt = o.gray ? PIX_MONO8 : PIX_RGB8;
    size_t frame_bytes = size_t(cap.get(cv::CAP_PROP_FRAME_WIDTH)) *
                         size_t(cap.get(cv::CAP_PROP_FRAME_HEIGHT)) * ch;
    if (frame_bytes == 0) frame_bytes = size_t(o.width) * o.height * ch;
    std::printf("config: %ds, device=%s, %s, %.1f MB/frame, out=%s\n", o.secs,
                o.device.c_str(), o.gray ? "Mono8" : "RGB8", frame_bytes / 1e6,
                o.dir.c_str());

    const size_t QCAP = 64;
    auto pool = FramePool::create(QCAP + 8, frame_bytes);
    BoundedQueue<FramePtr> q(QCAP);
    Recorder rec(q, o.dir, 512ull << 20);  // 每 512MB 滚动一个文件
    rec.start();

    CaptureStats st;
    std::atomic<bool> done{false};
    std::thread mon([&] {
        uint64_t last_w = 0, last_b = 0;
        while (!done) {
            std::this_thread::sleep_for(seconds(1));
            auto& rs = rec.stats();
            uint64_t wn = rs.written, bn = rs.bytes;
            std::printf("[stat] recv=%" PRIu64 " write=%" PRIu64 " fps=%" PRIu64
                        " %.1fMB/s queue=%zu/%zu drop(pool/queue/size)=%" PRIu64 "/%" PRIu64
                        "/%" PRIu64 " readFail=%" PRIu64 "\n",
                        st.received.load(), wn, wn - last_w, (bn - last_b) / 1e6, q.size(),
                        q.capacity(), st.dropped_pool.load(), st.dropped_queue.load(),
                        st.dropped_size.load(), st.read_fail.load());
            last_w = wn; last_b = bn;
        }
    });

    // 采集循环在主线程：OpenCV 的 imshow/waitKey 要求在同一线程调用
    cv::Mat raw, conv;
    uint64_t frame_id = 0;
    int consecutive_fail = 0;

    // 视频文件：按文件自身帧率限速，模拟摄像头的出图节拍（否则会以解码速度狂读）
    const bool file_mode = is_video_file(o.device);
    double file_fps = file_mode ? cap.get(cv::CAP_PROP_FPS) : 0;
    if (file_mode && !(file_fps > 0 && file_fps < 1000)) file_fps = o.fps;
    const auto period = file_mode ? duration_cast<steady_clock::duration>(
                                        duration<double>(1.0 / file_fps))
                                  : steady_clock::duration::zero();
    auto next_tick = steady_clock::now();

    while (steady_clock::now() < deadline) {
        if (file_mode) {
            next_tick += period;
            auto now = steady_clock::now();
            if (next_tick > now)
                std::this_thread::sleep_until(next_tick);
            else if (now - next_tick > period * 5)
                next_tick = now;  // 落后太多（如被调试器暂停）就重新对齐，不要补帧狂读
        }
        bool got = cap.read(raw) && !raw.empty();
        // 视频文件读到结尾：跳回开头循环播放，不算断线
        if (!got && file_mode && cap.isOpened() && cap.set(cv::CAP_PROP_POS_FRAMES, 0) &&
            cap.read(raw) && !raw.empty()) {
            st.loops++;
            got = true;
        }
        if (!got) {
            st.read_fail++;
            // 连续失败视为断线（USB 被拔 / 设备被占用），释放后重连
            if (++consecutive_fail >= 10) {
                std::printf("[state] camera lost, reconnecting\n");
                cap.release();
                backoff_ms = 100;
                while (steady_clock::now() < deadline && !open_camera(cap, o)) {
                    std::this_thread::sleep_for(milliseconds(backoff_ms));
                    backoff_ms = std::min(backoff_ms * 2, 5000);
                }
                st.reconnects++;
                consecutive_fail = 0;
            }
            continue;
        }
        consecutive_fail = 0;
        uint64_t host_ts = host_now_ns();
        // Linux V4L2 下 POS_MSEC 是驱动给的缓冲区时间戳（单调时钟），比主机时间更接近曝光时刻；
        // 视频文件的 POS_MSEC 是片内播放位置，循环时会跳回 0，所以文件模式直接用主机时间
        double pos_ms = file_mode ? 0 : cap.get(cv::CAP_PROP_POS_MSEC);
        uint64_t dev_ts = pos_ms > 0 ? uint64_t(pos_ms * 1e6) : host_ts;
        ++frame_id;  // UVC 不提供帧号，自己计数；丢帧只会体现在 drop 计数里
        st.received++;

        // OpenCV 默认给 BGR，转成 RGB8 / Mono8 再存，与 PixelFormat 定义一致
        if (o.gray)
            cv::cvtColor(raw, conv, cv::COLOR_BGR2GRAY);
        else
            cv::cvtColor(raw, conv, cv::COLOR_BGR2RGB);
        if (!conv.isContinuous()) conv = conv.clone();
        size_t n = conv.total() * conv.elemSize();

        FramePtr f = pool->acquire();
        if (!f) {
            st.dropped_pool++;
        } else if (n > f->data.size()) {
            st.dropped_size++;
        } else {
            std::memcpy(f->data.data(), conv.data, n);
            f->frame_id = frame_id;
            f->dev_ts_ns = dev_ts;
            f->host_ts_ns = host_ts;
            f->width = uint32_t(conv.cols);
            f->height = uint32_t(conv.rows);
            f->pixfmt = pixfmt;
            f->size = uint32_t(n);
            if (!q.try_push(std::move(f))) st.dropped_queue++;  // f 析构自动归还池
        }

        if (o.show) {
            cv::imshow("uvc_capture (q/Esc to quit)", raw);
            int key = cv::waitKey(1);
            if (key == 'q' || key == 27) break;
        }
    }
    cap.release();
    if (o.show) cv::destroyAllWindows();

    q.close();  // 通知写盘线程：不再有新数据，排空后退出
    rec.join();
    done = true;
    mon.join();

    auto& rs = rec.stats();
    std::printf("\n==== summary ====\n"
                "received      : %" PRIu64 "\n"
                "written       : %" PRIu64 "\n"
                "drop pool     : %" PRIu64 "\n"
                "drop queue    : %" PRIu64 "\n"
                "drop size     : %" PRIu64 "\n"
                "read fails    : %" PRIu64 "\n"
                "reconnects    : %" PRIu64 "\n"
                "file loops    : %" PRIu64 "\n"
                "max write     : %" PRIu64 " us\n",
                st.received.load(), rs.written.load(), st.dropped_pool.load(),
                st.dropped_queue.load(), st.dropped_size.load(), st.read_fail.load(),
                st.reconnects.load(), st.loops.load(), rs.max_write_us.load());
    return 0;
}

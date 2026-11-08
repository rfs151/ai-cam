// 模拟采集程序：不接相机也能跑，演示完整链路
//   模拟相机(回调线程) -> 帧池 + 有界队列 -> 写盘线程(CRC/滚动) -> 统计线程
//   采集状态机：断开 -> 连接 -> 配置 -> 采集 -> 出错 -> 指数退避重连
//
// 用法: ./sim_capture [秒数=10] [fps=100] [宽=1280] [高=1024] [慢盘us=0] [输出目录=./rec]
// 例:   ./sim_capture 10 200 2048 1536 8000   # 慢盘，观察队列打满、丢帧计数
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include <sys/stat.h>
#include <thread>

#include "frame.h"
#include "frame_queue.h"
#include "recorder.h"

using namespace std::chrono;

// ---------------------------------------------------------------------------
// 模拟相机：独立线程按帧率出图，回调给上层；会随机漏帧号、并在指定时刻"断线"
// 真实 SDK（海康 MVS / Aravis）也是类似的：注册回调或轮询取帧 + 异常回调
// ---------------------------------------------------------------------------
class SimCamera {
public:
    using FrameCb = std::function<void(uint64_t id, uint64_t dev_ts, const uint8_t*, size_t,
                                       uint32_t w, uint32_t h)>;
    using LostCb = std::function<void()>;

    bool open() {
        // 模拟：前两次连接失败，验证重试逻辑
        return ++open_tries_ > 2;
    }
    void set_params(uint32_t w, uint32_t h, double fps) {
        w_ = w; h_ = h; fps_ = fps;
        img_.assign(size_t(w) * h, 0);
    }
    void start(FrameCb cb, LostCb lost, int disconnect_after_ms) {
        running_ = true;
        th_ = std::thread([=] {
            std::mt19937 rng(42);
            std::uniform_int_distribution<int> d(0, 999);
            auto period = duration<double>(1.0 / fps_);
            auto next = steady_clock::now();
            auto t_start = next;
            while (running_) {
                next += duration_cast<steady_clock::duration>(period);
                std::this_thread::sleep_until(next);
                if (disconnect_after_ms > 0 &&
                    steady_clock::now() - t_start > milliseconds(disconnect_after_ms)) {
                    running_ = false;
                    lost();                     // 模拟网线被拔：异常回调
                    return;
                }
                ++frame_id_;
                if (d(rng) < 2) continue;       // 0.2% 概率链路丢帧（帧号跳变）
                img_[0] = uint8_t(frame_id_);   // 让每帧内容不同，CRC 才有意义
                cb(frame_id_, host_now_ns(), img_.data(), img_.size(), w_, h_);
            }
        });
    }
    void stop() {
        running_ = false;
        if (th_.joinable()) th_.join();
    }
    ~SimCamera() { stop(); }

private:
    int open_tries_ = 0;
    uint32_t w_ = 0, h_ = 0;
    double fps_ = 30;
    std::vector<uint8_t> img_;
    std::atomic<bool> running_{false};
    std::thread th_;
    uint64_t frame_id_ = 0;  // 断线重连后继续递增（真实相机重连后通常会从头计数）
};

// ---------------------------------------------------------------------------
// 采集控制器：状态机 + 重连
// ---------------------------------------------------------------------------
enum class State { Disconnected, Connecting, Configuring, Acquiring, Error, Stopped };

static const char* to_str(State s) {
    switch (s) {
        case State::Disconnected: return "Disconnected";
        case State::Connecting:   return "Connecting";
        case State::Configuring:  return "Configuring";
        case State::Acquiring:    return "Acquiring";
        case State::Error:        return "Error";
        case State::Stopped:      return "Stopped";
    }
    return "?";
}

struct CaptureStats {
    std::atomic<uint64_t> received{0};
    std::atomic<uint64_t> dropped_pool{0};   // 帧池耗尽导致的丢弃
    std::atomic<uint64_t> dropped_queue{0};  // 队列满导致的丢弃
    std::atomic<uint64_t> reconnects{0};
    // 在 on_frame 入口按帧号检查：只反映相机/链路丢帧，不含程序内部（池/队列）丢帧
    std::atomic<uint64_t> link_gaps{0};      // 帧号不连续次数
    std::atomic<uint64_t> link_missing{0};   // 估算缺失帧数
};

class CaptureController {
public:
    CaptureController(BoundedQueue<FramePtr>& q, std::shared_ptr<FramePool> pool,
                      uint32_t w, uint32_t h, double fps)
        : q_(q), pool_(std::move(pool)), w_(w), h_(h), fps_(fps) {}

    void run_until(steady_clock::time_point deadline) {
        int backoff_ms = 100;
        bool first_session = true;
        while (steady_clock::now() < deadline) {
            switch (state_) {
            case State::Disconnected:
                set_state(State::Connecting);
                break;

            case State::Connecting:
                if (cam_.open()) {
                    backoff_ms = 100;
                    set_state(State::Configuring);
                } else {
                    std::printf("  open failed, retry in %d ms\n", backoff_ms);
                    std::this_thread::sleep_for(milliseconds(backoff_ms));
                    backoff_ms = std::min(backoff_ms * 2, 5000);  // 指数退避
                }
                break;

            case State::Configuring:
                // 真实相机这里设置：像素格式、ROI、曝光、增益、触发模式、包大小等
                cam_.set_params(w_, h_, fps_);
                lost_ = false;
                // 新的采集会话：真实相机重连后帧号通常从头计数，不清掉会被误判成丢帧。
                // 这里在 cam_.start() 之前执行，相机线程还没启动，不存在并发访问。
                have_last_id_ = false;
                cam_.start(
                    [this](uint64_t id, uint64_t ts, const uint8_t* p, size_t n,
                           uint32_t w, uint32_t h) { on_frame(id, ts, p, n, w, h); },
                    [this] { lost_ = true; },
                    first_session ? 3000 : 0);   // 第一轮 3 秒后模拟断线
                first_session = false;
                set_state(State::Acquiring);
                break;

            case State::Acquiring:
                // 控制线程只做监视，不碰数据；数据走回调 -> 队列
                std::this_thread::sleep_for(milliseconds(50));
                if (lost_) set_state(State::Error);
                break;

            case State::Error:
                cam_.stop();          // 释放资源后重连
                st_.reconnects++;
                set_state(State::Disconnected);
                break;

            case State::Stopped:
                return;
            }
        }
        cam_.stop();
        set_state(State::Stopped);
    }

    CaptureStats& stats() { return st_; }

private:
    void set_state(State s) {
        std::printf("[state] %s -> %s\n", to_str(state_), to_str(s));
        state_ = s;
    }

    // 回调线程里只做：拿缓冲 -> 拷贝 -> 打元数据 -> 非阻塞入队。不做任何耗时操作。
    void on_frame(uint64_t id, uint64_t dev_ts, const uint8_t* p, size_t n,
                  uint32_t w, uint32_t h) {
        st_.received++;
        // 帧号连续性检查放在最前面（拿缓冲、入队之前），统计到的只有相机/链路丢的帧。
        // 只有相机线程会调用 on_frame，last_id_ 不需要加锁。
        if (have_last_id_) {
            int64_t miss = frame_id_missing(last_id_, id);   // 处理帧号绕回
            if (miss != 0) st_.link_gaps++;
            if (miss > 0) st_.link_missing += uint64_t(miss);
        }
        last_id_ = id;
        have_last_id_ = true;

        FramePtr f = pool_->acquire();
        if (!f) { st_.dropped_pool++; return; }
        if (n > f->data.size()) n = f->data.size();
        std::memcpy(f->data.data(), p, n);
        f->frame_id = id;
        f->dev_ts_ns = dev_ts;
        f->host_ts_ns = host_now_ns();
        f->width = w;
        f->height = h;
        f->pixfmt = PIX_MONO8;
        f->size = uint32_t(n);
        if (!q_.try_push(std::move(f))) st_.dropped_queue++;  // f 析构自动归还池
    }

    BoundedQueue<FramePtr>& q_;
    std::shared_ptr<FramePool> pool_;
    uint32_t w_, h_;
    double fps_;
    SimCamera cam_;
    std::atomic<bool> lost_{false};
    State state_ = State::Disconnected;
    CaptureStats st_;
    uint64_t last_id_ = 0;        // 上一帧帧号（只在相机线程里读写）
    bool have_last_id_ = false;
};

int main(int argc, char** argv) {
    int secs        = argc > 1 ? std::atoi(argv[1]) : 10;
    double fps      = argc > 2 ? std::atof(argv[2]) : 100;
    uint32_t w      = argc > 3 ? std::atoi(argv[3]) : 1280;
    uint32_t h      = argc > 4 ? std::atoi(argv[4]) : 1024;
    int slow_us     = argc > 5 ? std::atoi(argv[5]) : 0;
    std::string dir = argc > 6 ? argv[6] : "./rec";
    ::mkdir(dir.c_str(), 0755);

    size_t frame_bytes = size_t(w) * h;
    std::printf("config: %ds, %.0f fps, %ux%u Mono8, %.1f MB/s, slow_disk=%dus\n",
                secs, fps, w, h, frame_bytes * fps / 1e6, slow_us);

    // 帧池比队列大一些：队列满时池里还有余量给正在处理的帧
    const size_t QCAP = 64;
    auto pool = FramePool::create(QCAP + 8, frame_bytes);
    BoundedQueue<FramePtr> q(QCAP);

    Recorder rec(q, dir, 512ull << 20, slow_us);  // 每 512MB 滚动一个文件
    rec.start();

    CaptureController ctl(q, pool, w, h, fps);

    // 统计线程：每秒打印吞吐、队列水位、丢帧
    std::atomic<bool> done{false};
    std::thread mon([&] {
        uint64_t last_w = 0, last_b = 0;
        while (!done) {
            std::this_thread::sleep_for(seconds(1));
            auto& cs = ctl.stats();
            auto& rs = rec.stats();
            uint64_t wn = rs.written, bn = rs.bytes;
            std::printf("[stat] recv=%" PRIu64 " write=%" PRIu64 " fps=%" PRIu64
                        " %.1fMB/s queue=%zu/%zu drop(pool/queue)=%" PRIu64 "/%" PRIu64
                        " linkGap=%" PRIu64 " fileGap=%" PRIu64 " maxWrite=%" PRIu64 "us\n",
                        cs.received.load(), wn, wn - last_w, (bn - last_b) / 1e6,
                        q.size(), q.capacity(), cs.dropped_pool.load(),
                        cs.dropped_queue.load(), cs.link_gaps.load(), rs.gaps.load(),
                        rs.max_write_us.load());
            last_w = wn; last_b = bn;
        }
    });

    ctl.run_until(steady_clock::now() + seconds(secs));

    q.close();     // 通知写盘线程：不再有新数据，排空后退出
    rec.join();
    done = true;
    mon.join();

    auto& cs = ctl.stats();
    auto& rs = rec.stats();
    std::printf("\n==== summary ====\n"
                "received      : %" PRIu64 "\n"
                "written       : %" PRIu64 "\n"
                "drop pool     : %" PRIu64 "   (程序内部：帧池耗尽)\n"
                "drop queue    : %" PRIu64 "   (程序内部：队列满，写盘跟不上)\n"
                "link gaps     : %" PRIu64 " (missing ~%" PRIu64 " frames)   (相机/链路丢帧)\n"
                "file gaps     : %" PRIu64 " (missing ~%" PRIu64 " frames)   (文件缺帧 = 链路 + 内部)\n"
                "reconnects    : %" PRIu64 "\n"
                "max write     : %" PRIu64 " us\n",
                cs.received.load(), rs.written.load(), cs.dropped_pool.load(),
                cs.dropped_queue.load(), cs.link_gaps.load(), cs.link_missing.load(),
                rs.gaps.load(), rs.missing.load(), cs.reconnects.load(),
                rs.max_write_us.load());
    return 0;
}

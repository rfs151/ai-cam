// 写盘线程：从队列取帧 -> 帧号连续性检查 -> 加文件头和 CRC -> 按大小滚动文件
// 和采集线程完全解耦：写盘慢只会让队列变满、采集端计数丢帧，不会卡住设备控制。
#pragma once
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <string>
#include <thread>

#include "frame.h"
#include "frame_queue.h"

#pragma pack(push, 1)
struct FrameRecordHeader {
    char     magic[4];     // "FRM1"
    uint64_t frame_id;
    uint64_t dev_ts_ns;
    uint64_t host_ts_ns;
    uint32_t width;
    uint32_t height;
    uint32_t pixfmt;
    uint32_t size;
    uint32_t crc32;        // 图像数据的 CRC32
};
#pragma pack(pop)
static_assert(sizeof(FrameRecordHeader) == 48, "header layout");

struct RecorderStats {
    std::atomic<uint64_t> written{0};
    std::atomic<uint64_t> bytes{0};
    std::atomic<uint64_t> gaps{0};          // 帧号不连续次数
    std::atomic<uint64_t> missing{0};       // 估算缺失帧数
    std::atomic<uint64_t> max_write_us{0};  // 单帧最大写盘耗时
};

class Recorder {
public:
    Recorder(BoundedQueue<FramePtr>& q, std::string dir, uint64_t roll_bytes,
             int slow_disk_us = 0)
        : q_(q), dir_(std::move(dir)), roll_bytes_(roll_bytes), slow_us_(slow_disk_us) {}

    ~Recorder() { join(); }

    void start() { th_ = std::thread([this] { run(); }); }
    void join() {
        if (th_.joinable()) th_.join();
    }
    RecorderStats& stats() { return st_; }

private:
    void open_next() {
        if (fp_) std::fclose(fp_);
        char name[512];
        std::snprintf(name, sizeof(name), "%s/rec_%04d.bin", dir_.c_str(), file_idx_++);
        fp_ = std::fopen(name, "wb");
        if (!fp_) std::perror(name);
        // 大块缓冲减少 write 系统调用次数
        if (fp_) std::setvbuf(fp_, nullptr, _IOFBF, 4 << 20);
        cur_bytes_ = 0;
    }

    void run() {
        open_next();
        bool have_last = false;
        uint64_t last_id = 0;

        for (;;) {
            auto opt = q_.pop_for(std::chrono::milliseconds(200));
            if (!opt) {
                if (q_.closed() && q_.size() == 0) break;  // 收尾
                continue;                                  // 超时，继续等
            }
            FramePtr f = std::move(*opt);

            // 1) 帧号连续性检查（frame_id_missing 处理 16/32 位帧号绕回）
            if (have_last) {
                int64_t miss = frame_id_missing(last_id, f->frame_id);
                if (miss != 0) st_.gaps++;
                if (miss > 0) st_.missing += uint64_t(miss);
            }
            last_id = f->frame_id;
            have_last = true;

            // 2) 组织记录头
            FrameRecordHeader h{};
            std::memcpy(h.magic, "FRM1", 4);
            h.frame_id = f->frame_id;
            h.dev_ts_ns = f->dev_ts_ns;
            h.host_ts_ns = f->host_ts_ns;
            h.width = f->width;
            h.height = f->height;
            h.pixfmt = f->pixfmt;
            h.size = f->size;
            h.crc32 = crc32(f->data.data(), f->size);

            // 3) 写盘并计时（定位写盘瓶颈用）
            uint64_t t0 = host_now_ns();
            if (fp_) {
                std::fwrite(&h, sizeof(h), 1, fp_);
                std::fwrite(f->data.data(), 1, f->size, fp_);
            }
            if (slow_us_ > 0)  // 模拟慢磁盘，用于演示背压和丢帧
                std::this_thread::sleep_for(std::chrono::microseconds(slow_us_));
            uint64_t us = (host_now_ns() - t0) / 1000;
            uint64_t prev = st_.max_write_us.load();
            while (us > prev && !st_.max_write_us.compare_exchange_weak(prev, us)) {}

            st_.written++;
            st_.bytes += sizeof(h) + f->size;
            cur_bytes_ += sizeof(h) + f->size;

            // 4) 滚动文件
            if (cur_bytes_ >= roll_bytes_) open_next();
            // f 在这里析构，内存自动还回 FramePool
        }
        if (fp_) {
            std::fflush(fp_);
            std::fclose(fp_);
            fp_ = nullptr;
        }
    }

    BoundedQueue<FramePtr>& q_;
    std::string dir_;
    uint64_t roll_bytes_;
    int slow_us_;
    std::thread th_;
    std::FILE* fp_ = nullptr;
    int file_idx_ = 0;
    uint64_t cur_bytes_ = 0;
    RecorderStats st_;
};

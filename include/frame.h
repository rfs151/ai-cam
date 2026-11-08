// 帧数据结构 + 帧内存池 + CRC32 + 时间工具
#pragma once
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

// 像素格式（取 GenICam PFNC 编码的几个常用值，方便和真实相机对齐）
enum PixelFormat : uint32_t {
    PIX_MONO8     = 0x01080001,
    PIX_BAYERRG8  = 0x01080009,
    PIX_RGB8      = 0x02180014,
};

inline uint64_t host_now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

// 一帧图像：数据 + 元数据（帧号、相机时间戳、主机接收时间戳）
struct Frame {
    uint64_t frame_id   = 0;   // 相机帧号（用于检测丢帧）
    uint64_t dev_ts_ns  = 0;   // 相机硬件时间戳（PTP 同步后可跨设备对齐）
    uint64_t host_ts_ns = 0;   // 主机收到帧的时间（用于算传输延迟）
    uint32_t width = 0, height = 0;
    uint32_t pixfmt = PIX_MONO8;
    uint32_t size = 0;         // 有效字节数
    std::vector<uint8_t> data; // 预分配，复用
};

// 帧内存池：避免高帧率下反复 new/delete 大块内存
// acquire() 拿到的 shared_ptr 析构时自动归还池子
class FramePool : public std::enable_shared_from_this<FramePool> {
public:
    static std::shared_ptr<FramePool> create(size_t count, size_t bytes) {
        auto p = std::shared_ptr<FramePool>(new FramePool());
        for (size_t i = 0; i < count; ++i) {
            auto f = std::make_unique<Frame>();
            f->data.resize(bytes);
            p->free_.push_back(std::move(f));
        }
        return p;
    }

    // 池空返回 nullptr（说明下游积压，调用方按丢帧处理）
    std::shared_ptr<Frame> acquire() {
        std::unique_ptr<Frame> f;
        {
            std::lock_guard<std::mutex> lk(m_);
            if (free_.empty()) return nullptr;
            f = std::move(free_.back());
            free_.pop_back();
        }
        std::weak_ptr<FramePool> wp = shared_from_this();
        return std::shared_ptr<Frame>(f.release(), [wp](Frame* fr) {
            if (auto pool = wp.lock()) {
                std::lock_guard<std::mutex> lk(pool->m_);
                pool->free_.emplace_back(fr);
            } else {
                delete fr;
            }
        });
    }

    size_t available() {
        std::lock_guard<std::mutex> lk(m_);
        return free_.size();
    }

private:
    FramePool() = default;
    std::mutex m_;
    std::vector<std::unique_ptr<Frame>> free_;
};

using FramePtr = std::shared_ptr<Frame>;

// 计算 prev 之后到 cur 之间缺了几帧，处理帧号绕回。
//   返回 0：连续（包括正常绕回）；>0：缺了这么多帧；-1：帧号倒退但不像绕回（如相机重启后重新计数）
// 绕回规则：
//   16 位：GigE Vision 1.x 的 block ID，1..65535，绕回后从 1 开始（0 保留不用）
//   32 位：如海康 nFrameNum，0..0xFFFFFFFF，绕回后从 0 开始
// 只有 prev 接近上限、cur 接近起点时才认为是绕回，容差内允许绕回前后有丢帧。
inline int64_t frame_id_missing(uint64_t prev, uint64_t cur) {
    if (cur > prev) return int64_t(cur - prev - 1);
    const uint64_t TOL = 1000;  // 绕回前后最多容忍丢这么多帧仍判为绕回
    if (prev <= 0xFFFFu && prev + TOL >= 0xFFFFu && cur <= TOL)
        return int64_t((0xFFFFu - prev) + (cur >= 1 ? cur - 1 : 0));
    if (prev <= 0xFFFFFFFFull && prev + TOL >= 0xFFFFFFFFull && cur <= TOL)
        return int64_t((0xFFFFFFFFull - prev) + cur);
    return -1;
}

// 标准 CRC32（与 Python zlib.crc32 一致），用于写盘数据校验
inline uint32_t crc32(const uint8_t* buf, size_t len, uint32_t crc = 0) {
    static uint32_t table[256];
    static bool inited = [] {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        return true;
    }();
    (void)inited;
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) crc = table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

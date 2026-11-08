// 有界线程安全队列：采集线程 try_push（永不阻塞），写盘线程 pop_for（带超时）
// 设计要点：
//   1. 采集线程绝不能因为下游慢而阻塞，否则相机端缓存溢出、整批丢帧，
//      所以满了直接返回 false，由调用方计数丢帧（策略可换成"丢最旧"）。
//   2. close() 之后消费者仍能把剩余数据取完（排空），再退出。
#pragma once
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(size_t capacity) : cap_(capacity) {}

    BoundedQueue(const BoundedQueue&) = delete;
    BoundedQueue& operator=(const BoundedQueue&) = delete;

    // 非阻塞入队；队列已满或已关闭时返回 false
    bool try_push(T&& v) {
        {
            std::lock_guard<std::mutex> lk(m_);
            if (closed_ || q_.size() >= cap_) return false;
            q_.push_back(std::move(v));
        }
        cv_.notify_one();
        return true;
    }

    // 带超时出队；超时或"已关闭且已排空"时返回 std::nullopt
    template <class Rep, class Period>
    std::optional<T> pop_for(std::chrono::duration<Rep, Period> timeout) {
        std::unique_lock<std::mutex> lk(m_);
        if (!cv_.wait_for(lk, timeout, [&] { return closed_ || !q_.empty(); }))
            return std::nullopt;            // 超时
        if (q_.empty()) return std::nullopt; // 已关闭且排空
        T v = std::move(q_.front());
        q_.pop_front();
        return v;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lk(m_);
            closed_ = true;
        }
        cv_.notify_all();
    }

    bool closed() const {
        std::lock_guard<std::mutex> lk(m_);
        return closed_;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.size();
    }

    size_t capacity() const { return cap_; }

private:
    const size_t cap_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::deque<T> q_;
    bool closed_ = false;
};

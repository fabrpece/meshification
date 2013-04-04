#pragma once
#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

class AsyncOperation
{
    std::function<void()> process;
    std::mutex m;
    using lock = std::unique_lock<std::mutex>;
    std::condition_variable c_ready, c_finished;
    std::atomic<bool> is_running;
    std::thread t;

    void run();

public:
    AsyncOperation();
    ~AsyncOperation();
    void begin(const std::function<void()>&  f);
    void end();
};

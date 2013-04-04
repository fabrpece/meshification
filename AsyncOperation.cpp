#include <sstream>
#include "AsyncOperation.hpp"
#include "VideoEncoder.hpp"

AsyncOperation::AsyncOperation() :
    t(&AsyncOperation::run, this)
{
}

AsyncOperation::~AsyncOperation()
{
    is_running.store(false);
    c_ready.notify_all();
    t.join();
}

void AsyncOperation::begin(const std::function<void()> &f)
{
    lock l(m);
    process = f;
    c_ready.notify_all();
}

void AsyncOperation::end()
{
    lock l(m);
    while (process != 0)
        c_finished.wait(l);
}

void AsyncOperation::run()
{
    is_running.store(true);
    while (true) {
        lock l(m);
        while (process == 0 && is_running)
            c_ready.wait(l);
        if (process == 0)
            break;
        process();
        process = 0;
        c_finished.notify_all();
    }
}

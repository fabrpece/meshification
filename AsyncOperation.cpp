/*
    Copyright (C) 2011-2012 Paolo Simone Gasparello <p.gasparello@sssup.it>

    This file is part of meshificator.

    meshificator is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    meshificator is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with meshificator.  If not, see <http://www.gnu.org/licenses/>.
*/

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

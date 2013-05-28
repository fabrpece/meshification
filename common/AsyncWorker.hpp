#pragma once

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

#include <string>
#include <functional>
#include <boost/thread.hpp>

typedef boost::unique_lock<boost::mutex> lock;

class AsyncWorker
{
    std::function<void()> operation;
    boost::mutex m;
    boost::condition_variable c_ready, c_finished;
    bool is_running;
    boost::thread t;

    void run();

public:
    AsyncWorker();
    ~AsyncWorker();
    void begin(const std::function<void()>&  f);
    void end();
};

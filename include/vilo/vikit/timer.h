// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

namespace vilo
{

class Timer
{
private:
    timeval start_time_;
    double time_;
    double accumulated_;
public:

    /// The constructor directly starts the timer.
    Timer() : time_(0), accumulated_(0)
    {
        start();
    }

    ~Timer()
    {}

    inline void start()
    {
        accumulated_ = 0.0;
        gettimeofday(&start_time_, NULL);
    }

    inline void resume()
    {
        gettimeofday(&start_time_, NULL);
    }

    inline double stop()
    {
        timeval end_time;
        gettimeofday(&end_time, NULL);
        long seconds  = end_time.tv_sec  - start_time_.tv_sec;
        long useconds = end_time.tv_usec - start_time_.tv_usec;
        time_ = ((seconds) + useconds*0.000001) + accumulated_;
        accumulated_ = time_;
        return time_;
    }

    inline double getTime() const
    {
        return time_;
    }

    inline void reset()
    {
        time_ = 0;
        accumulated_ = 0;
    }

    static double getCurrentTime()
    {
        timeval time_now;
        gettimeofday(&time_now, NULL);
        return time_now.tv_sec + time_now.tv_usec*0.000001;
    }

    static double getCurrentSecond()
    {
        timeval time_now;
        gettimeofday(&time_now, NULL);
        return time_now.tv_sec;
    }

};

} // end namespace vk


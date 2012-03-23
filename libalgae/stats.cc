/*
 *  Copyright 2010, 2011, 2012 Adam Sampson
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. Neither the name of the CoSMoS Project nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "algae.h"
#include "stats.h"

#include <iostream>
#include <time.h>
#include <unistd.h>

using namespace algae;

static const Time OUTPUT_PERIOD = 5.0;

/*{{{  Stats::Stats */
Stats::Stats() {
    Time now = get_time();

    last_frame_time_ = now;
    last_output_time_ = now;
    last_output_frame_ = 0;

    /*{{{  check whether stderr is a terminal */
    if (isatty(2)) {
        highlight_output_ = true;
    } else {
        highlight_output_ = false;
    }
    /*}}}*/
}
/*}}}*/
/*{{{  Stats::~Stats */
Stats::~Stats() {
    show_stats(get_time(), true);
}
/*}}}*/
/*{{{  Stats::start_frame */
void Stats::start_frame() {
    /*{{{  add time to list */
    Time now = get_time();
    frame_times_.push_back(now - last_frame_time_);
    last_frame_time_ = now;
    /*}}}*/

    /*{{{  show stats periodically */
    if ((now - last_output_time_) > OUTPUT_PERIOD) {
        show_stats(now, false);
    }
    /*}}}*/
}
/*}}}*/
/*{{{  Stats::show_stats */
void Stats::show_stats(Time now, bool final) {
    int frame = frame_times_.size();

    Time all_mean = mean_time();

    /*{{{  enable highlighting */
    if (highlight_output_) {
        // XXX: this should really use terminfo
        std::cerr << "\033[33m";
    }
    /*}}}*/

    std::cerr << "[stats] frame=" << frame
              << " time=" << now
              << " all-mean=" << all_mean
              << " all-fps=" << (1.0 / all_mean);

    if (!final) {
        Time recent_mean = mean_time(last_output_frame_);
        std::cerr << " recent-mean=" << recent_mean
                  << " recent-fps=" << (1.0 / recent_mean);
    }

    /*{{{  disable highlighting */
    if (highlight_output_) {
        std::cerr << "\033[0m";
    }
    /*}}}*/

    std::cerr << std::endl;

    last_output_frame_ = frame;
    last_output_time_ = now;
}
/*}}}*/
/*{{{  Stats::get_time */
Time Stats::get_time() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + (ts.tv_nsec * 1.0e-9);
}
/*}}}*/
/*{{{  Stats::mean_time */
Time Stats::mean_time(int start) {
    Time total = 0.0;
    int count = 0;
    for (TimeList::iterator it = frame_times_.begin() + start; it != frame_times_.end(); ++it) {
        total += *it;
        ++count;
    }

    return total / count;
}
/*}}}*/

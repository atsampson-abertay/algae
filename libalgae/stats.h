/** Collect performance statistics. */

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

#ifndef STATS_H
#define STATS_H

#include "algae.h"

#include <boost/shared_ptr.hpp>
#include <vector>

namespace algae {

/** A timestamp or time delta in seconds. */
typedef float Time;

/*{{{  class Sample */
/** A sample, whatever that is. */
class Sample {
public:
    Sample(Time _time, int _frames) : time(_time), frames(_frames) {}

    Time time;
    int frames;
};
/*}}}*/

/*{{{  class Stats */
class Stats {
public:
    Stats();
    ~Stats();

    void start_frame();
    void show_stats(Time now, bool final);

protected:
    typedef boost::shared_ptr<Sample> SamplePtr;
    typedef std::vector<SamplePtr> SampleList;

    /** Return the current wall-clock timestamp. */
    Time get_time();

    /** Return the mean frame time across a series of samples. */
    Time mean_time(int start = 0);

    int frame_;
    int period_;

    SampleList samples_;

    Time last_sample_time_;
    int last_sample_frame_;

    Time last_output_time_;
    int last_output_sample_;

    bool highlight_output_;
};
/*}}}*/

}

#endif

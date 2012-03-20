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

#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

using namespace algae;

static boost::scoped_ptr<Viewer> viewer;
static boost::scoped_ptr<boost::thread> thread;
static FramePtr frame;

/*{{{  run_viewer */
static void run_viewer() {
    viewer->run();
}
/*}}}*/

/*{{{  algae_start */
void algae_start() {
    viewer.reset(new Viewer());
    thread.reset(new boost::thread(run_viewer));
}
/*}}}*/
/*{{{  algae_stop */
void algae_stop() {
    viewer.reset();
}
/*}}}*/
/*{{{  algae_frame_begin */
void algae_frame_begin(int *send) {
    if (send != NULL) {
        *send = 1;
    }
    frame = viewer->new_frame();
}
/*}}}*/
/*{{{  algae_frame_add */
void algae_frame_add(float x, float y, float z, float radius) {
    Object obj(x, y, z, radius);
    frame->add(obj);
}
/*}}}*/
/*{{{  algae_frame_end */
void algae_frame_end() {
    frame->end();
}
/*}}}*/

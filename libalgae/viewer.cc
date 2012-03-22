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
#include "display.h"

#include <boost/make_shared.hpp>
#include <cstdlib>
#include <iostream>

using namespace algae;

/*{{{  Viewer::Viewer */
Viewer::Viewer()
    : display_(new Display) {
}
/*}}}*/
/*{{{  Viewer::~Viewer */
Viewer::~Viewer() {
    // Subtle: this destructor doesn't do anything, but it's necessary in order
    // for display_ to be destroyed in a context where the full (not forward)
    // declaration of Display is available.
}
/*}}}*/
/*{{{  Viewer::new_frame */
FramePtr Viewer::new_frame() {
    if (display_) {
        return boost::make_shared<Frame>(*this);
    } else {
        // No frame needed.
        return FramePtr();
    }
}
/*}}}*/
/*{{{  Viewer::run */
void Viewer::run() {
    if (display_) {
        display_->run();
    }
}
/*}}}*/
/*{{{  Viewer::update */
void Viewer::update() {
    if (display_) {
        display_->update();
    }
}
/*}}}*/
/*{{{  Viewer::commit_frame */
void Viewer::commit_frame(FramePtr frame) {
    if (display_) {
        display_->add_frame(frame);
    }
}
/*}}}*/

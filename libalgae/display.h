/** The SDL/OpenGL display. */

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

#ifndef DISPLAY_H
#define DISPLAY_H

#include "algae.h"
#include "colour.h"

// The include ordering here is important: we need to pull in glew.h before
// gl.h, and stop SDL redefining the glext.h stuff.
#include <SDL.h>
#include <GL/glew.h>
#define NO_SDL_GLEXT
#include <SDL_opengl.h>

#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>
#include <list>

namespace algae {

class Display {
public:
    Display();

    void add_frame(FramePtr frame);
    void run();
    void update();

protected:
    typedef boost::unordered_map<int, Colour> PaletteMap;

    void reset_display();
    void init_display();
    void handle_rotate(float x, float y, float z, bool shift);
    void handle_event(SDL_Event& event);
    void draw();
    void draw_objects();
    void draw_text();

    int display_width_, display_height_;
    bool display_text_;
    SDL_Surface *window_;

    // XXX: this should be atomic
    boost::mutex need_redraw_mutex_;
    bool need_redraw_;
    /*{{{  need_redraw accessor/mutator */
    void need_redraw(bool value) {
        boost::mutex::scoped_lock guard(need_redraw_mutex_);

        need_redraw_ = value;
    }
    bool need_redraw() {
        boost::mutex::scoped_lock guard(need_redraw_mutex_);

        return need_redraw_;
    }
    /*}}}*/

    float zoom_;
    Vec3 rotate_, rotate_delta_;

    FramePtr latest_frame_;
    boost::mutex latest_frame_mutex_;
};

}

#endif

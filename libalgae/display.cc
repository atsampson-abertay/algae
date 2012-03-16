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
#include "group.h"

#include <boost/foreach.hpp>
#include <cstdlib>
#include <iostream>

using namespace algae;

/*{{{  Display::Display */
Display::Display()
    : display_width_(800), display_height_(600), display_text_(true) {

    /*{{{  initialise SDL */
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        exit(1);
    }
    atexit(SDL_Quit);

    SDL_WM_SetCaption("Algae", "Algae");

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);

    window_ = SDL_SetVideoMode(display_width_, display_height_, 0, SDL_OPENGL);

    SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);
    /*}}}*/

    /*{{{  initialise GLEW */
    if (glewInit() != GLEW_OK) {
        std::cerr << "glewInit() failed" << std::endl;
        exit(1);
    }
    /*}}}*/
}
/*}}}*/
/*{{{  Display::add_frame */
void Display::add_frame(FramePtr frame) {
    frames_.push_back(frame);
}
/*}}}*/
/*{{{  Display::update */
void Display::update() {
    /*{{{  check for events */
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        handle_event(event);
    }
    /*}}}*/

    glViewport(0, 0, display_width_, display_height_);

    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (1.0 * display_width_) / display_height_, 0.1, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();

    draw_objects();

    /*{{{  map to screen coordinates */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, display_width_, 0, display_height_);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.375, 0.375, 0.0);
    /*}}}*/

    draw_text();

    SDL_GL_SwapBuffers();

}
/*}}}*/
/*{{{  Display::handle_event */
void Display::handle_event(SDL_Event& event) {
    switch (event.type) {
    case SDL_QUIT:
        exit(0);
        break;
    case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {
        case SDLK_t:
            display_text_ = !display_text_;
            break;
        default:
            break;
        }
        break;
    }
}
/*}}}*/
/*{{{  Display::draw_objects */
void Display::draw_objects() {
    /*{{{  print objects */
    const Frame& frame(*frames_.back());
    BOOST_FOREACH (const GroupMap::value_type& item, frame.groups()) {
        const Group& group(*item.second);
        std::cout << "Group " << item.first << std::endl;
        BOOST_FOREACH (const Object& obj, group.objects()) {
            std::cout << "  Object pos=" << obj.pos << std::endl;
        }
        std::cout << "  min=" << group.min_pos() << std::endl;
        std::cout << "  max=" << group.max_pos() << std::endl;
        std::cout << std::endl;
    }
    /*}}}*/
}
/*}}}*/
/*{{{  Display::draw_text */
void Display::draw_text() {
    if (!display_text_) {
        return;
    }
}
/*}}}*/

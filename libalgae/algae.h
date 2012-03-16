/** Algae: a library for visualising collections of objects in 3D space. */

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

#ifndef ALGAE_H
#define ALGAE_H

#include <boost/shared_ptr.hpp>
#include <vector>

namespace algae {

/*{{{  forward declarations */
class Frame;
class Object;
class Viewer;
/*}}}*/

/*{{{  typedefs */
typedef boost::shared_ptr<Frame> FramePtr;
/*}}}*/

/*{{{  class Object */
class Object {
public:
    Object(float _x, float _y, float _z, float _radius)
        : x(_x), y(_y), z(_z), radius(_radius) {}

    float x, y, z;
    float radius;
};
/*}}}*/

/*{{{  class Viewer */
class Viewer {
    friend class Frame;

public:
    Viewer();

    FramePtr add_frame();
    void run();

protected:
    void commit_frame(Frame& frame);

    std::vector<FramePtr> frames_;
};
/*}}}*/

/*{{{  class Frame */
class Frame {
public:
    Frame(Viewer& viewer)
        : viewer_(viewer) {}

    Object& add(float x = 0.0, float y = 0.0, float z = 0.0,
                float radius = 0.0) {
        objects_.push_back(Object(x, y, z, radius));
        return objects_.back();
    }

    void end() {
        viewer_.commit_frame(*this);
    }

private:
    Viewer& viewer_;
    std::vector<Object> objects_;
};
/*}}}*/

}

#endif

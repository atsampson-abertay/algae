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

#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <vector>

namespace algae {

/*{{{  forward declarations */
class Display;
class Frame;
class Group;
class Object;
class Vec3;
class Viewer;
/*}}}*/

/*{{{  typedefs */
typedef boost::shared_ptr<Frame> FramePtr;
typedef boost::shared_ptr<Group> GroupPtr;
/*}}}*/

/*{{{  class Vec3 */
class Vec3 {
public:
    Vec3(float _x = 0.0, float _y = 0.0, float _z = 0.0)
        : x(_x), y(_y), z(_z) {}

    float x, y, z;
};
/*}}}*/

/*{{{  class Object */
class Object {
public:
    Object(float x = 0.0, float y = 0.0, float z = 0.0, float _radius = 0.0)
        : pos(x, y, z), radius(_radius) {}

    Vec3 pos;
    float radius;
};
/*}}}*/

/*{{{  class Viewer */
class Viewer {
    friend class Frame;

public:
    Viewer();
    ~Viewer();

    FramePtr new_frame();

    void run();
    void run_once();

protected:
    void commit_frame(FramePtr frame);

    boost::scoped_ptr<Display> display_;
};
/*}}}*/

/*{{{  class Frame */
class Frame : public boost::enable_shared_from_this<Frame> {
public:
    Frame(Viewer& viewer);

    void group(int num);
    Object& add(const Object& obj);
    void end();

protected:
    // groups_ is a map from group numbers to Groups.
    // If there is no entry in the map, that group was never selected,
    // and should stay the same as it was on the last frame.
    typedef boost::unordered_map<int, GroupPtr> GroupMap;
    typedef GroupMap::value_type GroupMapItem;

    Viewer& viewer_;
    GroupMap groups_;
    GroupPtr current_group_;
};
/*}}}*/

}

#endif

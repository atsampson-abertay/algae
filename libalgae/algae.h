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

#ifdef __cplusplus
#define EXTERN_C extern "C"

/*{{{  C++ interface */
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <iostream>
#include <vector>

namespace algae {

/*{{{  forward declarations */
class Display;
class Frame;
class Group;
class Object;
class Stats;
class Vec3;
class Viewer;
/*}}}*/

/*{{{  typedefs */
typedef boost::shared_ptr<Frame> FramePtr;
typedef boost::shared_ptr<Group> GroupPtr;
typedef boost::unordered_map<int, GroupPtr> GroupMap;
typedef std::vector<Object> ObjectList;
/*}}}*/

/*{{{  class Vec3 */
class Vec3 {
public:
    Vec3(float _x = 0.0, float _y = 0.0, float _z = 0.0)
        : x(_x), y(_y), z(_z) {}

    /*{{{  operator+= */
    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    /*}}}*/
    /*{{{  operator-= */
    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    /*}}}*/
    /*{{{  operator*= */
    Vec3& operator*=(float v) {
        x *= v;
        y *= v;
        z *= v;
        return *this;
    }
    /*}}}*/
    /*{{{  operator/= */
    Vec3& operator/=(float v) {
        x /= v;
        y /= v;
        z /= v;
        return *this;
    }
    /*}}}*/
    /*{{{  operator+ */
    Vec3 operator+(const Vec3& other) {
        Vec3 result(*this);
        result += other;
        return result;
    }
    /*}}}*/
    /*{{{  operator- */
    Vec3 operator-(const Vec3& other) {
        Vec3 result(*this);
        result -= other;
        return result;
    }
    /*}}}*/
    /*{{{  operator* */
    Vec3 operator*(const float v) {
        Vec3 result(*this);
        result *= v;
        return result;
    }
    /*}}}*/
    /*{{{  operator/ */
    Vec3 operator/(const float v) {
        Vec3 result(*this);
        result /= v;
        return result;
    }
    /*}}}*/
    /*{{{  to_min */
    void to_min(const Vec3& other) {
        if (other.x < x) x = other.x;
        if (other.y < y) y = other.y;
        if (other.z < z) z = other.z;
    }
    /*}}}*/
    /*{{{  to_max */
    void to_max(const Vec3& other) {
        if (other.x > x) x = other.x;
        if (other.y > y) y = other.y;
        if (other.z > z) z = other.z;
    }
    /*}}}*/

    float x, y, z;
};
/*{{{  ostream operator<< */
template <class charT, class traits>
inline std::basic_ostream<charT, traits>&
    operator<<(std::basic_ostream<charT, traits>& os, const Vec3& vec) {
    return os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
}
/*}}}*/
/*}}}*/

/*{{{  class Object */
class Object {
public:
    Object(float x = 0.0, float y = 0.0, float z = 0.0, float _radius = 0.0, int _col = 0)
        : pos(x, y, z), radius(_radius), col(_col) {}

    Vec3 pos;
    float radius;
    int col;
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
    void update();

protected:
    void commit_frame(FramePtr frame);

    boost::scoped_ptr<Display> display_;
    boost::scoped_ptr<Stats> stats_;
};
/*}}}*/

/*{{{  class Frame */
class Frame : public boost::enable_shared_from_this<Frame> {
public:
    Frame(Viewer& viewer);

    void group(int num);
    void add(const Object& obj);
    void end();

    const GroupMap& groups() const {
        return groups_;
    }

protected:
    Viewer& viewer_;
    // A map from group numbers to Groups.
    // If there is no entry in the map, that group was never selected,
    // and should stay the same as it was on the last frame.
    GroupMap groups_;
    GroupPtr current_group_;
};
/*}}}*/

}
/*}}}*/

#else
#define EXTERN_C
#endif

/*{{{  C interface  */
EXTERN_C void algae_start(void);
EXTERN_C void algae_stop(void);
EXTERN_C void algae_frame_begin(int *send);
EXTERN_C void algae_frame_add(float x, float y, float z, float radius, int col);
EXTERN_C void algae_frame_end(void);
/*}}}*/

#undef EXTERN_C

#endif

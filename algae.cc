/** Standalone viewer tool. */

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

#include <boost/program_options.hpp>
#include <cmath>
#include <iostream>

namespace opts = boost::program_options;

/*{{{  main */
extern "C" int main(int argc, char *argv[]) {
    /*{{{  parse command-line options */
    opts::options_description desc("Options");
#define simple(Type, Name, Default) opts::value<Type>(&config.Name)->default_value(Default)
    desc.add_options()
        ("help", "display this help and exit")
        ;
#undef simple

    opts::variables_map vars;
    opts::store(opts::parse_command_line(argc, argv, desc), vars);
    opts::notify(vars);

    if (vars.count("help")) {
        /*{{{  show help */
        std::cout << desc << std::endl;
        exit(0);
        /*}}}*/
    }
    /*}}}*/

    algae::Viewer viewer;

    algae::FramePtr frame = viewer.new_frame();
#if 0
    for (float f = -10.0; f < 10.0; f += 0.1) {
        frame->add(algae::Object(10.0 * sinf(f), 10.0 * cosf(f), f));
    }
#else
    for (float z = -10.0; z < 10.0; z += 1.0) {
        for (float y = -10.0; y < 10.0; y += 1.0) {
            for (float x = -10.0; x < 10.0; x += 1.0) {
                int col = int(x + 10) / 4;
                frame->add(algae::Object(x, y, z, 0.5, col));
            }
        }
    }
#endif
    frame->end();

    viewer.run();

    return 0;
}
/*}}}*/

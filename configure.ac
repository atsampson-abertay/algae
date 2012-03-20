AC_PREREQ(2.59)
AC_INIT(algae, 1.0, ats@offog.org)
AM_INIT_AUTOMAKE([foreign])

AC_PROG_CC
AC_PROG_CXX
AC_PROG_LIBTOOL
AC_LANG([C++])

use_sdl_display=true
use_gl_display=true

AX_CHECK_GL
CPPFLAGS="$GL_CFLAGS $CPPFLAGS"
LIBS="$GL_LIBS $LIBS"
if test "x$no_gl" = "xyes" -o "x$no_glu" = "xyes"; then
	AC_MSG_ERROR(GL is required)
fi

AX_CHECK_GLU
CPPFLAGS="$GLU_CFLAGS $CPPFLAGS"
LIBS="$GLU_LIBS $LIBS"
if test "x$no_glu" = "xyes"; then
	AC_MSG_ERROR(GLU is required)
fi

AC_SEARCH_LIBS([glewInit], [GLEW glew32], [], [AC_MSG_ERROR(GLEW is required)])

AX_BOOST_BASE([], [], [AC_MSG_ERROR(Boost is required)])
AX_BOOST_PROGRAM_OPTIONS
AX_BOOST_THREAD
CPPFLAGS="$BOOST_CPPFLAGS $CPPFLAGS"
LIBS="$BOOST_LDFLAGS $BOOST_PROGRAM_OPTIONS_LIB $BOOST_THREAD_LIB $LIBS"

# The test for SDL must go last, since the SDLmain hack will cause link
# errors with other libraries on MinGW.
AM_PATH_SDL([1.2.0], [],
            [AC_MSG_ERROR(SDL is required)])
CPPFLAGS="$SDL_CFLAGS $CPPFLAGS"
LIBS="$SDL_LIBS $LIBS"

CXXFLAGS="-std=c++0x -Wall $CXXFLAGS"

AC_CONFIG_FILES([Makefile])
AC_OUTPUT

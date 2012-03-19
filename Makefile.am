ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libalgae.la

libalgae_la_SOURCES = \
	libalgae/binding-c.cc \
	libalgae/colour.cc \
	libalgae/colour.h \
	libalgae/display.cc \
	libalgae/display.h \
	libalgae/frame.cc \
	libalgae/group.cc \
	libalgae/group.h \
	libalgae/viewer.cc

include_HEADERS = \
	libalgae/algae.h

bin_PROGRAMS = algae

algae_SOURCES = \
	algae.cc

algae_CPPFLAGS = \
	-I$(srcdir)/libalgae

algae_LDADD = \
	libalgae.la

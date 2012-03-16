ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libalgae.la

libalgae_la_SOURCES = \
	libalgae/display.cc \
	libalgae/display.h \
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

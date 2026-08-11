# By default, SoapySDR support is enabled.
# `USE_SOAPY=0 make` to disable SoapySDR interface but still with access to MMDVM-Multi.
USE_SOAPY ?= 1

#
# To compile with a SoapySDR interface and access to MMDVM-Multi then use the following lines
#
CXX      ?= c++
CXXFLAGS += -g -O3 -Wall -std=c++11 -fpermissive -MMD -MD -pthread -DARM_MATH_RPI -Wno-narrowing -Wno-strict-aliasing
LIBS     += -lpthread -lmosquitto
LDFLAGS  += -g -L/usr/local/lib

ifeq ($(USE_SOAPY), 1)
	CXXFLAGS+= -DUSE_SOAPY=1
	LIBS+= -lSoapySDR
endif

ifeq ($(shell uname -s),Darwin)
	CFLAGS+= -I/opt/homebrew/include -Wno-c++11-narrowing
	CXXFLAGS+= -I/opt/homebrew/include -Wno-c++11-narrowing
	LDFLAGS+= -L/opt/homebrew/lib
endif

CXXSRCS = $(wildcard *.cpp)
CXXDEPS = $(CXXSRCS:.cpp=.d)
CXXOBJS = $(CXXSRCS:.cpp=.o)

all:		MMDVM-IQ

MMDVM-IQ:	$(CXXOBJS)
		$(CXX) $(CXXOBJS) $(LDFLAGS) $(LIBS) -o MMDVM-IQ

%.o: %.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<
-include $(CXXDEPS)

MMDVM-IQ.o: GitVersion.h FORCE
SerialPort.o: GitVersion.h FORCE

.PHONY: GitVersion.h

FORCE:

install:
		install -m 755 MMDVM-IQ /usr/local/bin/
		install -m 644 MMDVM-IQ.ini /etc

clean:
		$(RM) MMDVM-IQ *.o *.d *.bak *~ GitVersion.h

# Export the current git version if the index file exists, else 000...
GitVersion.h:
ifneq ("$(wildcard .git/index)","")
	echo "const char *gitversion = \"$(shell git rev-parse HEAD)\";" > $@
else
	echo "const char *gitversion = \"0000000000000000000000000000000000000000\";" > $@
endif

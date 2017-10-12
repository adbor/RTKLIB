cmake_minimum_required(VERSION 3.2)

set( PROJ_NAME rtkrcv )
project (${PROJ_NAME})

add_definitions( -DENAGLO -DENAQZS -DENACMP -DENAGAL -DENAIRN -DNFREQ=3 -DSVR_REUSEADDR )

include_directories( ../../../src )

###########################################################
add_executable(rtkrcv
  ../rtkrcv.c 
  ../vt.c 
  ../../../src/rtkcmn.c 
  ../../../src/rtksvr.c 
  ../../../src/rtkpos.c 
  ../../../src/geoid.c 
  ../../../src/solution.c 
  ../../../src/lambda.c 
  ../../../src/sbas.c 
  ../../../src/stream.c 
  ../../../src/rcvraw.c 
  ../../../src/rtcm.c 
  ../../../src/preceph.c 
  ../../../src/options.c 
  ../../../src/pntpos.c 
  ../../../src/ppp.c 
  ../../../src/ppp_ar.c
  ../../../src/ephemeris.c 
  ../../../src/rinex.c 
  ../../../src/ionex.c 
  ../../../src/rtcm2.c 
  ../../../src/rtcm3.c 
  ../../../src/rtcm3e.c 
  ../../../src/qzslex.c 
  ../../../src/ppp_corr.c 
  ../../../src/tides.c 

  ../../../src/rcv/septentrio.c 
  ../../../src/rcv/cmr.c 
  ../../../src/rcv/tersus.c
  ../../../src/rcv/novatel.c 
  ../../../src/rcv/ublox.c 
  ../../../src/rcv/ss2.c 
  ../../../src/rcv/crescent.c 
  ../../../src/rcv/skytraq.c 
  ../../../src/rcv/gw10.c 
  ../../../src/rcv/javad.c 
  ../../../src/rcv/nvs.c 
  ../../../src/rcv/binex.c 
  ../../../src/rcv/rt17.c 
)

target_link_libraries(rtkrcv Winmm)

#CFLAGS = -Wall -O3 -ansi -pedantic -Wno-unused-but-set-variable -I$(SRC) -I.. -DTRACE $(CTARGET) -g
#LDLIBS  = -lm -lrt -lpthread

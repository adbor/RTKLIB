set( proj_name rtklib)

add_library( ${proj_name}
convrnx.c
rtklib.h
rtkcmn.c
rtksvr.c
rtkpos.c
geoid.c
solution.c
lambda.c
sbas.c
stream.c
rcvraw.c
rtcm.c
preceph.c
options.c
pntpos.c
postpos.c
ppp.c
ppp_ar.c
ephemeris.c
rinex.c
ionex.c
rtcm2.c
rtcm3.c
rtcm3e.c
qzslex.c
rcv/novatel.c
rcv/ublox.c
rcv/cmr.c
rcv/ss2.c
rcv/crescent.c
rcv/skytraq.c
rcv/gw10.c
rcv/javad.c
rcv/nvs.c
rcv/binex.c
rcv/rt17.c
rcv/tersus.c
rcv/septentrio.c
)


target_compile_definitions( ${proj_name} PUBLIC -DNFREQ=3 -DENAGLO -DENAGAL -DENAQZS -DTRACE )

target_include_directories( ${proj_name} PUBLIC ${CMAKE_CURRENT_LIST_DIR} )


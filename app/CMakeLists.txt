cmake_minimum_required(VERSION 2.8)

set( PROJ_NAME rtklibApps )
project (${PROJ_NAME})

add_subdirectory(./../src "${BUILD_DIR}/rtklib")

#should be WIN32 specific
if(WIN32)
  set (COMMON_LIBS Winmm ws2_32 rtklib)
endif()

add_executable(convbin
  convbin/convbin.c 
)
target_link_libraries(convbin rtklib ${COMMON_LIBS})



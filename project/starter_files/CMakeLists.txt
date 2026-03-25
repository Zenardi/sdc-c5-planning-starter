project(MotionPlanning)

cmake_minimum_required (VERSION 3.10)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++17 -O3)

set(CXX_FLAGS "-pthread -fPIC -DNDEBUG -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS} -Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic")

set(sources main.cpp behavior_planner_FSM.cpp motion_planner.cpp cubic_spiral.cpp spiral_base.cpp integral.cpp spiral_equations.cpp cost_functions.cpp utils.cpp velocity_profile_generator.cpp )

# Use pre-built CARLA client library from libcarla-install/ (symlinked from SDC C3 project).
# All CARLA static libs and headers live under a single flat directory.
set(CARLA_INSTALL_DIR ${CMAKE_SOURCE_DIR}/libcarla-install)
set(CARLA_INSTALL_LIB_DIR ${CARLA_INSTALL_DIR}/lib)
set(CARLA_INSTALL_INC_DIR ${CARLA_INSTALL_DIR}/include)

set(LIBUV_INCLUDE_DIR /usr/include/uv.h)
set(LIBUV_LIBRARY /usr/lib/x86_64-linux-gnu/libuv.so)

set(carla_libs
  ${CARLA_INSTALL_LIB_DIR}/libboost_filesystem.a
  ${CARLA_INSTALL_LIB_DIR}/libboost_program_options.a
  ${CARLA_INSTALL_LIB_DIR}/libboost_python38.a
  ${CARLA_INSTALL_LIB_DIR}/libboost_system.a
  ${CARLA_INSTALL_LIB_DIR}/libcarla_client.a
  ${CARLA_INSTALL_LIB_DIR}/librpc.a
  ${CARLA_INSTALL_LIB_DIR}/libDebugUtils.a
  ${CARLA_INSTALL_LIB_DIR}/libDetour.a
  ${CARLA_INSTALL_LIB_DIR}/libDetourCrowd.a
  ${CARLA_INSTALL_LIB_DIR}/libDetourTileCache.a
  ${CARLA_INSTALL_LIB_DIR}/libRecast.a)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

include_directories(eigen-3.3.7)

include_directories(${CARLA_INSTALL_INC_DIR})
include_directories(/usr/include/boost)

link_directories(${CARLA_INSTALL_LIB_DIR})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(spiral_planner ${sources} )

target_link_libraries(spiral_planner z ssl uv uWS ${carla_libs} gtest glog boost_filesystem lapack)

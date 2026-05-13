cmake_minimum_required(VERSION 3.14)
project(ROSCO VERSION 2.9.0 LANGUAGES C CXX)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the build type: Debug RelWithDebInfo Release" FORCE)
endif()

# --------------------------------------------------------------------------
# Phar Lap ETS static build option
# --------------------------------------------------------------------------
option(ROSCO_PHARLAP "Build a fully-static 32-bit DLL for Phar Lap ETS (MinGW MSVCRT)" OFF)

if(ROSCO_PHARLAP)
  message(STATUS "ROSCO_PHARLAP=ON: configuring static 32-bit build for Phar Lap ETS")
  set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -m32 -static -static-libgcc")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32 -static -static-libgcc -static-libstdc++")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -m32 -static -static-libgcc -static-libstdc++ -Wl,--kill-at")
  add_definitions(-DROSCO_NO_EXT_DLL -DROSCO_PHARLAP)
endif()

# C++17 required for toml++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# VIT: disable FP contraction for bit-identical verification during translation phase.
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=off")
endif()

# toml++ (single-header, C++17, no external deps)
include(FetchContent)
FetchContent_Declare(
    tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG        v3.4.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(tomlplusplus)

set(SOURCES
    src/rosco_types_io.cpp
    src/discon.cpp
    src/Stages/stage_1_sensing.cpp
    src/Stages/stage_2_setup.cpp
    src/Stages/stage_3_filtering.cpp
    src/Stages/stage_4_estimation.cpp
    src/Stages/stage_5_supervisory.cpp
    src/Stages/stage_6_setpoints.cpp
    src/Stages/stage_7_actuators.cpp
    src/Stages/stage_8_output.cpp
    src/Setpoints/speedsetpoints.cpp
    src/Setpoints/pitchsaturation.cpp
    src/Setpoints/powercontrolsetpoints.cpp
    src/Setpoints/refspeedexclusion.cpp
    src/Setpoints/setpointsmoother.cpp
    src/Supervisory/shutdown.cpp
    src/Supervisory/startup.cpp
    src/SubControllers/torquestatemachine.cpp
    src/Estimators/windspeedestimator.cpp
    src/SubControllers/activewakecontrol.cpp
    src/Controllers/cablecontrol.cpp
    src/Controllers/flapcontrol.cpp
    src/SubControllers/floatingfeedback.cpp
    src/SubControllers/foreaftdamping.cpp
    src/SubControllers/ipc.cpp
    src/Controllers/pitchcontrol.cpp
    src/Controllers/structuralcontrol.cpp
    src/Controllers/torquecontrol.cpp
    src/Controllers/yawratecontrol.cpp
    src/Filters/hpfilter.cpp
    src/Filters/lpfilter.cpp
    src/Filters/notchfilter.cpp
    src/Filters/notchfilterslopes.cpp
    src/Filters/prefiltermeasuredsignals.cpp
    src/Filters/seclpfilter.cpp
    src/Filters/seclpfilter_vel.cpp
    src/Functions/aerodyntorque.cpp
    src/Functions/colemantransform.cpp
    src/Functions/interpolation.cpp
    src/Functions/linalg.cpp
    src/Functions/sigma.cpp
    src/Functions/wrapping.cpp
    src/ControlElements/ratelimiter.cpp
    src/ControlElements/rescontroller.cpp
    src/ControlElements/saturate.cpp
    src/IO/debug.cpp
    src/IO/extcontroller.cpp
    src/IO/readrestartfile.cpp
    src/IO/updatezeromq.cpp
    src/IO/writerestartfile.cpp
    src/ReadSetParameters/checkinputs.cpp
    src/ReadSetParameters/readavrswap.cpp
    src/ReadSetParameters/readcontrolparameterfilesub.cpp
    src/ReadSetParameters/readcpfile.cpp
    src/ReadSetParameters/readconfigfiles.cpp
    src/ReadSetParameters/setparameters.cpp
)

# Phar Lap compatibility stubs — local implementations of Win32 functions
# that Phar Lap's KERNEL32.dll doesn't provide (threading, VEH, Unicode, etc.)
# Keeps them out of the import table so the DLL loads on Phar Lap ETS.
if(ROSCO_PHARLAP)
  list(APPEND SOURCES src/pharlap_compat.c)
endif()

# ZMQ Library — disabled for Phar Lap builds
if(NOT ROSCO_PHARLAP)
  find_package(PkgConfig)
  pkg_check_modules(PC_ZeroMQ libzmq)
  if(PC_ZeroMQ_FOUND)
      find_path(ZeroMQ_INCLUDE_DIR NAMES zmq.h PATHS ${PC_ZeroMQ_INCLUDE_DIRS})
      find_library(ZeroMQ_LIBRARY NAMES zmq PATHS ${PC_ZeroMQ_LIBRARY_DIRS})
      add_library(zmq_client OBJECT src/zmq_client.c)
      target_include_directories(zmq_client PRIVATE ${ZeroMQ_INCLUDE_DIR})
      if(NOT MSVC)
        target_compile_options(zmq_client PRIVATE -fPIC)
      endif()
      add_definitions(-DZMQ_CLIENT="TRUE")
      set(SOURCES ${SOURCES} $<TARGET_OBJECTS:zmq_client>)
  endif()
endif()

add_library(discon SHARED ${SOURCES})
target_include_directories(discon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../translations)

# Link libdl only on Unix (not Windows, not Phar Lap)
if(NOT MSVC AND NOT WIN32)
    target_link_libraries(discon PRIVATE dl)
endif()

target_link_libraries(discon PRIVATE tomlplusplus::tomlplusplus)

if(PC_ZeroMQ_FOUND AND NOT ROSCO_PHARLAP)
    target_include_directories(discon PUBLIC ${ZeroMQ_INCLUDE_DIR})
    target_link_libraries(discon PUBLIC ${ZeroMQ_LIBRARY})
endif()

install(TARGETS discon
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

set(linuxDefault ${CMAKE_INSTALL_PREFIX} STREQUAL "/usr/local")
set(windowsDefault ${CMAKE_INSTALL_PREFIX} STREQUAL "C:\\Program Files (x86)")
if(${linuxDefault} OR ${windowsDefault})
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/../")
endif()

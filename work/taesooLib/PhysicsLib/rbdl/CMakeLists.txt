# -*- mode: cmake; -*-
project(PhysicsLib)

cmake_minimum_required(VERSION 3.0)

get_filename_component(PhysicsLib_DIR ${CMAKE_CURRENT_SOURCE_DIR} PATH)
message(${PhysicsLib_DIR})
include_directories (
	${CMAKE_CURRENT_SOURCE_DIR}
	${PhysicsLib_DIR}
	/usr/include/eigen3
	/usr/local/include/eigen3
	${CMAKE_CURRENT_BINARY_DIR} # rbdl_config.h goes here.
	)

OPTION (RBDL_ENABLE_LOGGING "Enable logging (warning: major impact on performance!)" ON)
SET ( RBDL_VERSION_MAJOR 2 )
SET ( RBDL_VERSION_MINOR 6 )
SET ( RBDL_VERSION_PATCH 0 )
SET ( RBDL_VERSION
  ${RBDL_VERSION_MAJOR}.${RBDL_VERSION_MINOR}.${RBDL_VERSION_PATCH}
)
SET ( RBDL_SO_VERSION
  ${RBDL_VERSION_MAJOR}.${RBDL_VERSION_MINOR}.${RBDL_VERSION_PATCH}
)
SET (RBDL_BUILD_COMPILER_ID ${CMAKE_CXX_COMPILER_ID})
SET (RBDL_BUILD_COMPILER_VERSION ${CMAKE_CXX_COMPILER_VERSION})

CONFIGURE_FILE (
  "${CMAKE_CURRENT_SOURCE_DIR}/rbdl_config.h.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/rbdl/rbdl_config.h"
  )

FILE ( GLOB rbdl_headers
  ${CMAKE_CURRENT_SOURCE_DIR}/*.h
  ${CMAKE_CURRENT_BINARY_DIR}/rbdl_config.h
  )
SET(rbdl_src
	${rbdl_headers}
  rbdl_version.cc
  rbdl_mathutils.cc
  rbdl_utils.cc
  Constraints.cc
  Dynamics.cc
  Logging.cc
  Joint.cc
  Model.cc
  Kinematics.cc
  tfile_standalone.h
  tfile_standalone.cpp
	)
#add_executable(sample ${rbdl_src} example.cc)
add_executable(sample ${rbdl_src} exampleQuat.cc)
#add_executable(sample ${rbdl_src} exampleEuler.cc)

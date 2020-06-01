# -*- mode: cmake; -*-
cmake_minimum_required(VERSION 2.8)

project (console)


set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/..")

if (UseMPI)
	include(Common_console_mpi) 
else()
	include(Common_console) 
endif()
add_subdirectory(../../PhysicsLib "${CMAKE_SOURCE_DIR}/../../PhysicsLib/build_console_${CMAKE_BUILD_TYPE}" )
add_subdirectory(../../ClassificationLib "${CMAKE_SOURCE_DIR}/../../ClassificationLib/build_console_${CMAKE_BUILD_TYPE}" )


include_directories (
	.
	../QP_controller/qpOASES-3.0beta/include
	../QP_controller/
	)

set(src
  "./OgreFltk.cpp"
  "./RigidBodyWin.cpp"
  "./RigidBodyWin.h"
  "../QP_controller/eiquadprog.hpp"
  "../QP_controller/quadprog.cpp"
  "../QP_controller/quadprog.h"
  "../QP_controller/quadprog_qpOASES.cpp"
  "../classification/cma/cmaes.c"
  "../classification/cma/CMAwrap.cpp"
  "../classification/cma/CMAwrap.h"
  )


set(luna_script "${CMAKE_SOURCE_DIR}")

add_custom_command(
	OUTPUT "${luna_script}/luna_QP.cpp"
	DEPENDS "${luna_script}/luna_QP.lua" "${LUNA_GEN}"
	PRE_BUILD
	COMMAND lua "${LUNA_GEN}" "${luna_script}/luna_QP.lua"
)

add_executable(OgreFltk
	${src}
	"${CMAKE_SOURCE_DIR}/luna_QP.cpp"
  )

  set(QPOASES_LIB
	  # build using cd qp<TAB>;cmake  . ;make
	  "${CMAKE_SOURCE_DIR}/../QP_controller/qpOASES-3.0beta/libs/libqpOASES.a"
	  "${CMAKE_SOURCE_DIR}/../QP_controller/qpOASES-3.0beta/libs/libqpOASESextras.a"
	  )
  target_link_libraries(OgreFltk 
	  BaseLib 
	MainLib 
	PhysicsLib 
	ClassificationLib
	#${FLTK_LIBS}
	${IMAGE_LIBS}
	${OGRE_LIBS}
	${QPOASES_LIB}
	dl
	)
if(WIN32)
  add_custom_command(TARGET OgreFltk
                       POST_BUILD
                       COMMAND lua "${CMAKE_CURRENT_SOURCE_DIR}/make" install)
endif()

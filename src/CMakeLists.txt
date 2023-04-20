# -*- mode: cmake; -*-
cmake_minimum_required(VERSION 3.0)

project (QP_controller)

# include ../Common.cmake
set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/taesooLib/Samples")
get_filename_component(CAR_SAMPLE_DIR ${CMAKE_SOURCE_DIR} PATH)
get_filename_component(PROJ_ROOT_DIR ${CAR_SAMPLE_DIR} PATH)
set(TAESOOLIB_DIR "${CMAKE_SOURCE_DIR}/taesooLib")

#include(FindPkgConfig)
#pkg_search_module(PYTHON3 REQUIRED python3)
#pkg_search_module(IPOPT REQUIRED ipopt)
#message(${PYTHON3_LIBDIR})
#message(${PYTHON3_INCLUDE_DIRS})

execute_process(
	COMMAND "python3" -c "from sysconfig import get_paths as gp; print(gp()['include'])"
	OUTPUT_VARIABLE PYTHON3_INCLUDE_DIRS
	OUTPUT_STRIP_TRAILING_WHITESPACE
	)
execute_process(
	COMMAND "python3" -c "import numpy; print(numpy.get_include())"
	OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
	OUTPUT_STRIP_TRAILING_WHITESPACE
	)
message(${NUMPY_INCLUDE_DIR})


set(useShared TRUE)
set(NoDevil TRUE)
set(NoFreeImage FALSE)
set(NoLapack TRUE)
set(NoUMFpack TRUE)

include(Common_baselib) 
add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/BaseLib "./BaseLib/build_shared_${CMAKE_BUILD_TYPE}" )
add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/MainLib "./MainLib/build_shared_${CMAKE_BUILD_TYPE}" )
include(Common_mainlib) 

# PhysicsLib settings
set(NoAISTsim TRUE) # use TRL_LCP instead!
set(NoGMBSsim TRUE) # use TRL_LCP instead!
set(NoUTsim TRUE) # use TRL_LCP instead!
set(NoBulletSim TRUE) # use TRL_LCP instead!
add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/PhysicsLib "./PhysicsLib/build_shared_${CMAKE_BUILD_TYPE}" )
add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/ClassificationLib "./ClassificationLib/build_shared_${CMAKE_BUILD_TYPE}" )


add_definitions(-D_USRDLL)
add_definitions(-DMAINLIBPYTHON_EXPORTS)
add_definitions(-DML_LIBRARY)


include_directories (	
	${PYTHON3_INCLUDE_DIRS}
	${NUMPY_INCLUDE_DIR}
	#"/usr/include/python3.6"
	#"/usr/include/python3.7m"
	#"/usr/include/python3.8"
	#"/opt/homebrew/lib/python3.10/site-packages/numpy/core/include" #numpy headers
	#"/usr/local/lib/python3.10/dist-packages/numpy/core/include" #numpy headers
	#"/usr/local/lib/python3.8/site-packages/numpy/core/include"
	#"/usr/local/lib/python3.6/dist-packages/numpy/core/include" #numpy headers
	.
	../../taesooLib/Samples/QP_controller/qpOASES-3.0beta/include
	../../taesooLib/Samples/scripts
	${TAESOOLIB_DIR}
	/usr/include/eigen3
	)
set(src_python
# 	python only
  "MainlibPython.cpp"
  "MainlibPython.h"
  "PythonExtendWin.cpp"
  "PythonExtendWin.h"
  "./main.cpp"
  )

set(src
  #"./MyScriptWin.cpp"
  #"./MyScriptWin.h"
  "cma/cmaes.c"
  "cma/CMAwrap.cpp"
  "cma/CMAwrap.h"
  )

set(src11 
	"eigenTest.cpp"
	"GA_simple.h"
	"GA_simple.cpp"
	)
set(luna_script "${CMAKE_SOURCE_DIR}")

add_custom_command(
	OUTPUT "${luna_script}/luna_QP.cpp"
	DEPENDS "${luna_script}/luna_QP.lua" "${LUNA_GEN}" 
	PRE_BUILD
	COMMAND lua "${LUNA_GEN}" "${luna_script}/luna_QP.lua"
)

if(UNIX)
	#SET_SOURCE_FILES_PROPERTIES( ${src11} PROPERTIES COMPILE_FLAGS "--std=c++0x")
endif()

if(WIN32)
	
else()
	set(TAESOO_LIBS
		BaseLib
		MainLib
		PhysicsLib
		ClassificationLib
		)
endif()


	set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CAR_SAMPLE_DIR}/work )

	set(usePybind11 TRUE)
	add_definitions(-DUSE_PYBIND11)
	add_subdirectory(pybind11)
	pybind11_add_module(libmainlib 
		${src}
		${src11}
		${src_python}
		"${CMAKE_SOURCE_DIR}/luna_QP.cpp" # important. without this, luna_gen doesn't run.
		)

	target_link_libraries(libmainlib PUBLIC
		${TAESOO_LIBS}
		${FLTK_LIBS}
		${IMAGE_LIBS}
		${OGRE_LIBS}
		${QPOASES_LIB}
		dl
		#boost_numpy
		pthread
		)

if(APPLE)
else()
	target_link_libraries(libmainlib PUBLIC dl)
endif()


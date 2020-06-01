# -*- mode: cmake; -*-
cmake_minimum_required(VERSION 2.8)

project (QP_controller)

# include ../Common.cmake
set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/taesooLib/Samples")
get_filename_component(CAR_SAMPLE_DIR ${CMAKE_SOURCE_DIR} PATH)
get_filename_component(PROJ_ROOT_DIR ${CAR_SAMPLE_DIR} PATH)
set(TAESOOLIB_DIR "${CMAKE_SOURCE_DIR}/taesooLib")

if (APPLE)
	include(FindPkgConfig)
	pkg_search_module(PYTHON3 REQUIRED python-3.8)
#	pkg_search_module(IPOPT REQUIRED ipopt)
	message(${PYTHON3_LIBDIR})
	message(${PYTHON3_INCLUDE_DIRS})
endif()

if (WIN32)
	message(${TAESOOLIB_DIR})
	include(Common_fast) 

	set(LUNA_GEN "${TAESOOLIB_DIR}/MainLib/WrapperLua/luna_gen.lua")
else()
	set(useShared TRUE)
	set(NoDevil TRUE)
	set(NoFreeImage FALSE)
	set(NoLapack TRUE)
	set(NoUMFpack TRUE)

	include(Common_baselib) 
	add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/BaseLib "${CMAKE_SOURCE_DIR}/taesooLib/BaseLib/build_shared_${CMAKE_BUILD_TYPE}" )
	add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/MainLib "${CMAKE_SOURCE_DIR}/taesooLib/MainLib/build_shared_${CMAKE_BUILD_TYPE}" )
	include(Common_mainlib) 

	# PhysicsLib settings
	set(NoAISTsim TRUE) # use TRL_LCP instead!
	set(NoGMBSsim TRUE) # use TRL_LCP instead!
	set(NoUTsim TRUE) # use TRL_LCP instead!
	set(NoBulletSim TRUE) # use TRL_LCP instead!
	add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/PhysicsLib "${CMAKE_SOURCE_DIR}/taesooLib/PhysicsLib/build_shared_${CMAKE_BUILD_TYPE}" )
	add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/ClassificationLib "${CMAKE_SOURCE_DIR}/taesooLib/ClassificationLib/build_shared_${CMAKE_BUILD_TYPE}" )
endif()


add_definitions(-D_USRDLL)
add_definitions(-DMAINLIBPYTHON_EXPORTS)
add_definitions(-DML_LIBRARY)


include_directories (	
	${PYTHON3_INCLUDE_DIRS}
	"/usr/include/python3.6"
	"/usr/include/python3.7m"
	"/usr/local/lib/python3.6/dist-packages/numpy/core/include" #numpy headers
	"/usr/local/lib/python3.8/site-packages/numpy/core/include"
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
	if(APPLE)
		add_library(mainlib MODULE
			${src}
			${src11}
			${src_python}
			"${CMAKE_SOURCE_DIR}/luna_QP.cpp" # important. without this, luna_gen doesn't run.
			)
	else()
		add_library(mainlib SHARED
			${src}
			${src11}
			${src_python}
			"${CMAKE_SOURCE_DIR}/luna_QP.cpp" # important. without this, luna_gen doesn't run.
			)
	endif()

	target_link_libraries(mainlib 
		${TAESOO_LIBS}
		${FLTK_LIBS}
		${IMAGE_LIBS}
		${OGRE_LIBS}
		${QPOASES_LIB}
		dl
		#boost_numpy
		)

	if(APPLE)
		target_link_directories(mainlib  PUBLIC
			${PYTHON3_LIBDIR}
			)
		target_link_libraries(mainlib 
			python3.8
			boost_python38
			)
	else()
		target_link_libraries(mainlib 
			boost_python3
			)
	endif()

if(WIN32)
  add_custom_command(TARGET OgreFltk
                       POST_BUILD
                       COMMAND lua "${CMAKE_CURRENT_SOURCE_DIR}/build.lua" install)
endif()

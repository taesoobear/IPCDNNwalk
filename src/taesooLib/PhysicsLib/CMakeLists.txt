# -*- mode: cmake; -*-
project(PhysicsLib)

cmake_minimum_required(VERSION 3.0)

# Make sure the compiler can find include files from our Hello library.
set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Samples")
include(Common_baselib) 
include(Common_mainlib) 
include_directories (
	${PhysicsLib_SOURCE_DIR}
	${PhysicsLib_SOURCE_DIR}/CollisionDetector/Bullet/bullet-2.68/src
	${PhysicsLib_SOURCE_DIR}/../Samples/FlexibleBody
	${PhysicsLib_SOURCE_DIR}/AIST_implementation/tvmet-1.7.2/include
	${LUA_INCLUDE}
	${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/INCLUDE
	${PhysicsLib_SOURCE_DIR}/CollisionDetector/libccd
	${PhysicsLib_SOURCE_DIR}/../BaseLib/motion/GMBS
	/usr/include/eigen3
	)

if(WIN32)
include_directories (
	${PhysicsLib_SOURCE_DIR}/../dependencies_windows_only
	)
link_directories("${CMAKE_SOURCE_DIR}/../../dependencies/clapack-3.2.1-CMAKE/SRC/Release"
				"${CMAKE_SOURCE_DIR}/../../dependencies/clapack-3.2.1-CMAKE/F2CLIBS/libf2c/Release" 
				"${CMAKE_SOURCE_DIR}/../../dependencies/clapack-3.2.1-CMAKE/BLAS/SRC/Release" )
else()
link_directories("${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/SRC"
				 "${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/F2CLIBS/libf2c" 
				 "${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/BLAS/SRC" )


endif() 
if(useShared)
elseif(useOPCODEdetector)
	  add_definitions(-DINCLUE_OPCODE_DETECTOR)
	  add_subdirectory(CollisionDetector)
  endif()

  set(NoBullet TRUE)
if(NoBullet)
	add_definitions(-DNO_BULLET)
else()
	add_definitions(-DBT_USE_DOUBLE_PRECISION -DSEGA)
	add_subdirectory(../Samples/FlexibleBody/BulletDynamics "build_bulletDynamics_${CMAKE_BUILD_TYPE}" )
endif()

set(libccd
	CollisionDetector/libccd/ccd/config.h
	CollisionDetector/libccd/ccd.c
	CollisionDetector/libccd/mpr.c
	CollisionDetector/libccd/support.c
	CollisionDetector/libccd/vec3.c
	CollisionDetector/libccd/polytope.c
	CollisionDetector/libccd/testsuites/support.c
	CollisionDetector/libccd/testsuites/support.h
	CollisionDetector/CollisionDetector_libccd.h
	CollisionDetector/CollisionDetector_libccd.cpp
	#CollisionDetector/CollisionDetector_libccd_specialPlane.h
	#CollisionDetector/CollisionDetector_libccd_specialPlane.cpp
	#CollisionDetector/CollisionDetector_libccd_cylinder.h
	#CollisionDetector/CollisionDetector_libccd_cylinder.cpp
	)

set(src 
  "./SDRE.cpp"
  "./SDRE.h"
  "./clapack_wrap.cpp"
  "./clapack_wrap.h"
	mrdplot.c
	mrdplot.h
	mrdplot.hpp
	mrdplotWrap.cpp
	#"${luna_script}/luna_physics.h"
  "DynamicsSimulator_QP.cpp"
  "DynamicsSimulator_QP.h"
  "convexhull/graham.cpp"
  "CollisionDetector.cpp"
	  CollisionDetector/Bullet/CollisionDetector_bullet.cpp
	  CollisionDetector/Bullet/CollisionDetector_bullet.h
	  CollisionDetector/CollisionDetector_gjk.cpp
	  CollisionDetector/CollisionDetector_gjk.h
  "ScriptBaseWin.cpp"
  "ScriptBaseWin.h"
  "DynamicsSimulator.cpp"
  "DynamicsSimulator_penaltyMethod.cpp"
  "OpenHRPcommon.cpp"
	)

set(src11
	TRL/TRL_common.h
	TRL/mat33.cpp
	TRL/Body.cpp
	TRL/Body.h
	TRL/DynamicsSimulator_TRL_penalty.cpp
	TRL/DynamicsSimulator_TRL_penalty.h
	TRL/DynamicsSimulator_TRL_LCP.cpp
	TRL/DynamicsSimulator_TRL_LCP.h
	TRL/DynamicsSimulator_TRL_QP.cpp
	TRL/DynamicsSimulator_TRL_QP.h
	TRL/ForwardDynamicsABM.cpp
	TRL/ForwardDynamicsABM.h
	TRL/ContactForceSolver.cpp
	TRL/ContactForceSolver.h
	TRL/MCPsolver.cpp
	TRL/Link.cpp
	TRL/Link.h
	TRL/LinkPath.cpp
	TRL/LinkPath.h
	TRL/LinkTraverse.cpp
	TRL/LinkTraverse.h
	TRL/ModelLoaderUtil.cpp
	TRL/ModelLoaderUtil.h
	TRL/World.cpp
	TRL/World.h
	)
if(UNIX)
	SET_SOURCE_FILES_PROPERTIES( ${src11} PROPERTIES COMPILE_FLAGS "-std=c++0x")
endif()
set(NoAISTsim TRUE) # use TRL_LCP instead!
set(NoGMBSsim TRUE) # use TRL_LCP instead!
set(NoUTsim TRUE) # use TRL_LCP instead!
if(NoAISTsim)
	add_definitions(-DEXCLUDE_AIST_SIM)
		set(aist_src 
			)
else()
		set(aist_src 
			AIST_implementation/AIST_common.h
			AIST_implementation/Body.cpp
			AIST_implementation/BodyCustomizerInterface.cpp
			AIST_implementation/BodyCustomizerInterface.h
			AIST_implementation/Body.h
			AIST_implementation/ConstraintForceSolver.cpp
			AIST_implementation/ConstraintForceSolver.h
			AIST_implementation/ConstraintForceSolver_impl.h
			AIST_implementation/ConstraintForceSolver_impl.hpp
			AIST_implementation/DynamicsSimulator_AIST_penalty.cpp
			AIST_implementation/DynamicsSimulator_AIST_penalty.h
			AIST_implementation/DynamicsSimulator_impl.cpp
			AIST_implementation/DynamicsSimulator_impl.h
			AIST_implementation/ForwardDynamicsABM.cpp
			AIST_implementation/ForwardDynamicsABM.h
			AIST_implementation/ForwardDynamicsCBM.cpp
			AIST_implementation/ForwardDynamicsCBM.h
			AIST_implementation/ForwardDynamics.cpp
			AIST_implementation/ForwardDynamics.h
			AIST_implementation/hrpModelExportDef.h
			AIST_implementation/hrpModelHeaders.h
			AIST_implementation/Link.cpp
			AIST_implementation/Link.h
			AIST_implementation/LinkPath.cpp
			AIST_implementation/LinkPath.h
			AIST_implementation/LinkTraverse.cpp
			AIST_implementation/LinkTraverse.h
			#AIST_implementation/MatrixSolvers.cpp
			#AIST_implementation/MatrixSolvers.h
			AIST_implementation/ModelLoaderUtil.cpp
			AIST_implementation/ModelLoaderUtil.h
			AIST_implementation/quaternion.cpp
			AIST_implementation/quaternion.h
			AIST_implementation/Sensor.cpp
			AIST_implementation/Sensor.h
			#AIST_implementation/server.cpp
			AIST_implementation/TimeMeasure.h
			AIST_implementation/tvmet3d.cpp
			AIST_implementation/tvmet3d.h
			AIST_implementation/tvmetCommonTypes.h
			AIST_implementation/ublasCommonTypes.h
			AIST_implementation/World.cpp
			AIST_implementation/World.h
			)
endif()

if(NoGMBSsim)
	add_definitions(-DEXCLUDE_GMBS_SIM)
	set(gmbs_src
		)
else()
	set(gmbs_src
		)
endif()

FILE(GLOB Bullet "./Bullet_implementation/*.cpp")
LIST(APPEND src ${Bullet})

if(NoUTsim)
	add_definitions(-DEXCLUDE_UT_SIM)
else()
	FILE(GLOB UT "./UT_implementation/*.cpp")
	FILE(GLOB sDIMS "./sDIMS/*.cpp")
	FILE(GLOB sDIMS_remove "./sDIMS/load.cpp")
	list(REMOVE_ITEM sDIMS ${sDIMS_remove})
	LIST(APPEND UT 
		"./ModelLoaderUtil_UT.cpp"
		"./ModelLoaderUtil_UT.h"
		"./World_UT.h"
		"./World_UT.cpp"
		DynamicsSimulator_UT.cpp
		DynamicsSimulator_UT.h
		)
	LIST(APPEND src ${UT})
	LIST(APPEND src ${sDIMS})
endif()

include_directories(${PhysicsLib_SOURCE_DIR}/sDIMS)

set(luna_script "${PhysicsLib_SOURCE_DIR}")
set(LUNA_GEN ${PhysicsLib_SOURCE_DIR}/../MainLib/WrapperLua/luna_gen.lua)

add_custom_command(
	OUTPUT "${luna_script}/luna_physics.cpp"
	DEPENDS "${luna_script}/luna_physics.lua" "${LUNA_GEN}" 
	PRE_BUILD
	COMMAND lua "${LUNA_GEN}" "${luna_script}/luna_physics.lua"
)
list(APPEND src "${luna_script}/luna_physics.cpp")
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
if(useShared)
	add_library(PhysicsLib SHARED ${libccd} ${src} ${aist_src} ${gmbs_src} ${src11})
else()
	add_library(PhysicsLib ${libccd} ${src} ${aist_src} ${gmbs_src} ${src11})
endif()

if(NoLapack)
	add_definitions(-DEXCLUDE_CLAPACK)
	set(lapack_LIBS )
else()
	if(MSVC)
		set(lapack_LIBS 
			optimized "${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/SRC/Release/lapack.lib"
			optimized "${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/BLAS/SRC/Release/blas.lib"
			optimized "${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/F2CLIBS/libf2c/Release/libf2c.lib"
			debug "${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/SRC/Debug/lapack.lib"
			debug "${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/BLAS/SRC/Debug/blas.lib"
			debug "${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/F2CLIBS/libf2c/Debug/libf2c.lib"
			)
	else()
		if(UseMPI)
			set(lapack_LIBS
				"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/SRC/liblapack.a"
				"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/BLAS/SRC/libblas.a"
				"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/F2CLIBS/libf2c/libf2c.a"
				)
		else()
			set(lapack_LIBS
				lapack
				blas
				f2c
				#"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/SRC/liblapack.a"
				#"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/BLAS/SRC/libblas.a"
				#"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/F2CLIBS/libf2c/libf2c.a"
				)
			if(useShared)
				set(lapack_LIBS
					"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_shared_Release/SRC/liblapack.so"
					"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_shared_Release/BLAS/SRC/libblas.so"
					"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_shared_Release/F2CLIBS/libf2c/libf2c.so"
					)
			endif()
		endif()
	endif()
endif()

if(useOPCODEdetector)
	add_definitions(-DINCLUDE_OPCODE_DETECTOR)
	LIST(APPEND lapack_LIBS 
		OpcodeCollisionDetector
		)
endif()

if(useShared)
	set(BULLET_LIB
		)
else()

	if(NoBulletSim)
		set(BULLET_LIB)
	else()
		set(BULLET_LIB
			LibBulletCollision
			LibLinearMath
			#LibBulletSoftBody
			LibBulletDynamics
			)
	endif()
endif()
target_link_libraries(PhysicsLib
BaseLib
MainLib
${LUA_LIB}
#${OGRE_LIBS}
${lapack_LIBS}
)

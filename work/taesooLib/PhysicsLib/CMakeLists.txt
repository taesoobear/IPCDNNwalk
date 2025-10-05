# -*- mode: cmake; -*-
project(PhysicsLib)

cmake_minimum_required(VERSION 3.11)


# Make sure the compiler can find include files from our Hello library.
set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Samples")
include(Common_baselib) 
include(Common_mainlib) 
include_directories (
	${PhysicsLib_SOURCE_DIR}
	${PhysicsLib_SOURCE_DIR}/AIST_implementation/tvmet-1.7.2/include
	${LUA_INCLUDE}
	${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/INCLUDE
	${PhysicsLib_SOURCE_DIR}/CollisionDetector/libccd
	${PhysicsLib_SOURCE_DIR}/CollisionDetector/fcl/include
	${PhysicsLib_SOURCE_DIR}/../BaseLib/motion/GMBS
	/usr/include/eigen3
	${CMAKE_CURRENT_BINARY_DIR} # rbdl_config.h goes here.
	)

if(UseModifiedBullet)
include_directories (
	${PhysicsLib_SOURCE_DIR}/CollisionDetector/Bullet/bullet-2.68/src
	${PhysicsLib_SOURCE_DIR}/../Samples/FlexibleBody
	)
else()

	include_directories(
		/usr/include/bullet
		)
endif()
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
if(useOPCODEdetector)
	add_definitions(-DINCLUDE_OPCODE_DETECTOR)
	message("OPCODE detector enabled")
	include_directories (
		"{PhysicsLib_SOURCE_DIR}/CollisionDetector"
		)
endif()


if(EXCLUDE_RBDL_SIMULATOR)
	SET(rbdl_src )
	add_definitions(-DEXCLUDE_RBDL_SIMULATOR)
else()
OPTION (RBDL_ENABLE_LOGGING "Enable logging (warning: major impact on performance!)" OFF)
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
  "${CMAKE_CURRENT_SOURCE_DIR}/rbdl/rbdl_config.h.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/rbdl/rbdl_config.h"
  )

FILE ( GLOB rbdl_headers
  ${CMAKE_CURRENT_SOURCE_DIR}/rbdl/*.h
  ${CMAKE_CURRENT_BINARY_DIR}/rbdl/rbdl_config.h
  )
SET(rbdl_src
	${rbdl_headers}
  rbdl/rbdl_version.cc
  rbdl/rbdl_mathutils.cc
  rbdl/rbdl_utils.cc
  rbdl/Constraints.cc
  rbdl/Dynamics.cc
  rbdl/Logging.cc
  rbdl/Joint.cc
  rbdl/Model.cc
  rbdl/Kinematics.cc
  # Listed below are added by Taesoo
  rbdl/DynamicsSimulator_Trbdl_penalty.h
  rbdl/DynamicsSimulator_Trbdl_penalty.cpp
  rbdl/DynamicsSimulator_Trbdl_LCP.h
  rbdl/DynamicsSimulator_Trbdl_LCP.cpp
  rbdl/DynamicsSimulator_Trbdl_QP.h
  rbdl/DynamicsSimulator_Trbdl_QP.cpp
  rbdl/DynamicsSimulator_Trbdl_impulse.h
  rbdl/DynamicsSimulator_Trbdl_impulse.cpp
  rbdl/LCPsolver.cpp
  rbdl/LCPsolver.h
  #rbdl/simbody/simbody_test.cpp
  #rbdl/simbody/rbdlsim.cc
  #rbdl/simbody/rbdlsim.h
  #rbdl/simbody/utils.cc
  #rbdl/simbody/utils.h
	)
endif()
set(libccd
	#CollisionDetector/libccd/ccd/config.h
	#CollisionDetector/libccd/ccd.c
	#CollisionDetector/libccd/mpr.c
	#CollisionDetector/libccd/support.c
	#CollisionDetector/libccd/vec3.c
	#CollisionDetector/libccd/polytope.c
	#CollisionDetector/libccd/testsuites/support.c
	#CollisionDetector/libccd/testsuites/support.h
	CollisionDetector/CollisionDetector_fcl.h
	CollisionDetector/CollisionDetector_fcl.cpp
	CollisionDetector/CollisionDetector_libccd.h
	CollisionDetector/CollisionDetector_libccd.cpp
	#CollisionDetector/CollisionDetector_libccd_LBS.h
	#CollisionDetector/CollisionDetector_libccd_LBS.cpp
	#CollisionDetector/CollisionDetector_simple2.h
	#CollisionDetector/CollisionDetector_simple2.cpp
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
	  "GMBS_implementation/rmatrix3j.cpp"
	  "GMBS_implementation/rmatrix3j.h"
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
	  "GMBS_implementation/DynamicsSimulator_gmbs.cpp"
	  "GMBS_implementation/DynamicsSimulator_gmbs.h"
	  "GMBS_implementation/DynamicsSimulator_gmbs_penalty.cpp"
	  "GMBS_implementation/DynamicsSimulator_gmbs_penalty.h"
	  "GMBS_implementation/gbody.cpp"
	  "GMBS_implementation/gbody.h"
	  "GMBS_implementation/gbody_rigid.cpp"
	  "GMBS_implementation/gbody_rigid.h"
	  "GMBS_implementation/gconstraint.cpp"
	  "GMBS_implementation/gconstraint.h"
	  "GMBS_implementation/gconstraint_jointloop.cpp"
	  "GMBS_implementation/gconstraint_jointloop.h"
	  "GMBS_implementation/gcoordinate.h"
	  "GMBS_implementation/gelement.cpp"
	  "GMBS_implementation/gelement.h"
	  "GMBS_implementation/gfunc.cpp"
	  "GMBS_implementation/gfunc.h"
	  "GMBS_implementation/gjoint.cpp"
	  "GMBS_implementation/gjoint.h"
	  "GMBS_implementation/gjoint_composite.cpp"
	  "GMBS_implementation/gjoint_composite.h"
	  "GMBS_implementation/gjoint_fixed.cpp"
	  "GMBS_implementation/gjoint_fixed.h"
	  "GMBS_implementation/gjoint_free.cpp"
	  "GMBS_implementation/gjoint_free.h"
	  "GMBS_implementation/gjoint_planar.cpp"
	  "GMBS_implementation/gjoint_planar.h"
	  "GMBS_implementation/gjoint_prismatic.cpp"
	  "GMBS_implementation/gjoint_prismatic.h"
	  "GMBS_implementation/gjoint_revolute.cpp"
	  "GMBS_implementation/gjoint_revolute.h"
	  "GMBS_implementation/gjoint_spherical.cpp"
	  "GMBS_implementation/gjoint_spherical.h"
	  "GMBS_implementation/gjoint_quaternion.cpp"
	  "GMBS_implementation/gjoint_quaternion.h"
	  "GMBS_implementation/gjoint_translational.cpp"
	  "GMBS_implementation/gjoint_translational.h"
	  "GMBS_implementation/gjoint_universal.cpp"
	  "GMBS_implementation/gjoint_universal.h"
	  "GMBS_implementation/gmbs.h"
	  "GMBS_implementation/gsystem.cpp"
	  "GMBS_implementation/gsystem.h"
	  #dy/GMBS_implementation/gsystem_constrained.cpp"
	  #dy/GMBS_implementation/gsystem_constrained.h"
	  "GMBS_implementation/gsystem_ik.cpp"
	  "GMBS_implementation/gsystem_ik.h"
	  #"GMBS_implementation/liegroup.cpp"
	  #"GMBS_implementation/liegroup.h"
	  "GMBS_implementation/liegroup_rmatrix3_ext.cpp"
	  "GMBS_implementation/liegroup_rmatrix3_ext.h"
	  "GMBS_implementation/rmatrix3j.cpp"
	  "GMBS_implementation/rmatrix3j.h"
	  "Liegroup.cpp"
	  "Liegroup.h"
		)
endif()


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

if(useOPCODEdetector)
	set(opcode_src
"CollisionDetector/CdCache.cpp"
"CollisionDetector/CdCache.h"
"CollisionDetector/CdScene.cpp"
"CollisionDetector/CdScene.h"
"CollisionDetector/CdWrapper.cpp"
"CollisionDetector/CdWrapper.h"
"CollisionDetector/CollisionDetector_impl.cpp"
"CollisionDetector/CollisionDetector_impl.h"
"CollisionDetector/CollisionDetector_impl2.cpp"
"CollisionDetector/CollisionDetector_impl2.h"
"CollisionDetector/InsertCollisionPair.cpp"
"CollisionDetector/OPC_AABBTree.cpp"
"CollisionDetector/OPC_AABBTree.h"
"CollisionDetector/OPC_BaseModel.cpp"
"CollisionDetector/OPC_BaseModel.h"
"CollisionDetector/OPC_BoxBoxOverlap.h"
"CollisionDetector/OPC_Collider.cpp"
"CollisionDetector/OPC_Collider.h"
"CollisionDetector/OPC_Common.cpp"
"CollisionDetector/OPC_Common.h"
"CollisionDetector/OPC_IceHook.h"
"CollisionDetector/OPC_MeshInterface.cpp"
"CollisionDetector/OPC_MeshInterface.h"
"CollisionDetector/OPC_Model.cpp"
"CollisionDetector/OPC_Model.h"
"CollisionDetector/OPC_OptimizedTree.cpp"
"CollisionDetector/OPC_OptimizedTree.h"
"CollisionDetector/OPC_Picking.cpp"
"CollisionDetector/OPC_Picking.h"
"CollisionDetector/OPC_PlanesAABBOverlap.h"
"CollisionDetector/OPC_PlanesTriOverlap.h"
"CollisionDetector/OPC_Settings.h"
"CollisionDetector/OPC_SphereAABBOverlap.h"
"CollisionDetector/OPC_SphereTriOverlap.h"
#CollisionDetector./OPC_SweepAndPrune.cpp"
#CollisionDetector./OPC_SweepAndPrune.h"
"CollisionDetector/OPC_TreeBuilders.cpp"
"CollisionDetector/OPC_TreeBuilders.h"
"CollisionDetector/OPC_TreeCollider.cpp"
"CollisionDetector/OPC_TreeCollider.h"
"CollisionDetector/OPC_TriBoxOverlap.h"
"CollisionDetector/OPC_TriTriOverlap.h"
"CollisionDetector/OPC_VolumeCollider.cpp"
"CollisionDetector/OPC_VolumeCollider.h"
"CollisionDetector/Opcode.cpp"
"CollisionDetector/Opcode.h"
"CollisionDetector/StdAfx.cpp"
"CollisionDetector/StdAfxColDet.h"
"CollisionDetector/TriOverlap.cpp"
"CollisionDetector/utilities.h"
"CollisionDetector/Ice/IceAABB.cpp"
"CollisionDetector/Ice/IceAABB.h"
"CollisionDetector/Ice/IceAxes.h"
"CollisionDetector/Ice/IceBoundingSphere.h"
"CollisionDetector/Ice/IceContainer.cpp"
"CollisionDetector/Ice/IceContainer.h"
"CollisionDetector/Ice/IceFPU.h"
"CollisionDetector/Ice/IceHPoint.cpp"
"CollisionDetector/Ice/IceHPoint.h"
"CollisionDetector/Ice/IceIndexedTriangle.cpp"
"CollisionDetector/Ice/IceIndexedTriangle.h"
"CollisionDetector/Ice/IceLSS.h"
"CollisionDetector/Ice/IceMatrix3x3.cpp"
"CollisionDetector/Ice/IceMatrix3x3.h"
"CollisionDetector/Ice/IceMatrix4x4.cpp"
"CollisionDetector/Ice/IceMatrix4x4.h"
"CollisionDetector/Ice/IceMemoryMacros.h"
"CollisionDetector/Ice/IceOBB.cpp"
"CollisionDetector/Ice/IceOBB.h"
"CollisionDetector/Ice/IcePairs.h"
"CollisionDetector/Ice/IcePlane.cpp"
"CollisionDetector/Ice/IcePlane.h"
"CollisionDetector/Ice/IcePoint.cpp"
"CollisionDetector/Ice/IcePoint.h"
"CollisionDetector/Ice/IcePreprocessor.h"
"CollisionDetector/Ice/IceRandom.cpp"
"CollisionDetector/Ice/IceRandom.h"
"CollisionDetector/Ice/IceRay.cpp"
"CollisionDetector/Ice/IceRay.h"
"CollisionDetector/Ice/IceRevisitedRadix.cpp"
"CollisionDetector/Ice/IceRevisitedRadix.h"
"CollisionDetector/Ice/IceSegment.cpp"
"CollisionDetector/Ice/IceSegment.h"
"CollisionDetector/Ice/IceTriangle.cpp"
"CollisionDetector/Ice/IceTriangle.h"
"CollisionDetector/Ice/IceTriList.h"
"CollisionDetector/Ice/IceTypes.h"
"CollisionDetector/Ice/IceUtils.cpp"
"CollisionDetector/Ice/IceUtils.h"
)


	LIST(APPEND src ${opcode_src})
	message("src" ${src})
endif()

add_subdirectory(${PhysicsLib_SOURCE_DIR}/CollisionDetector/fcl ./fcl)
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
	add_library(PhysicsLib SHARED ${libccd} ${src} ${aist_src} ${gmbs_src} ${src11} ${rbdl_src})
else()
	add_library(PhysicsLib ${libccd} ${src} ${aist_src} ${gmbs_src} ${src11} ${rbdl_src})
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
				#f2c
				#"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/SRC/liblapack.a"
				#"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/BLAS/SRC/libblas.a"
				#"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_/F2CLIBS/libf2c/libf2c.a"
				)

			if(NOT APPLE)
				list(APPEND lapack_LIBS f2c)
			endif()

			if(useShared)

				add_subdirectory(${TAESOOLIB_DIR}/dependencies/clapack-3.2.1-CMAKE build_shared)
				set(lapack_LIBS
					lapack
					blas
					f2c
					#"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_shared_Release/SRC/liblapack.so"
					#"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_shared_Release/BLAS/SRC/libblas.so"
					#"${PhysicsLib_SOURCE_DIR}/../dependencies/clapack-3.2.1-CMAKE/build_shared_Release/F2CLIBS/libf2c/libf2c.so"
					)
			endif()
		endif()
	endif()
endif()

target_link_libraries(PhysicsLib
BaseLib
MainLib
${LUA_LIB}
#${OGRE_LIBS}
${lapack_LIBS}
fcl
)
if(MSVC)
target_compile_options(PhysicsLib PRIVATE $<$<CONFIG:DEBUG>:/bigobj>)
endif()

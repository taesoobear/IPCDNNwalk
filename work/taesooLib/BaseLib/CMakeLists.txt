# -*- mode: cmake; -*-
# The name of our project is "BaseLib".  CMakeLists files in this project can
# refer to the root source directory of the project as ${BaseLib_SOURCE_DIR} and
# to the root binary directory of the project as ${BaseLib_BINARY_DIR}.

project(BaseLib)

set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Samples")
get_filename_component(TAESOOLIB_DIR ${CMAKE_CURRENT_SOURCE_DIR} PATH)
get_filename_component(ROOT_DIR ${TAESOOLIB_DIR} PATH)
include(Common_baselib) 
# Make sure the compiler can find include files from our Hello library.
include_directories (${BaseLib_SOURCE_DIR})
include_directories (${BaseLib_SOURCE_DIR}/unix)

# devIL is no longer used.
add_definitions(-DNO_DEVIL)
if(WIN32)
	if(NoFreeImage)
		add_definitions(-DNO_FREEIMAGE)
		set(IMAGE_LIBS )
	else()
		if(CMAKE_GENERATOR_PLATFORM STREQUAL "ARM64")
			set (IMAGE_LIBS 
				"${ROOT_DIR}/windows_dependencies/FreeImage/Dist/arm64/FreeImage.lib"
			)
		else()
			set (IMAGE_LIBS 
				"${ROOT_DIR}/windows_dependencies/FreeImage/Dist/x64/FreeImage.lib"
			)
		endif()
	endif()
	include_directories (
		${ROOT_DIR}/windows_dependencies/eigen-3.4.0/
		${ROOT_DIR}/windows_dependencies/FreeImage/Dist/x64/
		${BaseLib_SOURCE_DIR}/../dependencies/libxml2-2.7.3.win32/include
		)

else()
	if(NoFreeImage)
		add_definitions(-DNO_FREEIMAGE)
		# warp cluster
		include_directories (
			${BaseLib_SOURCE_DIR}/../dependencies/FreeImage/Dist
			)
	endif()
	include_directories(
		/usr/include/libxml2
		/usr/include/eigen3
		/usr/local/include/eigen3
		)
endif()
set(utility
"./baselib.h"
"./stdafx.cpp"
"./stdafx.h"
"./utility/configtable.cpp"
"./utility/configtable.h"
"./utility/NameTable.cpp"
"./utility/NameTable.h"
"./utility/operatorString.cpp"
"./utility/operatorString.h"
"./utility/Parser.cpp"
"./utility/Parser.h"
"./utility/TextFile.cpp"
"./utility/TextFile.h"
"./utility/tfile.cpp"
"./utility/tfile.h"
"./utility/cnpy/cnpy.h"
"./utility/cnpy/cnpy.cpp"
"./utility/BinaryFile.cpp"
"./utility/BinaryFile.h"
"./utility/TGL.cpp"
"./utility/TGL.h"
"./utility/Graph.h"
"./utility/Graph.cpp"
"./utility/Chat.cpp"
"./utility/Chat.h"
"./utility/VoxelGraph.h"
"./utility/VoxelGraph.cpp"
"./utility/TOnlineArray.cpp"
"./utility/TOnlineArray.h"
"./utility/TUGL.cpp"
"./utility/TUGL.h"
"./utility/TWord.cpp"
"./utility/TWord.h"
"./utility/TypeString.cpp"
"./utility/TypeString.h"
"./utility/util.cpp"
"./utility/util.h"
"./utility/namedmapsupport.h"
"./utility/namedmapsupport.cpp"
"./utility/FBX/miniz.c"
"./utility/FBX/miniz.h"
"./utility/FBX/ofbx.cpp"
"./utility/FBX/ofbx.h"
"./utility/FBX/FBXimporter.cpp"

)

LIST(APPEND utility "./utility/checkPoints.cpp")

set(motion

  "./motion/InertiaCalculator.cpp"
  "./motion/InertiaCalculator.h"
  "./motion/intersectionTest.cpp"
  "./motion/intersectionTest.h"
	"./motion/Liegroup.cpp"
	"./motion/Liegroup.h"
	"./motion/GMBS/liegroup.cpp"
	"./motion/GMBS/liegroup.h"
	"./motion/GMBS/liegroup.inl"
	"./motion/GMBS/liegroup2.inl"
"./motion/BoneKinematics.cpp"
"./motion/BoneKinematics.h"
"./motion/ASFLoader.cpp"
"./motion/ASFLoader.h"
"./motion/VRMLloader.cpp"
"./motion/VRMLloader.h"
"./motion/BVHLoader.cpp"
"./motion/BVHLoader.h"
"./motion/ConstraintMarking.cpp"
"./motion/ConstraintMarking.h"
"./motion/FootPrint.cpp"
"./motion/FootPrint.h"
"./motion/FullbodyIK.cpp"
"./motion/FullbodyIK.h"
"./motion/FullbodyIK_MotionDOF.cpp"
"./motion/FullbodyIK_MotionDOF_MultiTarget_lbfgs.cpp"
"./motion/FullbodyIK_MotionDOF.h"
"./motion/IKSolver.cpp"
"./motion/IKSolver.h"
"./motion/LimbIKinfo.cpp"
"./motion/LimbIKinfo.h"
"./motion/LimbIKsolver.cpp"
"./motion/LimbIKsolver.h"
"./motion/LimbIKsolver2.cpp"
"./motion/LimbIKsolver2.h"
"./motion/HandIKsolver.cpp"
"./motion/HandIKsolver.h"
"./motion/COM_IKsolver.cpp"
"./motion/COM_IKsolver.h"
"./motion/Mesh.cpp"
"./motion/Mesh.h"
"./motion/Terrain.cpp"
"./motion/Terrain.h"
"./motion/Geometry.cpp"
"./motion/Geometry.h"
"./motion/ModelLoader.cpp"
"./motion/ModelLoader.h"
"./motion/Motion.cpp"
"./motion/Motion.h"
"./motion/MotionDOF.cpp"
"./motion/MotionDOF.h"
"./motion/MotionDOFinfo.h"
"./motion/MotionLoader.cpp"
"./motion/MotionLoader.h"
"./motion/MotionRetarget.cpp"
"./motion/MotionRetarget.h"
"./motion/MotionWrap.cpp"
"./motion/MotionWrap.h"
"./motion/MotionUtil.cpp"
"./motion/MotionUtil.h"
"./motion/node.cpp"
"./motion/node.h"
"./motion/nodestack.cpp"
"./motion/nodestack.h"
"./motion/OnlinePath2D.cpp"
"./motion/OnlinePath2D.h"
"./motion/Path2D.cpp"
"./motion/Path2D.h"
"./motion/postureip.cpp"
"./motion/postureip.h"
"./motion/Retarget.cpp"
"./motion/Retarget.h"
"./motion/version.cpp"
"./motion/version.h"
"./motion/viewpoint.cpp"
"./motion/viewpoint.h"
"./motion/VRMLexporter.cpp"
"./motion/VRMLexporter.h"
"./motion/IK_sdls/NodeWrap.cpp"
"./motion/IK_sdls/NodeWrap.h"
"./motion/IK_sdls/Jacobian.cpp"
"./motion/IK_sdls/Jacobian.h"
"./motion/IK_sdls/LinearR3.cpp"
"./motion/IK_sdls/LinearR3.h"
"./motion/IK_sdls/MatrixRmn.cpp"
"./motion/IK_sdls/MatrixRmn.h"
"./motion/IK_sdls/Misc.cpp"
"./motion/IK_sdls/Node.cpp"
"./motion/IK_sdls/Node.h"
"./motion/IK_sdls/Tree.cpp"
"./motion/IK_sdls/Tree.h"
"./motion/IK_sdls/VectorRn.cpp"
"./motion/IK_sdls/VectorRn.h"
"./motion/IK_lbfgs/lbfgs.cpp"
# following files are extracted from bullet-2.68 and have been heavily modified.
	motion/gjk/btGjkEpa2.cpp 
	motion/gjk/btGjkEpa.cpp 
	motion/gjk/btCollisionShape.cpp
	motion/gjk/btConvexShape.cpp
	motion/gjk/btConvexInternalShape.cpp
	motion/gjk/btSphereShape.cpp
	motion/gjk/btPolyhedralConvexShape.cpp
	motion/gjk/btConvexHullShape.cpp 
	motion/gjk/btCollisionObject.cpp 
	motion/gjk/btVoronoiSimplexSolver.cpp 
	motion/gjk/btSubSimplexConvexCast.cpp 
	motion/gjk/btConvexCast.cpp 
	#motion/gjk/btBoxShape.cpp 
	#motion/gjk/btMinkowskiSumShape.cpp 
)



set(math
"./math/matrix3.cpp"
"./math/matrix3.h"
"./math/matrix4.cpp"
"./math/matrix4.h"
"./math/matrixn.cpp"
"./math/matrixn.h"
"./math/vector3.cpp"
"./math/vector3.h"
"./math/tvector.cpp"
"./math/tvector.h"
"./math/vectorn.cpp"
"./math/vectorn.h"
"./math/tensor.hpp"
"./math/floatvec.cpp"
"./math/floatvec.h"
"./math/quater.cpp"
"./math/quater.h"
"./math/transf.cpp"
"./math/transf.h"
"./math/quaterN.cpp"
"./math/quaterN.h"
"./math/bitVectorN.cpp"
"./math/bitVectorN.h"
"./math/vector3N.cpp"
"./math/vector3N.h"
"./math/dualQuaternion.cpp"
"./math/dualQuaternion.h"
"./math/optimize.cpp"
"./math/optimize.h"
"./math/conversion.cpp"
"./math/conversion.h"
"./math/hyperMatrixN.cpp"
"./math/hyperMatrixN.h"
"./math/interval.cpp"
"./math/interval.h"
"./math/intervals.cpp"
"./math/intervals.h"
"./math/intervalN.cpp"
"./math/intervalN.h"
#"./math/analyticSolver.cpp"
#"./math/analyticSolver.h"
"./math/Operator.cpp"
"./math/Operator.h"
"./math/Operator_NR.cpp"
"./math/Operator_NR.h"
"./math/OperatorQuater.cpp"
"./math/OperatorQuater.h"
"./math/OperatorStitch.cpp"
"./math/OperatorStitch.h"
"./math/BSpline.cpp"
"./math/BSpline.h"
"./math/NumericalSpline.cpp"
"./math/NumericalSpline.h"
"./math/DynamicTimeWarping.cpp"
"./math/DynamicTimeWarping.h"
"./math/Filter.cpp"
"./math/Filter.h"
"./math/Metric.cpp"
"./math/Metric.h"
)


set(dependency_support
"./math/dependency_support/gmm.cpp")


set(image
"./image/Image.cpp"
"./image/Image.h"
"./image/ImagePixel.cpp"
"./image/ImagePixel.h"
"./image/ImageProcessor.cpp"
"./image/ImageProcessor.h"
"./image/DrawChart.cpp"
"./image/DrawChart.h"
)

set(GJK_EPA_2d
	./math/GJK_EPA_2D/ConcaveShape.cpp
	./math/GJK_EPA_2D/ConvexShape.cpp
	./math/GJK_EPA_2D/EPACalculator.cpp
	./math/GJK_EPA_2D/GJKCalculator.cpp
	./math/GJK_EPA_2D/IShape.cpp
	./math/GJK_EPA_2D/Math.cpp
	./math/GJK_EPA_2D/ConcaveShape.h
	./math/GJK_EPA_2D/ConvexShape.h
	./math/GJK_EPA_2D/EPACalculator.h
	./math/GJK_EPA_2D/GJKCalculator.h
	./math/GJK_EPA_2D/IShape.h
	./math/GJK_EPA_2D/MyMath.h
	)

set(src_baselib
${math} 
${image} 
${motion}
${utility}
${GJK_EPA_2d}
)

include_directories (${TAESOOLIB_DIR}/dependencies/ezc3d/include)
set(src_ezc3d
	${TAESOOLIB_DIR}/dependencies/ezc3d/src/math/Matrix.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/math/Matrix33.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/math/Matrix44.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/math/Matrix66.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/math/Vector3d.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/math/Vector6d.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Analogs.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/AnalogsInfo.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/AnalogsSubframe.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Channel.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Data.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/DataStartInfo.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/ezc3d.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Frame.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Group.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Header.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Parameter.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Parameters.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Point.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Points.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/PointsInfo.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Rotation.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/Rotations.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/RotationsInfo.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/RotationsSubframe.cpp
    ${TAESOOLIB_DIR}/dependencies/ezc3d/src/modules/ForcePlatforms.cpp
	)

#add_subdirectory(${TAESOOLIB_DIR}/dependencies/ezc3d ./ezc3d)

if (WIN32)
add_library(BaseLib 
	${src_baselib}
	${src_ezc3d}
)

target_link_libraries(BaseLib
	PRIVATE
	${IMAGE_LIBS}
)
else()
	if(useShared)
		add_library(BaseLib SHARED
			${src_baselib}
			${src_ezc3d}
			)
	else()
		add_library(BaseLib
			${src_baselib}
			${src_ezc3d}
			)
	endif()
	target_link_libraries(BaseLib ${IMAGE_LIBS})
endif()

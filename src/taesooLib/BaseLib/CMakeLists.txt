# -*- mode: cmake; -*-
# The name of our project is "BaseLib".  CMakeLists files in this project can
# refer to the root source directory of the project as ${BaseLib_SOURCE_DIR} and
# to the root binary directory of the project as ${BaseLib_BINARY_DIR}.

project(BaseLib)

set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Samples")
set (TAESOOLIB_DIR "${CMAKE_SOURCE_DIR}/../")
include(Common_baselib) 
# Make sure the compiler can find include files from our Hello library.
include_directories (${BaseLib_SOURCE_DIR})
include_directories (${BaseLib_SOURCE_DIR}/unix)

if(WIN32)
   	if(NoDevil)
		add_definitions(-DNO_DEVIL)
		add_definitions(-DNO_FREEIMAGE)
		message("nodevil?okay. many functionalities won't work.")
	else()
	include_directories (
		${BaseLib_SOURCE_DIR}/../../dependencies/DevIL-SDK-x86-1.7.8/include
		${BaseLib_SOURCE_DIR}/../dependencies/libxml2-2.7.3.win32/include
		${BaseLib_SOURCE_DIR}/image/FreeImage/Dist
		)
	link_directories (
		${BaseLib_SOURCE_DIR}/image/FreeImage/Dist
		${BaseLib_SOURCE_DIR}/../../dependencies/DevIL-SDK-x86-1.7.8/lib
		)
	endif()
		
else()
   	if(NoDevil)
		add_definitions(-DNO_DEVIL)
		add_definitions(-DNO_FREEIMAGE)
		message("nodevil?okay. many functionalities won't work.")
		# warp cluster
		include_directories (
			${BaseLib_SOURCE_DIR}/../dependencies/FreeImage/Dist
			)
	endif()
	include_directories(
		/usr/include/libxml2
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
"./utility/BinaryFile.cpp"
"./utility/BinaryFile.h"
"./utility/TGL.cpp"
"./utility/TGL.h"
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

)

LIST(APPEND utility "./utility/checkPoints.cpp")

set(motion

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
"./math/analyticSolver.cpp"
"./math/analyticSolver.h"
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
#"./math/nr/nr_all.cpp"
#"./math/nr/nr_all2.cpp"
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


set(src_baselib
${math} 
${image} 
${motion}
${utility}
)


if (WIN32)
add_library(BaseLib 
	${src_baselib}
)

target_link_libraries(BaseLib
	${IMAGE_LIBS}
)
else()
	if(useShared)
		add_library(BaseLib SHARED
			${src_baselib}
			)
	else()
		add_library(BaseLib
			${src_baselib}
			)
	endif()
	if (APPLE)
target_link_libraries(BaseLib
			${IMAGE_LIBS})
	else()
		if((EXISTS "/usr/lib/OGRE/libOgreOverlay.so")
			OR ( EXISTS "/usr/lib/x86_64-linux-gnu/libOgreOverlay.so") )
			target_link_libraries(BaseLib 
				${IMAGE_LIBS}
				#boost_system
				)
		else()
			target_link_libraries(BaseLib 
				${IMAGE_LIBS}
)
		endif()
	endif()
endif()

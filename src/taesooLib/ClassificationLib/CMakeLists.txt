
# -*- mode: cmake; -*-
project(ClassificationLib)

# Make sure the compiler can find include files from our Hello library.
set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Samples")
include(Common_baselib) 
include(Common_mainlib) 
include_directories (
	.
	)

set(src 
./math/KNearestInterpolationFast.cpp
./math/KNearestInterpolationFast.h
./math/Function.cpp
./math/LCBInterploator.cpp
./math/BijectiveFunction.cpp
./math/svm/svm.cpp
./math/FuzzyCluster.cpp
./math/FuzzyCluster.h
./math/Interpolation.cpp
./math/clust/subcluster.h
./math/clust/clust_io.cpp
./math/clust/classify.cpp
./math/clust/alloc_util.cpp
./math/clust/invert.cpp
./math/clust/eigen.cpp
./math/clust/alloc_util.h
./math/clust/classify_util.cpp
./math/clust/clust_io.h
./math/clust/clust_util.cpp
./math/clust/clust_defs.h
./math/clust/subcluster.cpp
./math/clust/clust.cpp
./math/clust/SplitClasses.cpp
./math/clust/clust_invert.cpp
./math/clust/classify_util.h
./math/clust/clust.h
./math/clust/clust_util.h
./math/cluster.cpp
./math/cluster.h
./math/statistics.cpp
./math/PCA.cpp
./math/SVMWrap.cpp
./math/tpros/tpros_nrutil.cpp
./math/tpros/tpros_lib.cpp
./math/tpros/macopt.cpp
./math/tpros/stdafx.cpp
./math/tpros/Tpros.cpp
./math/tpros/rand2.cpp
./math/GnuPlot.cpp
	)
set(src_gui
	)
set(luna_script "${ClassificationLib_SOURCE_DIR}")
set(LUNA_GEN ${ClassificationLib_SOURCE_DIR}/../MainLib/WrapperLua/luna_gen.lua)

add_custom_command(
	OUTPUT "${luna_script}/luna_classification.cpp"
	DEPENDS "${luna_script}/luna_classification.lua" "${LUNA_GEN}" 
	PRE_BUILD
	COMMAND lua "${LUNA_GEN}" "${luna_script}/luna_classification.lua"
)
list(APPEND src "${luna_script}/luna_classification.cpp")

add_subdirectory(math/ann_1.1.2/src )
if(NO_GUI)
	if(useShared)
		add_library(ClassificationLib SHARED ${src})
	else()
		add_library(ClassificationLib ${src})
	endif()
else()
	if(useShared)
		add_library(ClassificationLib SHARED ${src} ${src_gui})
	else()
		add_library(ClassificationLib ${src} ${src_gui})
	endif()
endif()
target_link_libraries(ClassificationLib
BaseLib
MainLib
ann
)

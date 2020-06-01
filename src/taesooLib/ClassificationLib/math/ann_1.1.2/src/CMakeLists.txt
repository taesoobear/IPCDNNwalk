
project (ann)
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
	add_definitions(-DAMD64)
endif()
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release" CACHE STRING
       "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
       FORCE)
endif(NOT CMAKE_BUILD_TYPE)
if(MSVC)
	add_definitions(-DDLL_EXPORTS)
	#	no longer depends on cached value because it causes some problems.
	#set(CMAKE_CXX_FLAGS_DEBUG "/D_DEBUG /DWIN32 /D_WINDOWS /WX /wd4355 /wd4275 /wd4819 /wd4251 /wd4193" CACHE STRING "Debug compiler flags" FORCE)
	# /Zi: generate debug info
	# /Od: no code optimization
	# /MDd: multi-threaded debug dll
	set(CMAKE_CXX_FLAGS_DEBUG "/D_DEBUG /DWIN32 /D_WINDOWS /MDd /Zi /Od /wd4355 /wd4275 /wd4819 /wd4251 /wd4193 /EHsc" CACHE STRING "Debug compiler flags" FORCE)
	set(CMAKE_CXX_FLAGS_RELEASE "/MD /O2 /Ob2 /D NDEBUG /wd4355 /wd4275 /wd4819 /wd4251 /wd4193 /EHsc" CACHE STRING "Release compiler flags" FORCE)
	#set(CMAKE_C_FLAGS_DEBUG "/D_DEBUG /DWIN32 /D_WINDOWS /WX /wd4355 /wd4275 /wd4819 /wd4251 /wd4193" CACHE STRING "C Debug compiler flags" FORCE)
	set(CMAKE_C_FLAGS_DEBUG "/D_DEBUG /DWIN32 /D_WINDOWS /MDd /Zi /Od /wd4355 /wd4275 /wd4819 /wd4251 /wd4193 /EHsc" CACHE STRING "C Debug compiler flags" FORCE)
	set(CMAKE_C_FLAGS_RELEASE "/MD /O2 /Ob2 /D NDEBUG /wd4355 /wd4275 /wd4819 /wd4251 /wd4193 /EHsc" CACHE STRING "C Release compiler flags" FORCE)
else()
	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -frounding-math")
endif()
if(UNIX)
	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -frounding-math")
endif()
include_directories (../include)

set(src
	ANN.cpp
	brute.cpp
	kd_tree.cpp
	kd_util.cpp
	kd_split.cpp
	kd_dump.cpp
	kd_search.cpp
	kd_pr_search.cpp
	kd_fix_rad_search.cpp 
	bd_tree.cpp
	bd_search.cpp
	bd_pr_search.cpp
	bd_fix_rad_search.cpp 
	perf.cpp
	kd_tree.h
	kd_split.h
	kd_util.h
	kd_search.h
	kd_pr_search.h
	kd_fix_rad_search.h
	pr_queue.h
	pr_queue_k.h
	)
if(useShared)
	add_library(ann SHARED ${src} )
else()
	add_library(ann ${src})
endif()

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
	add_definitions(-DAMD64)
endif()
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release" CACHE STRING
       "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
       FORCE)
endif(NOT CMAKE_BUILD_TYPE)
if(MSVC)
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
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -frounding-math -std=c++11")
endif()
if(UNIX)

	#collide with libsml
	#if(CMAKE_BUILD_TYPE MATCHES Release)
	#	add_definitions(-DNDEBUG)
	#endif()
	message("Unix OS")
	set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ftemplate-depth-100 -Wno-deprecated -std=c++11")
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(EIGEN eigen3 REQUIRED)
	pkg_check_modules(LUA lua5.1 REQUIRED)
	include_directories(${EIGEN_INCLUDE_DIRS})
	set(LUA_INCLUDE ${LUA_INCLUDE_DIRS})
	if (APPLE)
		set(LUA_LIB ${LUA_LIBRARIES}) # this is a static library
		set(HOMEBREW_DIR /opt/homebrew)

		set(DEFAULT_INCLUDE 
			${HOMEBREW_DIR}/include
			"/usr/local/include/"
		)
		link_directories(
			${HOMEBREW_DIR}/lib
			/usr/local/lib
			/opt/homebrew/lib
			)

	else()
		set(LUA_LIB lua51) # this is a shared library
	endif()
else()
	message("Windows OS")
	add_definitions(-D_SCL_SECURE_NO_DEPRECATE)
	# unused settings
	set(DEFAULT_INCLUDE )
	set(LUA_INCLUDE )
	set(LUA_LIB )
	set(EIGEN3_INCLUDE_DIR "${TAESOOLIB_DIR}/../windows_dependencies/eigen-3.4.0")
	include_directories (${EIGEN3_INCLUDE_DIR}	)
endif()

include_directories (	
	${DEFAULT_INCLUDE}
	${LUA_INCLUDE}
	)

if(WIN32)
	# settings are already in BaseLib/CMakeLists.txt 
else()
	if(NoDevil)
		if(NoFreeImage)
			set(IMAGE_LIBS )
		else()
			set(IMAGE_LIBS freeimage)
		endif()
	else()
		if(UseMPI)
			set(IMAGE_LIBS freeimage)
		else()
			set(IMAGE_LIBS
				IL
				ILU
				freeimage
				z
				jpeg
				)
		endif()
	endif()
endif()


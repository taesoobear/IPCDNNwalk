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
	set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ftemplate-depth-100 -msse2 -mfpmath=sse -Wno-deprecated -std=c++11")
	if (APPLE)
		set(OGRE_INCLUDE 
			"/usr/local/include/"
			/usr/local/include/eigen3
            "/Applications/OgreSDK/include/OGRE" 
			"/Applications/OgreSDK/include/OIS"
			"/Applications/OgreSDK/boost"
			"/Applications/OgreSDK/include/OGRE/RenderSystems/GL/OSX"
			"/Applications/OgreSDK/include/OGRE/RenderSystems/GL"
			)
		set(LUA_INCLUDE "/opt/local/include/lua-5.1" "/opt/local/include" "/usr/local/include/lua-5.1")
		if(EXISTS "/opt/local/lib/lua-5.1/liblua-5.1.a")
			set(LUA_LIB "/opt/local/lib/lua-5.1/liblua-5.1.a")
		else()
			set(LUA_LIB "/usr/local/lib/liblua.5.1.dylib")
		endif()
		if(EXISTS "/opt/local/lib/lua-5.1/liblua-5.1.a")
			link_directories( "/opt/local/lib/")
		else()
			link_directories( "/usr/local/lib/")
		endif()
	else()
		set(OGRE_INCLUDE "/usr/include/OGRE"
			"/usr/include/OIS"
			)
		if(UseLuaJit)
			set(LUA_INCLUDE 
				"/usr/include/luajit-2.0"
				"/usr/include/luajit-2.1"
				)
			set(LUA_LIB "luajit-5.1")
		else()
			set(LUA_INCLUDE 
				"/usr/include/lua5.1"
				"${TAESOOLIB_DIR}/../dependencies_mpi/lua-5.1.5/src"
				)
			if(EXISTS "/usr/lib64/liblua.so")
				set(LUA_LIB "lua")
			elseif(EXISTS "${TAESOOLIB_DIR}/../dependencies_mpi/lua-5.1.5/src/liblua.a")
				set(LUA_LIB "${TAESOOLIB_DIR}/../dependencies_mpi/lua-5.1.5/src/liblua.a")
			else()
				set(LUA_LIB "lua5.1")
			endif()
		endif()
	endif()
else()
	message("Windows OS")
	add_definitions(-D_SCL_SECURE_NO_DEPRECATE)
	add_definitions(-DMANUALLY_ADD_LIB)
	set(OGRE_INCLUDE "${TAESOOLIB_DIR}/../dependencies/OgreSDK_vc10_v1-8-1/include/OGRE")
	set(LUA_INCLUDE "${TAESOOLIB_DIR}/../dependencies/lua-5.1.5/src")
	set(LUA_LIB "${TAESOOLIB_DIR}/../dependencies/lua-5.1.5/src/lua51.lib")
endif()

include_directories (	
	/usr/include/eigen3
	${OGRE_INCLUDE}
	${LUA_INCLUDE}
	)

if(WIN32)
	if(NoDevil)
		set(IMAGE_LIBS )
	else()
	set(IMAGE_LIBS
		DevIL
		ILU
		freeimage
		)
	endif()
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

set(LUNA_GEN ${MainLib_SOURCE_DIR}/WrapperLua/luna_gen.lua)


INCLUDE(FindPkgConfig)
if(WIN32)
	include_directories(
		${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8
		${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win
		)
	set(FLTK_LIBS
		ws2_32.lib
		comctl32.lib
		#optimized ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Release/fltk.lib
		#debug ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Debug/fltk.lib
		# static build
		#${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win/lib/Release/fltk.lib
		#${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win/lib/Release/fltk_forms.lib
		#${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win/lib/Release/fltk_gl.lib
		#${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win/lib/Release/fltk_images.lib
		#${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win/lib/Release/fltk_jpeg.lib
		#${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win/lib/Release/fltk_png.lib
		#${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win/lib/Release/fltk_z.lib
		# shared build
		${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win/lib/Release/libfltk.lib
		${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win/lib/Release/libfltk_png.lib
		)
	set(OGRE_LIBS
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Release/OgreNextAtmosphere.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Release/OgreNextHlmsPbs.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Release/OgreNextHlmsUnlit.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Release/OgreNextMain.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Release/OgreNextMeshLodGenerator.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Release/OgreNextOverlay.lib
		#${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Release/OgreNextProperty.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Release/OgreNextSceneFormat.lib
		)
	set(OGRE_DEBUG_LIBS
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Debug/OgreNextAtmosphere_d.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Debug/OgreNextHlmsPbs_d.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Debug/OgreNextHlmsUnlit_d.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Debug/OgreNextMain_d.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Debug/OgreNextMeshLodGenerator_d.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Debug/OgreNextOverlay_d.lib
		#${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Debug/OgreNextProperty_d.lib
		${MainLib_SOURCE_DIR}/../../ogre-next3/build/lib/Debug/OgreNextSceneFormat_d.lib
		)
		set(OGRENEXT_PATH ${MainLib_SOURCE_DIR}/../../ogre-next3)
		set(OGRE_INCLUDE_DIRS
			${OGRENEXT_PATH}/Components/Overlay/include
			${OGRENEXT_PATH}/OgreMain/include 
			${OGRENEXT_PATH}/OgreMain/include/Animation
			${OGRENEXT_PATH}/OgreMain/include/Math/Array
			${OGRENEXT_PATH}/Components/Hlms/Common/include
			${OGRENEXT_PATH}/Components/Hlms/Pbs/include
			${OGRENEXT_PATH}/Components/Hlms/Unlit/include
			${OGRENEXT_PATH}/build/include
		)

	include_directories(
		${OGRE_INCLUDE_DIRS}
		${EIGEN_INCLUDE_DIRS}
		${LUA_INCLUDE_DIRS}
	)
else()
	if(APPLE)
		#set(OGRENEXT_PATH /opt/homebrew/include/ogre-2.3)
		set(OGRENEXT_PATH /opt/homebrew/include/ogre-3)
		link_directories(
			/usr/local/lib
			${HOMEBREW_DIR}/lib/ogre-3
		)
		set(FLTK_LIBS
			fltk_images
			fltk
			fltk_gl
		)
		if(CMAKE_BUILD_TYPE MATCHES Release)
			set(OGRE_LIBS
				OIS
				libOgreNextMain.4.0.dylib
				libOgreNextOverlay.4.0.dylib
				libOgreNextHlmsPbs.4.0.dylib
				libOgreNextHlmsUnlit.4.0.dylib
				libOgreNextAtmosphere.4.0.dylib
				#/opt/local/Library/Frameworks/Ogre.framework
				#/opt/local/Library/Frameworks/OgreOverlay.framework
				/System/Library/Frameworks/CoreFoundation.framework
				/System/Library/Frameworks/IOKit.framework
				/System/Library/Frameworks/Cocoa.framework
				#/System/Library/Frameworks/ScreenCaptureKit.framework
			)
		else()
			set(OGRE_LIBS
				OIS
				${HOMEBREW_DIR}/lib/ogre-3/Debug/libOgreNextMain.4.0.dylib
				${HOMEBREW_DIR}/lib/ogre-3/Debug/libOgreNextOverlay.4.0.dylib
				${HOMEBREW_DIR}/lib/ogre-3/Debug/libOgreNextHlmsPbs.4.0.dylib
				${HOMEBREW_DIR}/lib/ogre-3/Debug/libOgreNextHlmsUnlit.4.0.dylib
				${HOMEBREW_DIR}/lib/ogre-3/Debug/libOgreNextAtmosphere.4.0.dylib
				#/opt/local/Library/Frameworks/Ogre.framework
				#/opt/local/Library/Frameworks/OgreOverlay.framework
				/System/Library/Frameworks/CoreFoundation.framework
				/System/Library/Frameworks/IOKit.framework
				/System/Library/Frameworks/Cocoa.framework
			)
		endif()
		set(OGRE_INCLUDE_DIRS
			${OGRENEXT_PATH}/Components/Overlay/include
			${OGRENEXT_PATH}/OgreMain/include 
			${OGRENEXT_PATH}/OgreMain/include/Animation
			${OGRENEXT_PATH}/OgreMain/include/Math/Array
			${OGRENEXT_PATH}/Components/Hlms/Common/include
			${OGRENEXT_PATH}/Components/Hlms/Pbs/include
			${OGRENEXT_PATH}/Components/Hlms/Unlit/include
			${OGRENEXT_PATH}/build/include 
		)

	else()
		PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)
		# linux
		set(OGRENEXT_PATH /usr/local/include/OGRE-Next)
		link_directories(
			/usr/local/lib/OGRE-Next
		)
		set(FLTK_LIBS
			fltk_images
			fltk
			fltk_gl
			X11
		)
		set(OGRE_INCLUDE_DIRS
			${OGRENEXT_PATH}/Overlay
			${OGRENEXT_PATH}
			${OGRENEXT_PATH}/Hlms/Common
			${OGRENEXT_PATH}/Hlms/Pbs
			${OGRENEXT_PATH}/Hlms/Unlit
			${SDL2_INCLUDE_DIRS}
			"/usr/local/include/"
		)
		if(CMAKE_BUILD_TYPE MATCHES Release)
			set(OGRE_LIBS
				OIS
				libOgreNextMain.so.4.0
				libOgreNextMeshLodGenerator.so.4.0
				libOgreNextOverlay.so.4.0
				libOgreNextHlmsPbs.so.4.0
				libOgreNextHlmsUnlit.so.4.0
				libOgreNextAtmosphere.so.4.0
				SDL2
			)
		else()
			set(OGRE_LIBS
				OIS
				libOgreNextMain_d.so.4.0
				libOgreNextMeshLodGenerator_d.so.4.0
				libOgreNextOverlay_d.so.4.0
				libOgreNextHlmsPbs_d.so.4.0
				libOgreNextHlmsUnlit_d.so.4.0
				libOgreNextAtmosphere_d.so.4.0
				SDL2
			)
		endif()
	endif()
	include_directories(
		${OGRE_INCLUDE_DIRS}
		${EIGEN_INCLUDE_DIRS}
		${LUA_INCLUDE_DIRS}
		/usr/include/ois
		/usr/local/include/ois
		/usr/local/include/eigen3
		)
endif()

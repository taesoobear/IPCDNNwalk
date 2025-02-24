set(LUNA_GEN ${MainLib_SOURCE_DIR}/WrapperLua/luna_gen.lua)


INCLUDE(FindPkgConfig)
if(WIN32)
	include_directories(
		${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8
		${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8/build_win
		${MainLib_SOURCE_DIR}/../../windows_dependencies/lua-5.1.5/src
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
		)
else()
	if(APPLE)
		set(OGRENEXT_PATH /opt/homebrew/include/ogre-2.3)
		link_directories(
			/usr/local/lib
		)
		set(FLTK_LIBS
			fltk_images
			fltk
			fltk_gl
		)
		if(CMAKE_BUILD_TYPE MATCHES Release)
			set(OGRE_LIBS
				OIS
				${HOMEBREW_DIR}/lib/libOgreMain.2.3.4.dylib
				${HOMEBREW_DIR}/lib/libOgreOverlay.2.3.4.dylib
				${HOMEBREW_DIR}/lib/libOgreHlmsPbs.2.3.4.dylib
				${HOMEBREW_DIR}/lib/libOgreHlmsUnlit.2.3.4.dylib
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
				${HOMEBREW_DIR}/lib/Debug/libOgreMain.2.3.4.dylib
				${HOMEBREW_DIR}/lib/Debug/libOgreOverlay.2.3.4.dylib
				${HOMEBREW_DIR}/lib/Debug/libOgreHlmsPbs.2.3.4.dylib
				${HOMEBREW_DIR}/lib/Debug/libOgreHlmsUnlit.2.3.4.dylib
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
		set(OGRENEXT_PATH /usr/local/include/OGRE)
		link_directories(
			/usr/local/lib/OGRE
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
		)
		set(OGRE_INCLUDE_DIRS
			${OGRENEXT_PATH}/Overlay
			${OGRENEXT_PATH}
			${OGRENEXT_PATH}/Hlms/Common
			${OGRENEXT_PATH}/Hlms/Pbs
			${OGRENEXT_PATH}/Hlms/Unlit
			${SDL2_INCLUDE_DIRS}
		)
		if(CMAKE_BUILD_TYPE MATCHES Release)
			set(OGRE_LIBS
				OIS
				libOgreMain.so.2.3.4
				libOgreOverlay.so.2.3.4
				libOgreHlmsPbs.so.2.3.4
				libOgreHlmsUnlit.so.2.3.4
				SDL2
			)
		else()
			set(OGRE_LIBS
				OIS
				libOgreMain_d.so.2.3.4
				libOgreOverlay_d.so.2.3.4
				libOgreHlmsPbs_d.so.2.3.4
				libOgreHlmsUnlit_d.so.2.3.4
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

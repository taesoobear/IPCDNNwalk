set(LUNA_GEN ${MainLib_SOURCE_DIR}/WrapperLua/luna_gen.lua)
if(WIN32)
	include_directories(
		${MainLib_SOURCE_DIR}/../../windows_dependencies/ogre-13.4.4/include/Ogre
		${MainLib_SOURCE_DIR}/../../windows_dependencies/fltk-1.3.8
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
		"${MainLib_SOURCE_DIR}/../../windows_dependencies/ogre-13.4.4/lib/OgreMain.lib"
		"${MainLib_SOURCE_DIR}/../../windows_dependencies/ogre-13.4.4/lib/OgreOverlay.lib"
		"${MainLib_SOURCE_DIR}/../../windows_dependencies/ogre-13.4.4/lib/OgreBites.lib"
		)
else()
	include_directories(
		/usr/local/include/OGRE
		/usr/include/ois
		/usr/local/include/ois
		/usr/local/include/eigen3
	)
	if(APPLE)
		set(FLTK_LIBS
			fltk_images
			fltk
			fltk_gl
			)
		set(OGRE_LIBS
			OIS
			/Applications/OgreSDK/lib/macosx/Release/Ogre.framework
			/Applications/OgreSDK/lib/macosx/Release/OgreOverlay.framework
			/Applications/OgreSDK/lib/macosx/Release/OgreBites.framework
			#/opt/local/Library/Frameworks/Ogre.framework
			#/opt/local/Library/Frameworks/OgreOverlay.framework
			/System/Library/Frameworks/CoreFoundation.framework
			/System/Library/Frameworks/IOKit.framework
			/System/Library/Frameworks/Cocoa.framework
			)
	else()
		# linux
		set(FLTK_LIBS
			fltk_images
			fltk
			fltk_gl
			X11
			)
		if((EXISTS "/usr/local/lib/libOgreBites.so")
			OR ( EXISTS "/usr/lib/x86_64-linux-gnu/libOgreBites.so") 
			)
			# Ogre 1.12 or 13(manually installed)
			set(OGRE_LIBS
				OgreMain
				boost_system
				OgreOverlay
				OgreBites
				OgreRTShaderSystem
				OIS
				)
		elseif((EXISTS "/usr/lib/OGRE/libOgreOverlay.so")
			OR ( EXISTS "/usr/lib/x86_64-linux-gnu/libOgreOverlay.so") 
			OR ( EXISTS "/usr/lib/i386-linux-gnu/libOgreOverlay.so") )
			# Ogre 1.9 
			set(OGRE_LIBS
				OgreMain
				boost_system
				OgreOverlay
				OIS
				)
		else()
			# Ogre 1.7
			set(OGRE_LIBS
				OgreMain
				OIS
				)
		endif()
	endif()
endif()

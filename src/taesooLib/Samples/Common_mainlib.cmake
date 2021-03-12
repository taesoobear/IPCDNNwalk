set(LUNA_GEN ${MainLib_SOURCE_DIR}/WrapperLua/luna_gen.lua)
if(WIN32)
include_directories(
${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost
${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/include/OIS
${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10
)
link_directories(
${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/lib
${BaseLib_SOURCE_DIR}/../../dependencies/DevIL-SDK-x86-1.7.8/lib
${BaseLib_SOURCE_DIR}/image/FreeImage/Dist
${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/lib
${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/test
)
set(FLTK_LIBS
ws2_32.lib
comctl32.lib
#optimized ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Release/fltk.lib
#optimized ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Release/fltk_forms.lib
#optimized ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Release/fltk_gl.lib
#optimized ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Release/fltk_images.lib
#optimized ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Release/fltk_png.lib
#optimized ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Release/fltk_zlib.lib
#debug ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Debug/fltk.lib
#debug ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Debug/fltk_forms.lib
#debug ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Debug/fltk_gl.lib
#debug ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Debug/fltk_images.lib
#debug ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Debug/fltk_png.lib
#debug ${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/bin/Debug/fltk_zlib.lib
${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/lib/fltk.lib
${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/lib/fltkforms.lib
${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/lib/fltkgl.lib
${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/lib/fltkimages.lib
${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/lib/fltkpng.lib
${MainLib_SOURCE_DIR}/../../dependencies/fltk-1.1.10/lib/fltkz.lib
	)
set(OGRE_LIBS
	${LUA_LIB}
	optimized "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/lib/release/OgreMain.lib"
	optimized "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/lib/release/OgreOverlay.lib"
	#optimized "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_date_time-vc100-mt-1_49.lib"
	#optimized "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_thread-vc100-mt-1_49.lib"
	optimized "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_date_time-vc140-mt-1_61.lib"
	optimized "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_system-vc140-mt-1_61.lib"
	optimized "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_thread-vc140-mt-1_61.lib"
	optimized "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_chrono-vc140-mt-1_61.lib"
	optimized "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/lib/release/OIS.lib"
	debug "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/lib/debug/OgreMain_d.lib"
	debug "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/lib/debug/OgreOverlay_d.lib"
	#debug "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_date_time-vc100-mt-gd-1_49.lib"
	#debug "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_thread-vc100-mt-gd-1_49.lib"
	debug "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_date_time-vc140-mt-gd-1_61.lib"
	debug "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_system-vc140-mt-gd-1_61.lib"
	debug "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_thread-vc140-mt-gd-1_61.lib"
	debug "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/boost/lib/libboost_chrono-vc140-mt-gd-1_61.lib"
	debug "${MainLib_SOURCE_DIR}/../../dependencies/OgreSDK_vc10_v1-8-1/lib/release/OIS.lib"
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
else()
	set(FLTK_LIBS
		fltk_images
		fltk
		fltk_gl
		X11
		)
endif()
if(APPLE)
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
if((EXISTS "/usr/local/lib/libOgreBites.so")
	OR ( EXISTS "/usr/lib/x86_64-linux-gnu/libOgreBites.so") 
	)
	# Ogre 1.12 (manually installed)
	set(OGRE_LIBS
		OgreMain
		#boost_system
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
		#boost_system
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
endif() #APPLE
 
endif()

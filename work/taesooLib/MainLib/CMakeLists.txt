# -*- mode: cmake; -*-
project(MainLib)

set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Samples")
include(Common_baselib) 
include(Common_mainlib) 


if(SKELTRACK)
	message("MAinLib SKELTRACK enabled")
	include(Common_skeltrack)
endif()

set(NoGSL TRUE)

if(WIN32)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../windows_dependencies/lua-5.1.5/src ./build_lua)
else()
	if (BUILD_CONSOLE )
	else()
		include_directories(
			/usr/local/include/OGRE
			/usr/include/ois
			)
	endif()
endif()
include_directories (
  ${MainLib_SOURCE_DIR}
  ${MainLib_SOURCE_DIR}/BaseLib
  ${LUA_INCLUDE}
  ${MainLib_SOURCE_DIR}/../BaseLib/unix
  )
# Make sure the compiler can find include files from our Hello library.

set(src 
	OgreFltk/BaseSystem.h
	OgreFltk/BaseSystem.cpp
	OgreFltk/ShadowMapFromCodeGameState.cpp
	OgreFltk/ShadowMapFromCodeGameState.h
	OgreFltk/StaticPluginLoader.cpp
	OgreFltk/StaticPluginLoader.h
  "./OgreFltk/Joystick.cpp"
  "./MainLib.h"
  "./MainLib.cpp"
  #"./Ogre/AssimpLoader.h" use ogre-assimp.git instead.
  #"./Ogre/AssimpLoader.cpp"
  "./OgreFltk/AlzzaPostureIP.cpp"
  "./OgreFltk/AnimationObject.cpp"
  "./OgreFltk/fastCapture.cpp"
  "./OgreFltk/FlLayout.cpp"
  "./OgreFltk/FlLayouts.cpp"
  "./OgreFltk/FltkAddon.cpp"
  "./OgreFltk/LimbIKsolverLua.cpp"
  "./OgreFltk/FltkAddon.h"
  "./OgreFltk/FltkRenderer.cpp"
  "./OgreFltk/framemoveobject.cpp"
  "./OgreFltk/GlobalUI.cpp"
  "./OgreFltk/GlobalUI.h"
  "./OgreFltk/interpolator.cpp"
  "./OgreFltk/Loader.cpp"
  "./OgreFltk/MotionManager.cpp"
  "./OgreFltk/MotionPanel.cpp"
  "./OgreFltk/MotionPanel.h"
  "./OgreFltk/FltkScrollPanel.h"
  "./OgreFltk/pldprimskin.cpp"
  "./OgreFltk/pldprimskin.h"
  "./OgreFltk/pldprimskin_impl.h"
  "./OgreFltk/RE.cpp"
  "./OgreFltk/renderer.cpp"
  "./OgreFltk/timesensor.cpp"
  "./OgreFltk/TraceManager.cpp"
  "./OgreFltk/VRMLloader.cpp"
  "./OgreFltk/VRMLloaderView.cpp"
  "./OgreFltk/VRMLloaderView.h"
  "./OgreFltk/PointClouds.cpp"
  "./OgreFltk/PointClouds.h"
  )

if(useShared)
	list(REMOVE_ITEM src 
  )
endif()

set(Ogre
  "./OgreFltk/Circle.cpp"
  "./OgreFltk/dynamicRenderable.cpp"
  "./OgreFltk/Line3D.cpp"
  "./OgreFltk/LineSegment.cpp"
  "./OgreFltk/LineStrip.cpp"
  "./OgreFltk/Mesh.cpp"
  "./OgreFltk/Mesh.h"
  "./OgreFltk/MovableText.cpp"
  "./OgreFltk/objectList.cpp"
  )

set(console
  "./console/dummies.cpp"
  "./console/traceManager.cpp"
  )

if(WIN32)
endif()

LIST(APPEND src ${Ogre})
LIST(APPEND src ${console})




set(luna_script "${MainLib_SOURCE_DIR}/WrapperLua")
set(LUNA_GEN ${MainLib_SOURCE_DIR}/WrapperLua/luna_gen.lua)

function (add_luna_baselib x)
	message( "${luna_script}/${x}")
add_custom_command(
	OUTPUT "${luna_script}/${x}"
	DEPENDS "${luna_script}/luna_baselib.lua" "${luna_script}/luna_mainlib.lua" "${LUNA_GEN}" 
	PRE_BUILD
	COMMAND lua "${LUNA_GEN}" "${luna_script}/luna_baselib.lua"
	)
endfunction()
add_luna_baselib("luna_baselib.cpp")
add_luna_baselib("luna_mainlib.cpp")
add_luna_baselib("luna_baselib.h")
add_luna_baselib("luna_mainlib.h")


LIST(APPEND src 
	"./WrapperLua/luna.h"
	"./WrapperLua/luna.cpp"
	"./WrapperLua/LUAwrapper.cpp"
	"./WrapperLua/LUAwrapper.h"
	"./WrapperLua/mainliblua_wrap.cpp"
	"./WrapperLua/mainliblua_wrap.h"
	# generated files should also be added here
	"${luna_script}/luna_baselib.cpp"
	"${luna_script}/luna_mainlib.cpp"
	"${luna_script}/luna_baselib.h"
	"${luna_script}/luna_mainlib.h"
	"./WrapperLua/FlLayoutLua.cpp"
	"./WrapperLua/FlLayoutLua.h"
  "./WrapperLua/ScriptWin.cpp"
  "./WrapperLua/ScriptWin.h"
  #  "./WrapperLua/ThreadedScript.cpp"
  #"./WrapperLua/ThreadedScript.h"
	)

if(WIN32)
LIST(APPEND src 
	"./OgreFltk/Fl_Native_File_Chooser-0.84/Fl_Native_File_Chooser.cxx"
	"./OgreFltk/Fl_Native_File_Chooser-0.84/Fl_Native_File_Chooser_WIN32.cxx"
	)
elseif(APPLE)
	if (BUILD_CONSOLE )
	else()
		LIST(APPEND src 
			OgreFltk/mac/OgreFltk.mm
			OgreFltk/OSXUtils.h
			OgreFltk/OSXUtils.mm
		)
	endif()
endif()

if(useShared)
  add_library(MainLib SHARED ${src})
  if (BUILD_CONSOLE )
  else()
  target_link_libraries(MainLib
	  ${OGRE_LIBS}
	  ${FLTK_LIBS}
    )
endif()
else()
	add_library(MainLib ${src})
endif()



IF(UNIX)
  target_link_libraries(MainLib
    BaseLib
	#${Luabind_LIB}
	${LUA_LIB}
    )
	IF(NOT NoGSL)
		target_link_libraries(MainLib
		gsl
		gslcblas
		)
	endif()
elseif(WIN32)
	target_link_libraries(MainLib
		BaseLib
		#LuaBind
		lua51
		)

ELSE()
  target_link_libraries(MainLib
    BaseLib
	#LuaBind
    )
ENDIF()


if(SKELTRACK)
	include_directories(
		${SKELTRACK_INC}
		)
	target_link_libraries(MainLib
		${SKELTRACK_LIB}
		)
endif()

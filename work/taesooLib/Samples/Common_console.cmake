set(NoDevil TRUE)
set(NoLapack TRUE)
set(NoUMFpack TRUE)

# NoAIST, NoGMBSsim, NoUTsim settings are in PhysicsLib/CMakeLists.txt
include(Common_baselib) 
add_subdirectory(${TAESOOLIB_DIR}/BaseLib "${TAESOOLIB_DIR}/BaseLib/build_console_${CMAKE_BUILD_TYPE}" )

add_definitions(-DSEGA)				
add_definitions(-DNO_GUI -DNO_OGRE)  
set(NO_GUI TRUE)
add_subdirectory(${TAESOOLIB_DIR}/MainLib "${TAESOOLIB_DIR}/MainLib/build_console_${CMAKE_BUILD_TYPE}" )
include(Common_mainlib) 

# PhysicsLib settings
set(NoAISTsim TRUE) # use TRL_LCP instead!
set(NoGMBSsim TRUE) # use TRL_LCP instead!
set(NoUTsim TRUE) # use TRL_LCP instead!
set(NoBulletSim TRUE) # use TRL_LCP instead!

set(TAESOO_LIBS
	BaseLib 
	${LUA_LIB}
	MainLib 
	#blas
	#LibLinearMath
	#LibBulletCollision
	PhysicsLib 
	ClassificationLib
	)

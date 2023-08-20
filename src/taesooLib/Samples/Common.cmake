set(NoDevil TRUE)
set(NoFreeImage FALSE)
set(NoLapack TRUE)
set(NoUMFpack TRUE)
if(WIN32)
	set(NoFreeImage TRUE)
endif()

# all important dependencies are set in Common_baselib.cmake
# NoAIST, NoGMBSsim, NoUTsim settings are in PhysicsLib/CMakeLists.txt
include(Common_baselib) 
add_subdirectory(${TAESOOLIB_DIR}/BaseLib "${TAESOOLIB_DIR}/BaseLib/build_${CMAKE_BUILD_TYPE}" )
add_subdirectory(${TAESOOLIB_DIR}/MainLib "${TAESOOLIB_DIR}/MainLib/build_${CMAKE_BUILD_TYPE}" )
include(Common_mainlib) 

# PhysicsLib settings
set(NoAISTsim TRUE) # use TRL_LCP instead!
set(NoGMBSsim TRUE) # use TRL_LCP instead!
set(NoUTsim TRUE) # use TRL_LCP instead!
set(useOPCODEdetector FALSE)

# To use bullet simulator, I highly recommend you to refer to sample_bullet.git instead of turning on the above options.
# It uses the bullet library in the system. (libbullet-dev)
# also, if you change these settings, do not forget to change the build folder.

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


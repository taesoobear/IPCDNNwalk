include(Common_baselib) 
add_subdirectory(../../BaseLib "${CMAKE_SOURCE_DIR}/../../BaseLib/build_mpi_${CMAKE_BUILD_TYPE}" )

add_definitions(-DSEGA)				
add_definitions(-DNO_GUI -DNO_OGRE)  
add_definitions(-DUSE_MPI)
message("MPI enabled")
set(NO_GUI TRUE)

add_subdirectory(../../MainLib "${CMAKE_SOURCE_DIR}/../../MainLib/build_console_mpi_${CMAKE_BUILD_TYPE}" )

include(Common_mainlib) 

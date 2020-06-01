# -*- mode: cmake; -*-
cmake_minimum_required(VERSION 2.8)

project (openFileDialog)

#add_subdirectory(../../dependencies/clapack-3.2.1-CMAKE "${CMAKE_SOURCE_DIR}/../../dependencies/clapack-3.2.1-CMAKE/build_${CMAKE_BUILD_TYPE}" )

set(src
  "./openFileDialog.mm"
  )

add_executable(mac_openFileDialog ${src})

target_link_libraries(mac_openFileDialog
	/System/Library/Frameworks/CoreFoundation.framework
	/System/Library/Frameworks/IOKit.framework
	/System/Library/Frameworks/Cocoa.framework
	)

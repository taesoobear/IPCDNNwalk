##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2011 by Hans Joachim Ferreau, Andreas Potschka,
##	Christian Kirches et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  CMakeLists.txt
##	Author:    Hans Joachim Ferreau
##	Version:   3.0beta
##	Date:      2007-2011
##


cmake_minimum_required(VERSION 2.6)

PROJECT(qpOASES CXX)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs)
SET(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fPIC")
#SET(CMAKE_VERBOSE_MAKEFILE ON)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)


# compile qpOASES libraries
FILE(GLOB SRC src/*.cpp)
FILE(GLOB EXTRAS_SRC src/extras/*.cpp)

if(useShared)
	set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CAR_SAMPLE_DIR}/work )
	ADD_LIBRARY(qpOASES SHARED ${SRC})
else()
	ADD_LIBRARY(qpOASES ${SRC})
endif()
#ADD_LIBRARY(qpOASESextras ${SRC} ${EXTRAS_SRC})
#
#
## compile qpOASES examples
#SET(EXAMPLE_NAMES
#example1
#example1a
#example1b
#example4
#example5
#example6
#exampleLP
#qrecipe
#)
#
#SET(EXTRAS_EXAMPLE_NAMES
#example2
#example3
#example3b
#)
#
#FOREACH(ELEMENT ${EXAMPLE_NAMES})
#  ADD_EXECUTABLE(${ELEMENT} examples/${ELEMENT}.cpp)
#  TARGET_LINK_LIBRARIES(${ELEMENT} qpOASES)
#ENDFOREACH(ELEMENT ${EXAMPLE_NAMES})
#
#FOREACH(ELEMENT ${EXTRAS_EXAMPLE_NAMES})
#  ADD_EXECUTABLE(${ELEMENT} examples/${ELEMENT}.cpp)
#  TARGET_LINK_LIBRARIES(${ELEMENT} qpOASESextras)
#ENDFOREACH(ELEMENT ${EXTRAS_EXAMPLE_NAMES})


##
##   end of file
##

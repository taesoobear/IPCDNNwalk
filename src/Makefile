# Usage:
# for the first time: make install_dependencies; make init; make
# from next time, just make or make debug 
#
# This file internally uses cmake.
# Makefile is reusable. Simply change the EXE variable.
EXE=mainlib
BIN_FOLDER=../build_$(EXE)
BIN_TO_SRC=../../src
SRC_TO_WORK=../work
SRC_TO_EXE=../work/$(EXE)
NUM_CORES=$(shell if [-f /proc/cpuinfo]; then cat /proc/cpuinfo | grep processor | wc -l;	else echo 4;fi)

# all depends on init_finished.txt
all: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux;make --no-print-directory -j $(NUM_CORES)

mac: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux;make --no-print-directory -j 4

# debug depends on init_finished.txt
debug: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux_debug;make --no-print-directory VERBOSE=1 -j $(NUM_CORES)

console:
	cd ../work;python3 -i config.py

run2: 
	cd ../work;python3 test_bidirectional_functioncalls.py
rung:
	cd ../work;python3 test_sample_python.py
run: 
	cd ../work;python3 test_console.py
cgdb:
	# use gdb commands:
	#break <source file name>:<line number>
	make debug
	cd ../work;cgdb -- --args python3 test_walk2.py gym_walkCDM/testWalkV2_FA.lua

# init_finished.txt depends on CMakeLists.txt
init_finished.txt: CMakeLists.txt
	mkdir -p $(BIN_FOLDER)
	mkdir -p $(BIN_FOLDER)/build_linux;cd $(BIN_FOLDER)/build_linux;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Release $(BIN_TO_SRC)
	mkdir -p $(BIN_FOLDER)/build_linux_debug;cd $(BIN_FOLDER)/build_linux_debug;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug $(BIN_TO_SRC)
	touch init_finished.txt

clean:
	rm -rf $(BIN_FOLDER)
	rm init_finished.txt
cleanall:
	rm -rf taesooLib/BaseLib/build_shared_release
	rm -rf taesooLib/MainLib/build_shared_release
	make clean

showcapture:
	avidemux ../dump/dump/00000.jpg
	
# this would probably install more libraries than necessary, but I guess it doens't matter for most.
install_dependencies:
	sudo apt-get install python3-pip libpython3-dev libcupti-dev
	pip3 install numpy
	sudo apt install python3-numpy

install_dependencies_mac:
	brew install python@3.8
	python3 -m pip install numpy
	echo "Important!!!! Now open a finder and manually copy all frameworks in /Applications/OgreSDK/lib/macosx/Release/ to ~/Library/Frameworks/!!!"


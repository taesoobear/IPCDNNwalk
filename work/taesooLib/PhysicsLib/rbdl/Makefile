# Usage:
# for the first time: make install_dependencies; make init; make
# from next time, just make or make debug 
#
# This file internally uses cmake.
# Makefile is reusable. Simply change the EXE variable.
EXE=sample
BIN_FOLDER=build_$(EXE)
BIN_TO_SRC=../..
NUM_CORES=$(shell if test -f /proc/cpuinfo; then cat /proc/cpuinfo | grep processor | wc -l;	else echo 4;fi)

# all depends on init_finished.txt
all: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux;make --no-print-directory -j $(NUM_CORES)
	cp $(BIN_FOLDER)/build_linux/${EXE} .

# debug depends on init_finished.txt
debug: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux_debug;make --no-print-directory VERBOSE=1 -j $(NUM_CORES)
	cp $(BIN_FOLDER)/build_linux_debug/${EXE} .

run: 
	cd ../work;python3 test_console.py
cgdb:
	# use gdb commands:
	#break <source file name>:<line number>
	make debug
	#cd ../work;cgdb -- --args python test_console.py
	#cd ../work;cgdb -- --args python3 test_bidirectional_functioncalls.py
	cd ../work;cgdb -- --args python3 gym_humanoid/test_gym_humanoid.py --load-dir trained_models/ppo --env-name "humanoid-v3" --device "cpu"


# init_finished.txt depends on CMakeLists.txt
init_finished.txt: CMakeLists.txt
	mkdir -p $(BIN_FOLDER)
	mkdir -p $(BIN_FOLDER)/build_linux;cd $(BIN_FOLDER)/build_linux;export CC=gcc;export CXX=g++;cmake -D CMAKE_BUILD_TYPE=Release $(BIN_TO_SRC)
	mkdir -p $(BIN_FOLDER)/build_linux_debug;cd $(BIN_FOLDER)/build_linux_debug;export CC=gcc;export CXX=g++;cmake -D CMAKE_BUILD_TYPE=Debug $(BIN_TO_SRC)
	touch init_finished.txt

clean:
	rm -rf $(BIN_FOLDER)
	rm init_finished.txt

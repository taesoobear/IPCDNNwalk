# Usage:
# for the first time: make install_dependencies; make init; make
# from next time, just make or make debug  or make mpi
#
# This file internally uses cmake.
# Makefile is reusable. Simply change the EXE variable.
EXE=console
BIN_FOLDER=../../../taesooLib_minimal_build/$(EXE)
BIN_TO_SRC=../../../taesooLib_minimal/Samples/$(EXE)
SRC_TO_EXE=../../work/$(EXE)

# all depends on init_finished.txt
all: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux;make --no-print-directory
	cp $(BIN_FOLDER)/build_linux/OgreFltk $(SRC_TO_EXE)

# debug depends on init_finished.txt
debug: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux_debug;make --no-print-directory VERBOSE=1
	cp $(BIN_FOLDER)/build_linux_debug/OgreFltk $(SRC_TO_EXE)_debug

mpi: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	if ! test -f $(BIN_FOLDER)/build_linux_mpi/Makefile ; then mkdir -p $(BIN_FOLDER)/build_linux_mpi;cd $(BIN_FOLDER)/build_linux_mpi;export CC=mpicc;export CXX=mpiCC;cmake -G "Eclipse CDT4 - Unix Makefiles" -D UseMPI:BOOL=TRUE $(BIN_TO_SRC) ; fi
	cd $(BIN_FOLDER)/build_linux_mpi;export CC=mpicc;export CXX=mpiCC;make --no-print-directory VERBOSE=1
	cp $(BIN_FOLDER)/build_linux_mpi/OgreFltk $(SRC_TO_EXE)_mpi
	cp $(BIN_FOLDER)/build_linux_mpi/OgreFltk $(SRC_TO_EXE)

run: 
	cd ../../work;./$(EXE) driverDynamic.lua ../Samples/QP_controller/lua/

testopt:
	cd ../../work;./$(EXE) MovingWindowOptimizer.lua ../Samples/QP_controller/lua/IPC_based/

mpirun:
	cd ../../work;mpiexec -np 16  ./$(EXE)_mpi MovingWindowOptimizerMPI.lua ../Samples/QP_controller/lua/IPC_based/

mpirun4:
	cd ../../work;mpiexec -np 4  ./$(EXE)_mpi MovingWindowOptimizerMPI.lua ../Samples/QP_controller/lua/IPC_based/

mpirun1:
	cd ../../work;mpiexec -np 1  ./$(EXE)_mpi MovingWindowOptimizerMPI.lua ../Samples/QP_controller/lua/IPC_based/

doc:
	vim ../../doc/index.wiki

cgdb:
	make debug
	cd ../../work;cgdb -- --args ./$(EXE)_debug driverDynamic.lua ../Samples/QP_controller/lua/
cgdb_testopt:
	cd ../../work;cgdb -- --args ./$(EXE)_debug MovingWindowOptimizer.lua ../Samples/QP_controller/lua/IPC_based/

# init_finished.txt depends on CMakeLists.txt
init_finished.txt: CMakeLists.txt
	mkdir -p $(BIN_FOLDER)
	mkdir -p $(BIN_FOLDER)/build_linux;cd $(BIN_FOLDER)/build_linux;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" $(BIN_TO_SRC)
	mkdir -p $(BIN_FOLDER)/build_linux_debug;cd $(BIN_FOLDER)/build_linux_debug;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug $(BIN_TO_SRC)

	touch init_finished.txt

clean:
	rm -rf $(BIN_FOLDER)
	rm init_finished.txt
	
# launch vim
vim: 
	vim -c "set tags=./TAGS,../../BaseLib/TAGS,../../MainLib/TAGS,../../PhysicsLib/TAGS" lua/cart_pole.lua
gvim:
	gvim  -f -c "set tags=./TAGS,../../BaseLib/TAGS,../../MainLib/TAGS,../../PhysicsLib/TAGS" OgreFltk.cpp &

# this would probably install more libraries than necessary, but I guess it doens't matter for most.
install_dependencies:
	cd ../QP_controller/qpOASES-3.0beta;cmake . ;make
	cd ../../dependencies/clapack-3.2.1-CMAKE;mkdir -p build_;cd build_;cmake ..;make
	cd ../../MainLib/WrapperLua;sh install_readline.sh

install_dependencies_ubuntu:
	sudo apt-get install libatlas-base-dev libdevil-dev libfreeimage-dev liblua5.1-dev libxml2-dev

install_dependencies_mpi:
	sudo apt-get install libopenmpi-dev openmpi-bin

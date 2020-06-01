# all depends on init_finished.txt
all: init_finished.txt 
	if ! test -d build/build_Release ; then rm init_finished.txt; exit 1;fi
	cd build/build_Release;make --no-print-directory

# debug depends on init_finished.txt
debug: init_finished.txt
	if ! test -d build/build_Debug ; then rm init_finished.txt; exit 1;fi
	cd build/build_Debug;make --no-print-directory VERBOSE=1

# init_finished.txt depends on CMakeLists.txt
init_finished.txt: CMakeLists.txt
	mkdir -p build
	mkdir -p build/build_Release;cd build/build_Release;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" ../..
	mkdir -p build/build_Debug;cd build/build_Debug;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../..
	touch init_finished.txt

clean:
	rm -rf build
	rm init_finished.txt
	

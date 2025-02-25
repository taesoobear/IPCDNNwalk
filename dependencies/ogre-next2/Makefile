# you first need to download ogre source code, and then copy them here.  
# git checkout -b v2-3 remotes/origin/v2-3
help:
	@echo "On linux, use 'make linuxbuild'.  After that, try 'make shadowmap'"
	@echo "On mac, use 'make macbuild'. After that, try 'make shadowmap'"

ois_build:
	if ! test -d OIS ; then git clone https://github.com/wgois/OIS.git;fi
	# 4. install ois manually (https://github.com/wgois/OIS)
	# you may need to type in the following line manually.
	cd OIS;cmake -H. -B./build; cd ./build; make; sudo make install

linuxbuild:
	sudo apt-get install build-essential  cmake libfreetype6-dev libfreeimage-dev libzzip-dev libxrandr-dev libxaw7-dev freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev libx11-xcb-dev libxcb-keysyms1-dev doxygen graphviz libsdl2-dev cmake ninja-build
	cd ogre-next-deps; mkdir build; cd build; cmake ../ -D OGREDEPS_BUILD_FREEIMAGE=0 -D OGREDEPS_BUILD_FREETYPE=0 -D OGREDEPS_BUILD_ZLIB=0 -D OGREDEPS_BUILD_SHADERC=0 -G Ninja; ninja ; ninja install
	if ! test -d Dependencies ; then ln -s ogre-next-deps/build/ogredeps Dependencies; fi
	mkdir -p build/Debug;cd build/Debug; cmake -D OGRE_USE_BOOST=0\
		-D OGRE_CONFIG_THREAD_PROVIDER=0 \
		-D OGRE_CONFIG_THREADS=0 \
	   -D OGRE_BUILD_TESTS=0 \
	   -D OGRE_BUILD_SAMPLES2=1 \
	   -D CMAKE_BUILD_TYPE="Debug" \
	   -G Ninja ../..;ninja; sudo ninja install
	mkdir -p build/Release;cd build/Release; cmake -D OGRE_USE_BOOST=0\
		-D OGRE_CONFIG_THREAD_PROVIDER=0 \
		-D OGRE_CONFIG_THREADS=0 \
	   -D OGRE_BUILD_TESTS=0 \
	   -D OGRE_BUILD_SAMPLES2=1 \
	   -D CMAKE_BUILD_TYPE="Release" \
	   -G Ninja ../..;ninja; sudo ninja install

macbuild:
	# make sure xcode command line tools are already installed!!! for example,
	# xcode-select --install
	if ! test -d ogre-next-deps ; then git clone --recurse-submodules --shallow-submodules https://github.com/OGRECave/ogre-next-deps; fi
	cd ogre-next-deps; mkdir build; cd build; cmake ../ -G Xcode; cmake --build . --target ALL_BUILD --config Release; cmake --build . --target install --config Release
	if ! test -d Dependencies ; then ln -s ogre-next-deps/build/ogredeps Dependencies; fi
	brew install cmake sdl2 doxygen freeimage pkgconfig libzzip
	mkdir -p build;cd build; cmake -D OGRE_USE_BOOST=0\
		-D OGRE_CONFIG_THREAD_PROVIDER=0 \
		-D OGRE_CONFIG_THREADS=0 \
		-D OGRE_BUILD_RENDERSYSTEM_GL3PLUS=0 \
       -D OGRE_BUILD_LIBS_AS_FRAMEWORKS=0 \
	   -D OGRE_BUILD_TESTS=0 \
	   -D OGRE_BUILD_SAMPLES2=1 \
       -D CMAKE_CXX_STANDARD=11 \
       -D CMAKE_INSTALL_PREFIX=$(shell pwd)/sdk \
       -D CMAKE_CXX_FLAGS="-I/usr/local/include -F/Library/Frameworks" \
       -D CMAKE_INSTALL_NAME_DIR=$(shell pwd)/sdk/lib -G Xcode ..
	cd build;cmake --build . --target ALL_BUILD --config Release
	cd build;cmake --build . --target ALL_BUILD --config Debug
	#cd build;cmake --build . --target ALL_BUILD --config RelWithDebInfo || exit $?
	#cd build;make -j4 
	make macbuild_install_only
macbuild_install_only:
	rsync -avm --include='*.h' --include='*.inl' -f 'hide,! */' . /opt/homebrew/include/ogre-2.3
	cp build/lib/macosx/Release/*.2.3.4.dylib /opt/homebrew/lib/
	if ! test -D /opt/homebrew/lib/Debug; then mkdir -p /opt/homebrew/lib/Debug ; fi
	cp -rf build/lib/macosx/Debug/*.2.3.4.dylib /opt/homebrew/lib/Debug/
	cp build/lib/Release/libOgreSamplesCommon.a /opt/homebrew/lib/libOgreSamplesCommon.2.3.4.a
	cp build/bin/Release/OgreMeshTool /opt/homebrew/bin/
	cp taesoo/plugins2_tools_mac.cfg /opt/homebrew/bin/plugins_tools.cfg
	#Include Components/Hlms/Common/include
	#Include Components/Hlms/Pbs/include
	#Include Components/Hlms/Unlit/include
	#Include build/Release/include (that's where OgreBuildSettings.h is)
	#Include Components/Overlay/include for 2D Overlays
	echo "you can set PluginFolder=/opt/homebrew/lib for each sample you are trying to run." 

clean:
	rm -rf build
cleanall:
	rm -rf build
	rm -rf ogre-next-deps/build
	rm -f Dependencies

shadowmap:
	# mac
	if test -d build/bin/Release; then cd build/bin/Release;./Sample_ShadowMapFromCode.app/Contents/MacOS/Sample_ShadowMapFromCode;fi
	# linux
	if test -d build/Release/bin; then cd build/Release/bin;./Sample_ShadowMapFromCode;fi
open_sample_folder:
	if test -d build/Release/bin; then open build/Release/bin;fi
	if test -d build/bin/Release; then open build/bin/Release;fi

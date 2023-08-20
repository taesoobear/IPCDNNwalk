.PHONY: help doc commit_doc clean 
all:
	cd Samples/classification;make

doc:
	vi doc/index.wiki

help:
	cd doc;lua help.lua

commit_doc:
	cd doc;sh sync.sh

cp_from:
	cp -rf ~/taesooLib/Samples/QP_controller/lua Samples/QP_controller/
	cp -rf ~/taesooLib/Samples/classification/lua Samples/classification/
	cp -rf ~/taesooLib/Resource/motion/gymnist Resource/motion/
	cp -rf ~/taesooLib/Samples/scripts/RigidBodyWin Samples/scripts/
	cp -rf ~/taesooLib/BaseLib/math BaseLib
	cp -rf ~/taesooLib/BaseLib/motion BaseLib
	cp -rf ~/taesooLib/MainLib/OgreFltk MainLib
	cp -rf ~/taesooLib/MainLib/WrapperLua MainLib
cp_to:
	cp -rf Samples/QP_controller/lua ~/taesooLib/Samples/QP_controller/
	cp -rf Samples/classification/lua ~/taesooLib/Samples/classification/
	cp -rf Resource/motion/gymnist ~/taesooLib/Resource/motion/

clean:
	rm -rf BaseLib/build_*
	rm -rf MainLib/build_*
	rm -rf PhysicsLib/build_*
	rm -rf ClassificationLib/build_*

install_dependencies_only_for_ubuntu12.04:
	sudo apt-get install libatlas-base-dev libdevil-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev libpng12-dev  lua5.1 cmake g++ libxml2-dev libogre-dev libgsl0-dev libncurses5-dev 
	if ! test -d /usr/lib/OGRE; then sudo ln -s /usr/lib/i386-linux-gnu/OGRE-1.7.4 /usr/lib/OGRE ;fi
	make build_dependencies

install_dependencies_only_for_ubuntu12.10:
	sudo apt-get install libsuitesparse-dev libatlas-base-dev libdevil-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev libpng12-dev  lua5.1 cmake g++ libxml2-dev libogre-1.8-dev libgsl0-dev libncurses5-dev 
	make build_dependencies
	make symlink

install_dependencies_only_for_ubuntu14.04:
	sudo apt-get install libsuitesparse-dev libatlas-base-dev libdevil-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev libpng12-dev  lua5.1 cmake g++ libxml2-dev libogre-1.9-dev libgsl0-dev libncurses5-dev libois-dev libboost-all-dev libf2c2-dev
	make build_dependencies
	make symlink

install_dependencies_only_for_ubuntu15_16_17:
	sudo apt-get install python-gtk2
	make install_dependencies_only_for_ubuntu14.04

install_dependencies_only_for_ubuntu18:
	sudo apt-get install python-gtk2 libeigen3-dev
	sudo apt-get install libsuitesparse-dev libatlas-base-dev libdevil-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev  lua5.1 cmake g++ libxml2-dev libogre-1.9-dev libgsl0-dev libncurses5-dev libois-dev libboost-all-dev libf2c2-dev freeglut3-dev
	make build_dependencies
	make symlink

install_dependencies_only_for_ubuntu18_and_ogre1_12:
	sudo apt-get install python-gtk2 libeigen3-dev
	sudo apt-get install libsuitesparse-dev libatlas-base-dev libdevil-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev  lua5.1 cmake g++ libxml2-dev libgsl0-dev libncurses5-dev libois-dev libboost-all-dev libf2c2-dev freeglut3-dev
	make build_dependencies
	make symlink
	cd ..;git clone ssh://students@calab.hanyang.ac.kr:8022/~/ogre-1.12.5.git;cd ogre-1.12.5; make build
	echo "Read wiki  : section fbx to ogre mesh"

install_dependencies_only_for_ubuntu19:
	sudo apt-get install libeigen3-dev python3-wxgtk4.0
	sudo apt-get install libsuitesparse-dev libatlas-base-dev libdevil-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev  lua5.1 cmake g++ libxml2-dev libgsl0-dev libncurses5-dev libois-dev libboost-all-dev libf2c2-dev freeglut3-dev
	make build_dependencies
	make symlink
	cd ..;git clone ssh://students@calab.hanyang.ac.kr:8022/~/ogre-1.12.5.git;cd ogre-1.12.5; make build
	echo "Read wiki  : section fbx to ogre mesh"

install_dependencies_only_for_ubuntu20and21:
	sudo apt-get install libeigen3-dev python3-wxgtk4.0 libogre-1.12-dev
	# libboost is unnecessary in ubuntu20 branch, but master branch also works for ubuntu20.
	sudo apt-get install libsuitesparse-dev libatlas-base-dev libdevil-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev  lua5.1 cmake g++ libxml2-dev libgsl0-dev libncurses5-dev libois-dev libf2c2-dev freeglut3-dev libboost-system-dev 
	make build_dependencies
	make symlink
	sudo mkdir /usr/local/lib/OGRE
	if test -d /usr/lib/x86_64-linux-gnu/OGRE-1.12; then sudo cp /usr/lib/x86_64-linux-gnu/OGRE-1.12/* /usr/local/lib/OGRE/ ; fi
	if test -d /usr/lib/x86_64-linux-gnu/OGRE; then sudo cp /usr/lib/x86_64-linux-gnu/OGRE/* /usr/local/lib/OGRE/ ; fi
	echo "Read wiki  : section fbx to ogre mesh"

install_dependencies_intelmac:
	brew install fltk eigen gsl lua@5.1
	pip3 install wxPython
	ln -s /usr/local/bin/lua5.1 /usr/local/bin/lua
	cd ..;git clone ssh://students@calab.hanyang.ac.kr:8022/~/ogre-1.12.5.git;cd ogre-1.12.5; make macbuild
	echo "1. Build all in the xcode by first selecting the build target by clicking the button on the right side of the play button, and then clicking the play button"
	echo "2. Important!!!! Now open a finder and manually copy all frameworks in build/lib/macosx/Release/ to /Applications/OgreSDK/lib/macosx/Release/!!!"
	echo "You cannot use cp. You have to use finder. cp -Rp might work but I didn't try. cp -R does not work."
	echo "3. copy build/sdk/include/OGRE to /usr/local/include/"
	echo "4. install ois manually (https://github.com/wgois/OIS)"
	echo "5. cp libfreetype*.dylib to /usr/local/lib"
	echo "6. cp ogre-1.12.5/RenderSystems/GLSupport/include/OSX/*.h /usr/local/include"
	echo "7. If this doesn't work, try copying all frameworks directly to /System/Library/Frameworks using finder."
	
install_dependencies_m1mac:
	brew install fltk eigen gsl lua@5.1 ctags
	pip3 install wxPython
	ln -s /opt/homebrew/bin/lua5.1 /opt/homebrew/bin/lua
	cd ..;git clone ssh://students@calab.hanyang.ac.kr:8022/~/ogre-13.4.3.git;cd ogre-13.4.3; make macbuild
	echo "1. Build all in the xcode by first selecting the build target, and then clicking the play button"
	echo "2. Important!!!! Now open a finder and manually copy all frameworks in build/lib/macosx/Release/ to /Applications/OgreSDK/lib/macosx/Release/!!!"
	echo "You cannot use cp. You have to use finder. cp -Rp might work but I didn't try. cp -R does not work."
	echo "3. copy build/sdk/include/OGRE to /usr/local/include/"
	echo "4. install ois manually (https://github.com/wgois/OIS)"
	echo "5. cp libfreetype*.dylib to /usr/local/lib"
	echo "6. cp ogre-13.4.3/RenderSystems/GLSupport/include/OSX/*.h /usr/local/include"
	echo "7. If this doesn't work, try copying all frameworks directly to /System/Library/Frameworks using finder."

symlink_mac:
	ln -s /Applications/OgreSDK/lib/macosx/Release Frameworks
symlink:
	sudo rm -f /usr/lib/OGRE
	if test -d /usr/lib/i386-linux-gnu/OGRE-1.7.4; then sudo ln -s /usr/lib/i386-linux-gnu/OGRE-1.7.4 /usr/lib/OGRE ;fi
	if test -d /usr/lib/x86_64-linux-gnu/OGRE-1.7.4; then sudo ln -s /usr/lib/x86_64-linux-gnu/OGRE-1.7.4 /usr/lib/OGRE ; fi
	if test -d /usr/lib/i386-linux-gnu/OGRE-1.9.0; then sudo ln -s /usr/lib/i386-linux-gnu/OGRE-1.9.0 /usr/lib/OGRE ;fi
	if test -d /usr/lib/x86_64-linux-gnu/OGRE-1.9.0; then sudo ln -s /usr/lib/x86_64-linux-gnu/OGRE-1.9.0 /usr/lib/OGRE ; fi

build_dependencies:
	cd MainLib/WrapperLua;sh install_readline.sh
	cd MainLib/fbxDumpLua;make clean;make clean all
build_dependencies_mac:
	cd MainLib/WrapperLua;sh install_readline_mac.sh
	cd MainLib/fbxDumpLua;make clean;make clean all
build_clapack:
	cd dependencies/clapack-3.2.1-CMAKE;mkdir -p build_;cd build_;cmake ..;make

diff:
	meld Samples/scripts ../taesooLib/Samples/scripts
	meld BaseLib ../taesooLib/BaseLib/
	meld Samples/QP_controller ../taesooLib/Samples/QP_controller
	meld Samples/make.lua ../taesooLib/Samples/make.lua

diff2:
	meld Samples/scripts ../../taesooLib/Samples/scripts
	meld BaseLib ../../taesooLib/BaseLib/
	meld Samples/QP_controller ../../taesooLib/Samples/QP_controller
	meld Samples/make.lua ../../taesooLib/Samples/make.lua

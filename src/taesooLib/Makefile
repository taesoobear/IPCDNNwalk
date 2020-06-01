.PHONY: help doc commit_doc clean 
all:
	cd Samples/sample1;make
	cd Samples/sample2;make
	cd Samples/sample3_IK;make
	cd Samples/sample4_luascripting;make
	cd Samples/sample5_limbIK;make
	cd Samples/classification;make
	cd Samples/CASA2007_locomotion;make

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

destroy_self:
	find . -iname "*.cpp" | xargs rm
	find . -iname "*.o" | xargs rm
	rm -rf .git

install_dependencies_only_for_ubuntu12.10:
	sudo apt-get install libsuitesparse-dev libatlas-base-dev libdevil-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev libpng12-dev  lua5.1 cmake g++ libxml2-dev libogre-1.8-dev libgsl0-dev libncurses5-dev 
	make build_dependencies
	make symlink

install_dependencies_only_for_ubuntu14.04:
	sudo apt-get install libsuitesparse-dev libatlas-base-dev libdevil-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev libpng12-dev  lua5.1 cmake g++ libxml2-dev libogre-1.9-dev libgsl0-dev libncurses5-dev libois-dev libboost-all-dev libf2c2-dev
	make build_dependencies
	make symlink

install_dependencies_only_for_ubuntu15.10_and_above:
	sudo apt-get install python-gtk2
	make install_dependencies_only_for_ubuntu14.04

install_dependencies_mac:
	ln -s /Applications/OgreSDK/lib/macosx/Release Frameworks

symlink:
	sudo rm -f /usr/lib/OGRE
	if test -d /usr/lib/i386-linux-gnu/OGRE-1.7.4; then sudo ln -s /usr/lib/i386-linux-gnu/OGRE-1.7.4 /usr/lib/OGRE ;fi
	if test -d /usr/lib/x86_64-linux-gnu/OGRE-1.7.4; then sudo ln -s /usr/lib/x86_64-linux-gnu/OGRE-1.7.4 /usr/lib/OGRE ; fi
	if test -d /usr/lib/i386-linux-gnu/OGRE-1.9.0; then sudo ln -s /usr/lib/i386-linux-gnu/OGRE-1.9.0 /usr/lib/OGRE ;fi
	if test -d /usr/lib/x86_64-linux-gnu/OGRE-1.9.0; then sudo ln -s /usr/lib/x86_64-linux-gnu/OGRE-1.9.0 /usr/lib/OGRE ; fi
	ls -alF /usr/lib/OGRE

build_dependencies:
	cd MainLib/WrapperLua;sh install_readline.sh
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

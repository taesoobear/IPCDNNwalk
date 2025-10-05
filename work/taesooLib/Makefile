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
	cd Samples/classification;make clean;

	
install_dependencies_mac:
	brew install fltk eigen gsl ctags
	pip3 install wxPython
	if ! test -d ../lua-5.1.5; then cd ..;git clone ssh://students@calab.hanyang.ac.kr:8022/~/lua-5.1.5.git;cd lua-5.1.5; make install_macosx ; fi
	if ! test -d ../ogre-next3; then cd ..;git clone ssh://students@calab.hanyang.ac.kr:8022/~/ogre-next3.git;cd ogre-next3; make macbuild;fi
	make build_dependencies_mac
	echo "0. if make macbuild fails, retry after rebooting!"
	

install_python_ubuntu:
	sudo apt install python3 python3-pip  virtualenvwrapper
	echo "source /usr/share/virtualenvwrapper/virtualenvwrapper.sh" >> ~/.bashrc
	
install_dependencies_ubuntu: # tested on ubuntu 24
	sudo apt install libfltk1.3-dev libreadline-dev  wxpython-tools libois-dev libeigen3-dev
	if ! test -d ../lua-5.1.5; then cd ..;git clone ssh://students@calab.hanyang.ac.kr:8022/~/lua-5.1.5.git;cd lua-5.1.5; make install_linux; fi
	if ! test -d ../ogre-next3; then cd ..;git clone ssh://students@calab.hanyang.ac.kr:8022/~/ogre-next3.git;cd ogre-next3; make linuxbuild; fi
	make build_dependences_ubuntu


build_dependencies:
	# for backward compatibility
	make build_dependencies_ubuntu
build_dependencies_ubuntu:
	make build_dependencies_mac

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

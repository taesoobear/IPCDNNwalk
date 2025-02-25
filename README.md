# Fast and Flexible Multilegged Locomotion Using Learned Centroidal Dynamics, in proc. ACM SIGGRAPH 2020
# Adaptive Tracking of a Single-Rigid-Body Character in Various Environments, In proc. ACM SIGGRAPH ASIA 2023 

Target platform
=
Linux and MacOS.

How to build (Linux)
=

  1. First, install necessary dependencies. This probably installs more than actually necessary:
```
	sudo apt install libfltk1.3-dev libreadline-dev  wxpython-tools
	cd dependenceies/lua5.1; make install_linux 
  mkdir ogre-next-source;  cd ogre-next-source; git clone https://github.com/OGRECave/ogre-next.git; cd ogre-next; git checkout 50bbee7d
	cd dependenceies/ogre-next3; cp -rf ../ogre-next-source/ogre-next/* . 
```
  Then you need to apply the patch file in the dependencies/ogre-next3 as follows (untested. you can do this manually by looking at the patch file). 
```
    cd dependences/ogre-next3;patch -p0 < ogre-latest_50bbee7d.patch
```
   2. Now you can build ogre-next (latest):
```
   cd dependencies/ogre-next3
make linuxbuild
```
  3.  Also install python3 and pip3 (versions do not matter.)

  4. Now, build this project.
```
	pip3 install stable-baselines3 torch numpy gymnasium
  cd src; make
```
  This should work on any versions of ubuntu.

How to build (Mac)
=
  1. First, install necessary dependencies:
```
  brew install fltk eigen gsl cmake 
  pip3 install wxPython
	cd dependenceies/lua5.1; make install_macosx 
  mkdir ogre-next-source;  cd ogre-next-source; git clone https://github.com/OGRECave/ogre-next.git; cd ogre-next; git checkout -b v2-3 remotes/origin/v2-3
	cd dependenceies/ogre-next3; cp -rf ../ogre-next-source/ogre-next/* . 
```

  Then you need to apply the patch file in the dependencies/ogre-next3 as follows (untested. you can do this manually by looking at the patch file). 
```
	cd dependences/ogre-next3/ogre-next-deps/src/freetype/src;git checkout -f gzip/zconf.h;patch -p1 gzip/zconf.h < ../../../../zconf.patch
    cd dependences/ogre-next3;patch -p0 < ogre-latest_50bbee7d.patch
```
   2. Now you can build ogre-next3 (latest):
```
   cd dependencies/ogre-next3
make macbuild
```
3.   Also install python3 and pip3 (versions do not matter.)

  4. Now, build this project.
```
  cd src; make 
```
  FYI, most linker and compiler settings are in src/taesoolib/Samples/Common_baselib.cmake and Common_mainlib.cmake files.

How to run FlexLoco 2020
= 
```
  pip3 install torch python3-tk gymnasium
  cd work; python3 test_walk2.py gym_walkCDM/testWalkV2_FA.lua
```
   Choose the opengl or metal renderer if asked to choose one.
  Also, on Linux, choose 1920x1080 resolution. If you already chose a different resolution, re-try after deleting ogre2_linux.cf2_linux.cfg
   Now, click the play button, and adjust the slider bars for speed/orientation. Also, you can change the motion type by clicking the button.

How to run AdaptiveSRB 2023
=
```
  cd work; make walk 
```



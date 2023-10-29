# Fast and Flexible Multilegged Locomotion Using Learned Centroidal Dynamics, in proc. ACM SIGGRAPH 2020
# Adaptive Tracking of a Single-Rigid-Body Character in Various Environments, In proc. ACM SIGGRAPH ASIA 2023 

Target platform
=
Linux and MacOS.

How to build (Linux)
=

  1. First, install necessary dependencies. This probably installs more than actually necessary:
```
  sudo apt-get install python-gtk2 libeigen3-dev libsuitesparse-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev  lua5.1 cmake g++ libxml2-dev libncurses5-dev libois-dev libboost-all-dev libf2c2-dev libogre-1.9-dev
```
  
  2. Install the system-provided ogre library to /usr/lib/OGRE:
```
  sudo ln -s /usr/lib/x86_64-linux-gnu/OGRE-1.9.0 /usr/lib/OGRE 
```

  3. Install more dependencies (after reading src/Makefile carefully):
```
  cd src; make install_dependencies
```

  4. Now, build it.
```
  cd src; make
```

  This should work on ubuntu 18 and 20.
  If this does not work, please remove the system-provided libogre-1.9-dev, and install ogre3D-1.12 from source (see below), and retry the above steps (2-).
	(There are a few linux distros that provide libogre-1.9-dev which is broken.)

How to build (Mac)
=
  1. First, install necessary dependencies:
```
  brew install fltk eigen gsl cmake lua@5.1
  pip3 install wxPython
  (intel mac)
  cd /usr/local/bin; ln -s lua5.1 lua
  (m1 mac)
	ln -s /opt/homebrew/bin/lua5.1 /opt/homebrew/bin/lua
```

  2. Then, build Ogre 13.4 or Ogre 1.12 from sources (see below).

  3. Install more dependencies (after reading src/Makefile carefully):
```
  cd src; make install_dependencies_mac
```

  4. Now, build it.
```
  cd src; make mac
```
  FYI, most linker and compiler settings are in src/taesoolib/Samples/Common_baselib.cmake and Common_mainlib.cmake files.

How to run FlexLoco 2020
= 
```
  pip3 install torch python3-tk gym
  cd work; python3 test_walk2.py gym_walkCDM/testWalkV2_FA.lua
```
   Choose the opengl renderer if asked to choose one.
   Now, click the play button, and adjust the slider bars for speed/orientation. Also, you can change the motion type by clicking the button.

How to run AdaptiveSRB 2023
=
  This section will be updated soon.


How to build ogre 3D from source codes (Linux) 
=
This is usually unnecessary because the ogre3d in the APT works well.
```
  sudo apt-get install libgles2-mesa-dev libxt-dev libxaw7-dev nvidia-cg-toolkit libsdl2-dev doxygen
  mkdir -p build;cd build; cmake ..
  cd build;make
  cd build; sudo make install
```

How to build ogre 3D-13.4.3 from source codes (Mac)
=
```
  brew install cmake sdl2 doxygen freeimage pkgconfig libzzip
  mkdir -p build;cd build; cmake -G Xcode .. 
  cd build; xed .
```


 1. Build "build-all" in the xcode after manually selecting the release build target. 
 2. Build "install" in the xcode again after manually selecting the release build target. 
 3. Install ois and ogre manually as below (https://github.com/wgois/OIS)

```
  git clone https://github.com/wgois/OIS.git
  cd OIS;cmake -H. -B./build; cd ./build; make; sudo make install
  sudo mkdir -p /usr/local/include/OGRE
  sudo mkdir -p /usr/local/include/OGRE/Overlay
  sudo mkdir -p /usr/local/include/OGRE/Bites
  sudo cp -rf OgreMain/include/* /usr/local/include/OGRE
  sudo cp -rf Components/Overlay/include/*  /usr/local/include/OGRE/Overlay
  sudo cp -rf Components/Bites/include/*  /usr/local/include/OGRE/Bites
  sudo cp -rf RenderSystems/GLSupport/include/OSX/OgreOSXCocoa*.h  /usr/local/include/OGRE
  sudo cp -rf RenderSystems/GLSupport/include/*.h  /usr/local/include/OGRE
  sudo cp -rf build/include/* /usr/local/include/OGRE
  sudo cp -rf build/include/OgreBites* /usr/local/include/OGRE/Bites
  mkdir -p  /Applications/OgreSDK/lib/macosx/Release
  cp -Rp build/lib/macosx/Release/* /Applications/OgreSDK/lib/macosx/Release
  sudo cp -Rp build/lib/macosx/Release/*.framework /Library/Frameworks/
```


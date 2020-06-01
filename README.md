# IPCDNNwalk


Target platform
=
Linux and MacOS.

How to build (Linux)
=

  1. First, install necessary dependencies:
```
	sudo apt-get install python-gtk2 libeigen3-dev libsuitesparse-dev libfontconfig-dev libfltk1.1-dev libdevil-dev libfreeimage-dev liblua5.1-dev  lua5.1 cmake g++ libxml2-dev libncurses5-dev libois-dev libboost-all-dev libf2c2-dev libogre-1.9-dev
```
  
  2. Install the system-provided ogre library to /usr/lib/OGRE
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

  If this does not work, please remove libogre-1.9-dev, and install ogre3D-1.12 from the source (see below), and retry the above steps!

How to build (Mac)
=
  1. First, install necessary dependencies:

```
	brew install boost fltk eigen gsl cmake
```

  2. Then, build Ogre 1.12 from sources (see below).

  3. Install more dependencies (after reading src/Makefile carefully):
```
  cd src; make install_dependencies_mac
```

  4. Now, build it.
```
  cd src; make mac
```


How to run
= 
```
  cd work; make walknn
```
   Now, click the play button, and adjust the slider bars for speed/orientation. Also, you can change the motion type by clicking the button.

How to build ogre 3D from the source code (Linux)
=
```
	sudo apt-get install libgles2-mesa-dev libxt-dev libxaw7-dev nvidia-cg-toolkit libsdl2-dev doxygen
	mkdir -p build;cd build; cmake ..
	cd build;make
	cd build; sudo make install
```

How to build ogre 3D from the source code (Mac)
=
```
  brew install cmake sdl2 doxygen freeimage pkgconfig libzzip
  mkdir -p build;cd build; cmake -G Xcode .. 
  cd build; xed .
```


 1. Build "build-all" in the xcode after manually selecting the release build target. 
 2. Build "install" in the xcode again after manually selecting the release build target. 
 3. Now open a finder, and copy build/sdk to /Applications/OgreSDK
 3. Also, manually copy all frameworks in build/lib/macosx/Release/ to ~/Library/Frameworks/!!!
 4. copy build/sdk/include/OGRE to /usr/local/include/
 5. install ois manually (https://github.com/wgois/OIS)

```
  git clone https://github.com/wgois/OIS.git
  cd OIS;cmake -H. -B./build; cd ./build; make; make install
  sudo cp build/Dependencies/lib/libzzip.so* /usr/local/lib/
```
 6. cp libfreetype*.dylib to /usr/local/lib


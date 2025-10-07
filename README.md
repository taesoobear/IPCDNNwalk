# Target platforms
Windows (AMD64, ARM64), MacOS, and ubuntu24.04 with python 3.12 only.
Otherwise, you will need to build the dependencies manually.
(see https://pypi.org/project/libcalab-ogre3d/ for the list of prebuilt binaries available on PyPI)

# SRBTrack: Terrain-Adaptive Tracking of a Single-Rigid-Body Character Using Momentum-Mapped Space-Time Optimization, In proc. SIGGRAPH Asia 2025
How to run SRBTrack 2025
=
```
  pip3 install torch python3-tk gymnasium libcalab_ogre3d opt_einsum easydict mujoco clarabel scipy transformations
  python3 walk_SRBTrack2025.py
```
Choose the D3D (windows), openGL (ubuntu), or metal renderer (mac) if asked to choose one. Also, on Linux, choose 1920x1080 resolution. 
If you already chose a different resolution, re-try after deleting ogre2_linux.cfg 

env, gym_trackSRB folders contain the source codes of "SRBTrack: Terrain-Adaptive Tracking of a Single-Rigid-Body Character Using Momentum-Mapped Space-Time Optimization"

# AdaptiveSRB: Adaptive Tracking of a Single-Rigid-Body Character in Various Environments, In proc. ACM SIGGRAPH ASIA 2023 
How to run AdaptiveSRB 2023
=
```
  pip3 install torch python3-tk gymnasium libcalab_ogre3d
  pip3 install -r requirements<TAB>.txt
  python3 walk_SRB2023.py
```
Choose the D3D (windows), openGL (ubuntu), or metal renderer (mac) if asked to choose one.
Also, on Linux, choose 1920x1080 resolution. 
If you already chose a different resolution, re-try after deleting ogre2_linux.cfg

gym_cdm2 contains the source codes of "Adaptive Tracking of a Single-Rigid-Body Character in Various Environments".


# FlexLoco: Fast and Flexible Multilegged Locomotion Using Learned Centroidal Dynamics, in proc. ACM SIGGRAPH 2020
How to run FlexLoco 2020
= 
```
  pip3 install torch python3-tk gymnasium libcalab_ogre3d
  python3 walk_FlexLoco2020.py 
```
Now, click the play button, and adjust the slider bars for speed/orientation. Also, you can change the motion type by clicking the button.

work/gym_walkCDM contains the source codes of "Fast and Flexible Multilegged Locomotion Using Learned Centroidal Dynamics".

# (Optional) How to build (Linux)
=

todo: this part needs to be updated.
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
``` cd src; make 
```
  FYI, most linker and compiler settings are in src/taesoolib/Samples/Common_baselib.cmake and Common_mainlib.cmake files.


src/taesooLib contains the taesooLib. All of my research projects rely on this library, which contains C++ code related to motion loading, inverse kinematics (IK), rendering, and more.

src/taesooLib/BaseLib contains the core library which is GUI-independent.

src/taesooLib/MainLib extends the core library with Ogre3D and FLTK-based GUI components.

src/taesooLib/PhysicsLib contains a few rigid-body simulators.

src/taesooLib/MainLib/WrapperLua/luna_mainlib.lua defines the functions and classes that can be used in lua scripts.

src/taesooLib/MainLib/python/MainlibPython.hpp defines the functions and classes that can be used in python scripts.



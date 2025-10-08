default:
	echo "use make mac_build_wheelor linux_build_wheel"
mac_build_wheel:
	cd ../sample_python/src;make 
	cp ../sample_python/build_mainLib/build_linux/MainLib/build_shared_Release/libMainLib.dylib libcalab_ogre3d/
	cp ../sample_python/build_mainLib/build_linux/MainLib/build_shared_Release/build_Release/liblua51.dylib libcalab_ogre3d/
	cp ../sample_python/build_mainLib/build_linux/BaseLib/build_shared_Release/libBaseLib.dylib libcalab_ogre3d/
	cp ../sample_python/build_mainLib/build_linux/PhysicsLib/build_shared_Release/libPhysicsLib.dylib libcalab_ogre3d/
	cp ../sample_python/build_mainLib/build_linux/ClassificationLib/build_shared_Release/libClassificationLib.dylib libcalab_ogre3d/
	cp ../sample_python/build_mainLib/build_linux/ClassificationLib/build_shared_Release/math/ann_1.1.2/src/libann.dylib libcalab_ogre3d/
	cp /opt/homebrew/lib/libfreeimage.dylib libcalab_ogre3d/ || echo "no freeimage files"
	cp /opt/homebrew/lib/ogre-3/*.4.0.dylib libcalab_ogre3d/ || echo "no ogre files"
	cp /usr/local/lib/libOIS.1.5.0.dylib libcalab_ogre3d/ || echo "no OIS files"
	cp /opt/homebrew/lib/libfltk*1*.dylib libcalab_ogre3d/ || echo "no fltk files"
	cp  ../sample_python/work/*.pyd libcalab_ogre3d/ || echo "no pyd files"
	cp  ../sample_python/work/*.so libcalab_ogre3d/|| echo "no so files"
	cp  ../sample_python/work/*.dylib libcalab_ogre3d/|| echo "no dylib files"
	# make libmainlib portable
	python3 fix_mac.py
	#pip wheel . -w dist
	python -m build
	echo "also copy macosx15.whl to macosx13.whl"
install_dependencies:
	pip install setuptools pybind11 wheel build twine
run_luna_gen:
	cd src/taesooLib/Samples/classification; lua ../../MainLib/WrapperLua/luna_gen.lua  ../../MainLib/WrapperLua/luna_baselib.lua
	cd src/taesooLib/Samples/classification; lua ../../MainLib/WrapperLua/luna_gen.lua  ../../PhysicsLib/luna_physics.lua
	cd src/taesooLib/Samples/classification; lua ../../MainLib/WrapperLua/luna_gen.lua  ../../ClassificationLib/luna_classification.lua
	cd src/sample_python/src; lua ../../taesooLib/MainLib/WrapperLua/luna_gen.lua  luna_QP.lua
diff:
	gitv diffd src/sample_python/src ../sample_python/src
	gitv diffd src/taesooLib ../taesooLib
diff_script:
	gitv diffd libcalab_ogre3d ../sample_python/work
	gitv diffd libcalab_ogre3d/Resource/scripts ../taesooLib/Resource/scripts
	gitv diffd libcalab_ogre3d/Resource/scripts ../taesooLib/Samples/scripts/lua/
	gitv diffd libcalab_ogre3d/Resource/scripts/RigidBodyWin ../taesooLib/Samples/scripts/RigidBodyWin
fix_linux:
	auditwheel repair dist/*.whl
  
clean:
	rm -f dist/*.whl
	rm -f dist/*.tar.gz
	rm -f wheelhouse/*

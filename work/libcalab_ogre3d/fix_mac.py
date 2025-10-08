import glob,os,pdb

#dylib_files = glob.glob("libcalab_ogre3d/*.so")+glob.glob("libcalab_ogre3d/*.dylib")
dylib_files=[ 'libcalab_ogre3d/libmainlib.cpython-312-darwin.so', 'libcalab_ogre3d/readline.so', 'libcalab_ogre3d/libann.dylib', 'libcalab_ogre3d/liblua51.dylib', 'libcalab_ogre3d/libMainLib.dylib',  'libcalab_ogre3d/libClassificationLib.dylib',  'libcalab_ogre3d/libqpOASES.dylib',  'libcalab_ogre3d/libPhysicsLib.dylib', 'libcalab_ogre3d/libBaseLib.dylib', ]
print(dylib_files)

local_targets=['libBaseLib.dylib', 'libClassificationLib.dylib', 'libann.dylib', 'libMainLib.dylib', 'liblua51.dylib', 'libPhysicsLib.dylib', 'libOgreNextMain.4.0.dylib', 'libOgreNextAtmosphere.4.0.dylib', 'libOgreNextHlmsPbs.4.0.dylib', 'libOgreNextHlmsUnlit.4.0.dylib', 'libOgreNextMain.4.0.dylib', 'libOgreNextMeshLodGenerator.4.0.dylib', 'libOgreNextOverlay.4.0.dylib', 'libOgreNextSceneFormat.4.0.dylib', 'libqpOASES.dylib', 'Plugin_ParticleFX.4.0.dylib', 'Plugin_ParticleFX2.4.0.dylib', 'RenderSystem_Metal.4.0.dylib', 'RenderSystem_NULL.4.0.dylib','libOIS.1.5.0.dylib']

global_fltk_targets=[ 'libfltk_images.1.3.dylib', 'libfltk_images.1.4.dylib', 'libfltk.1.3.dylib', 'libfltk.1.4.dylib', 'libfltk_gl.1.3.dylib', 'libfltk_gl.1.4.dylib', ]

for i, v in enumerate(dylib_files):
    print(v)
    for j, target in enumerate(local_targets):
        if not v.endswith(target):
            os.system('install_name_tool -change @rpath/'+target+' @loader_path/'+target+' '+v)
    for j, target in enumerate(global_fltk_targets):
        os.system('install_name_tool -change /opt/homebrew/opt/fltk/lib/'+target+' @loader_path/'+target+' '+v)
    os.system('install_name_tool -change /opt/homebrew/opt/freeimage/lib/libfreeimage.dylib @loader_path/libfreeimage.dylib '+v)


dylib_files=glob.glob("libcalab_ogre3d/libfltk*.dylib")

for i, v in enumerate(dylib_files):
    for j, target in enumerate(global_fltk_targets):
        os.system('install_name_tool -change /opt/homebrew/opt/fltk/lib/'+target+' @loader_path/'+target+' '+v)
        os.system('install_name_tool -change /opt/homebrew/Cellar/fltk/1.3.9/lib/'+target+' @loader_path/'+target+' '+v)
        os.system('install_name_tool -change /opt/homebrew/Cellar/fltk/1.4.2/lib/'+target+' @loader_path/'+target+' '+v)

    os.system('codesign --remove-signature '+v)
    os.system('codesign --force --sign - '+v)
    #os.system('codesign -s "taesoobear@gmail.com" '+v)


v='libcalab_ogre3d/libfreeimage.dylib'
os.system('codesign --remove-signature '+v)
os.system('codesign --force --sign - '+v)

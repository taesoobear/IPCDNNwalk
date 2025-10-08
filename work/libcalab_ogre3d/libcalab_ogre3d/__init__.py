# __init__.py
"""
mypackage: Python bindings with C++ extension and resources.
"""

script_version=1 # version of the lua modules
import os, sys, platform, easydict

package_dir = os.path.dirname(os.path.abspath(__file__))

if platform.system() == "Windows":
    import ctypes
    testlib = ctypes.CDLL(package_dir+"\\RenderSystem_Direct3D11.dll")
    testlib = ctypes.CDLL(package_dir+"\\Plugin_ParticleFX.dll")

# C++ extension module (built via pybind11_add_module(libcalab ...))
try:
    from . import libmainlib as m
    print('taesooLibPath:', package_dir)
    m.setTaesooLibPath(package_dir+"/")
except ImportError as e:
    print('failed to import libmainlib', package_dir)
    raise ImportError("Failed to import C++ extension 'libmainlib'. "
                      "Did you build and install the wheel correctly?") from e
# temporarily ignore stderr because windows output is too verbose
tempErr=sys.stderr
sys.stderr = open(os.devnull, 'w')
try:
    import settings
except ModuleNotFoundError:
    try:
        import work.settings as settings
        settings.relativeMode=True

    except ModuleNotFoundError:  # work path doesn't exist so we have to use internal resources
        from . import default_settings as settings
        settings.relativeMode=True
sys.stderr=tempErr

settings.mlib=m

if hasattr(settings, 'script_version') and settings.script_version>script_version:
    # use local scripts (usually in ./work)
    settings._loadDefault()
    lua=settings.lua
    RE=settings.RE
    control=settings.control

else:
    # use installed scripts
    try:
        from . import luamodule as lua
        settings.lua=lua
        from . import rendermodule as RE
        settings.RE=RE
        from . import controlmodule as control
        settings.control=control
    except ImportError as e:
        raise ImportError("Failed to import python extensions 'luamodule, rendermodule, controlmodule'. "
                          "Did you build and install the wheel correctly?") from e




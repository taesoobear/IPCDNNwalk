# globals variables are kept in this module to avoid name collisions. 

lua=None
mlib=None
control=None
RE=None
relativeMode=False
useConsole=False
script_version=2 # this script now has a higher priority to libcalab-provided ones

def _loadMlib():
    global mlib, useConsole
    if mlib is None:
        if relativeMode:
            if useConsole: 
                import work.console.libmainlib as _mlib  # built using make console
            else:
                import work.libmainlib as _mlib
        else:
            if useConsole:
                import console.libmainlib as _mlib
            else:
                import libmainlib as _mlib
        mlib=_mlib
def _loadDefault():
    _loadMlib()
    global lua, control, RE, useConsole
    if lua is None:
        if relativeMode:
            import work.luamodule as _lua
            import work.controlmodule as _control
            if RE is None: import work.rendermodule as _RE
        else:
            import luamodule as _lua
            import controlmodule as _control
            if RE is None: import rendermodule as _RE
        lua=_lua
        control=_control
        if RE is None: RE=_RE


def defaultModules():
    _loadDefault()
    return mlib, lua, control, RE

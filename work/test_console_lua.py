
# test_console_lua aims to allow seamless integration with lua.
# see examples below
# for GUI applications, see test_sample_python.py
import os
import sys
import pdb # use pdb.set_trace() for debugging
import code # or use code.interact(...) for debugging. see below.
sys.path.append(os.getcwd())
import libmainlib as m   
import luamodule as lua  # see luamodule.py
import numpy as np 

def main():

    # lua.init_console just creates an invisible pythonWin
    # It won't be shown unless you call m.showMainWin(). 
    # If you need the window/GUI, use test_sample_python.py instead.
    lua.init_console()
    # now you can use lua funtions.
    # test lua-python interfacing
    #lua.out(3)
    #lua.out('asdf')
    #lua.out(m.vector3(3,4,5))

    # let's go to lua
    if len(sys.argv)==2:
        lua.dofile(sys.argv[1])
    else:
        lua.dofile('../lua/console.lua')

if __name__=="__main__":
    main()

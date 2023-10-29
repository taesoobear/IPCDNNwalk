import libmainlib as m
import luamodule as lua
import numpy as np
import pdb

def pyFunction(a,b,c,d):
    print('pyFunction')
    print(a,b,c,d)
    lua.out(a,b,c,d)

def pyFunctionGetData(out1,out2):
    out1.setSize(10)
    out1.ref()[:]=7
    out1.set(0,12)
    out2.resize(10)
    out2.set(0,'asdf')
    out2.set(1,'string2')


def pyFunctionString(a,b):
    print(a,b)

def pyFunction2(a,b,c,d):
    print('pyFunction2')
    print(b)
    c= 50
    b[1] =9
    print(a,b,c,d)

def pyFunction3(a):
    lua.out(a)
    a.ref()[:,:]=2.0

def main():

    lua.init_console()
    # see also this lua file carefully. especially, luaFunction2
    lua.dofile('../lua/testLuaFunction.lua')

    a=m.vector3(3,4,5)
    b=lua.array(np.array([[1,0,0],[ 0,1,0],[ 0,0,1]]))

    lua.F('luaFunction1',  a,b,3, 'asdfasdf')
    lua.F('luaFunction2',  a,b,3, 'asdfasdf')

    c=m.hypermatrixn()
    lua.F('luaFunction3', c)
    print(c.ref().shape)
    print(c.ref())

    print('finished!')

if __name__=="__main__":
    main()

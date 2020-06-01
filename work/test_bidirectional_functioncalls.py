import libmainlib as m
import luamodule as lua
import numpy as np

def pyFunction(a,b,c,d):
    print('pyFunction')
    print(a,b,c,d)
    lua.out(a,b,c,d)

def pyFunction2(a,b,c,d):
    print('pyFunction2')
    print(a,b,c,d)


def main():

    lua.init_console()
    lua.dofile('../lua/testLuaFunction.lua')

    a=m.vector3(3,4,5)
    b=lua.array(np.array([[1,0,0],[ 0,1,0],[ 0,0,1]]))

    lua.F('luaFunction1',  a,b,3, 'asdfasdf')
    lua.F('luaFunction2',  a,b,3, 'asdfasdf')

    print('finished!')

if __name__=="__main__":
    main()

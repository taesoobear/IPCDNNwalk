import libmainlib as m
import luamodule as lua
import rendermodule as RE
import numpy as np
import pdb,sys

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


    #lua.init_console() # -> cannot use GUI components
    RE.createMainWin(sys.argv) # -> can use GUI components.
    # see also this lua file carefully. especially, luaFunction2
    lua.dofile('../lua/testLuaFunction.lua')
    lua.dostring("""  
    print(luaFunction1)
    print(luaFunction2)
    print("many functions are defined.")
    """)

    a=m.vector3(3,4,5)
    b=lua.array(np.array([[1,0,0],[ 0,1,0],[ 0,0,1]]))   

    lua.F('luaFunction1',  a,b,3, 'asdfasdf')
    lua.F('luaFunction2',  a,b,3, 'asdfasdf')

    c=m.hypermatrixn()
    lua.F('luaFunction3', c)
    print(c.ref().shape)
    print(c.ref())

    mat=m.matrixn()
    mat.resize(3,3)
    mat.setAllValue(1.0)
    # test dictionary conversion
    lua.F('NAME.luaFunction4', { 'mat' :mat, 'hyper': c, 0:3, 1:'asdf', 5:'asdfadsf', 7:[0,1,2,3,4]})
    print(mat)

    # test list conversion
    lua.F('luaFunction5', [ mat, c])

    # return values (the return values are always copied so that lua versions can be safely garbage collected. if you need references, use arguments or global variables.)
    mat1, mat2=lua.F('luaFunction6', mat, c)
    print(mat1)
    print(mat2)

    # 아래는 python에서 수행됨. 
    mLoader=RE.createLoader('../Resource/motion/MOB1/hanyang_lowdof_T.wrl')
    print('mLoader in python:', mLoader)

    # 아래는 위와 같은 일을 lua에서 수행함. 
    # use lua instance when you don't want to COPY the result in lua to python side.)
    lua.F_lua('mLoader', 'RE.createLoader', '../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T.wrl')
    lua.F('print', 'mLoader in lua:', lua.instance('mLoader.loader'))
    # equivalent luascript: mLoader=RE.createLoader('...wrl') print(mLoader.loader)
    # 즉, mLoader가 python에도 lua에도 하나씩 있고, 서로 다른 instance임. 

    # the code below uses GUI components.
    # you can use any lua instance as well as python instances as function input.
    # mSkin=RE.createSkin(mLoader, true)
    lua.F_lua('mSkin', 'RE.createSkin', lua.instance('mLoader'), True)

    # member call. mSkin:setTranslation(100,0,0)
    # python으로 포팅되지 않은 멤버함수들은 아래처럼 lua함수를 호출 할 수 있음. 
    lua.M('mSkin', 'setTranslation', 100,0,0)

    # lua.M의 첫번째 argument(==self)는 python에 있던 루아에 있던 모두 동작함. 
    print(lua.M(mLoader.loader, 'getPoseDOF')) 


    # now, let's use a lua class in python
    lua.dostring("""
    A=LUAclass()
    function A:__init(param)
        self.param=param
    end
    function A:printParam(param2)
        printTable(self.param)
        printTable(param2)
    end
    """)

    a=lua.new("A", [1,2,3])
    # print A.param
    print(a.get("param"))
    # call A.printParam
    a('printParam', [3,4,5])

    mLoaderInLua=lua.instance('mLoader')
    print(mLoaderInLua)
    mLoaderInLua.get('loader').printHierarchy() # get('loader') function returns a reference to the VRMLloader in the lua env. (doesn't copy because a lua instance internally uses G (global variables) instead of F (function returns by values)). 

    print('finished!')




if __name__=="__main__":
    main()

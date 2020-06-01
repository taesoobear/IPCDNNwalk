# -*- coding: utf-8 -*
# sample_python aims to allow seamless integration with lua.
# see examples below
import os
import sys
import pdb # use pdb.set_trace() for debugging
import code # or use code.interact(...) for debugging. see below.
sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.path.abspath('..'),'work'))
print(os.path.join(os.path.abspath('..'),'work'))
print(os.getcwd())

# the following three modules are important!!!
import libmainlib as m   
import luamodule as lua  # see luamodule.py
import numpy as np 

def main():

    lua.init_console()
    
    # now you can use lua funtions.
    # test lua-python interfacing
    lua.out(3)
    lua.out('asdf')
    lua.out(m.vector3(3,4,5))
    a=m.vectorn()
    a.assign([1,2,3,4,5])
    lua.out(a)
    a.assign(np.array([5,4,3,2,1]).tolist()) 
    lua.out(a)
    print(a.ref())
    a.ref()[1]=10
    lua.out(a)
    b=m.intvectorn()
    b.assign([1,2,3,4,5])
    lua.out(b)
    c=np.array([[1,0,0],[ 0,1,0],[ 0,0,1]])
    print(c)
    # lua.array converts ndarray to vectorn, matrixn or hypermatrixn
    lua.out(lua.array(c))
    d=m.matrixn(3,3)
    lua.out(d.row(0))
    d.row(0).assign([1,2,3])
    d.row(1).assign([7,3,4])
    d.row(2).assign([3,4,5])
    lua.out(d)
    print(d.row(0).ref())
    print(d.column(0).ref())
    print(d.column(0).ref().tolist())
    print(d.ref().tolist())
    e=m.matrixn(10,10)
    e.setAllValue(0)
    e.row(1).setAllValue(10)
    e.column(1).setAllValue(20)
    e.ref()[0,0]=10
    f=e.range(5,7,5,7).ref()
    f[0,0]=22
    f[1,1]=23
    print(e.ref())

    # low-level apis
    l=m.getPythonWin()
    l.dostring('g_dataset={ 1,2,3}')

    # out=g_dataset[2]
    l.getglobal('g_dataset')
    l.replaceTop(2)
    print('the second element in the table is ', l.popnumber())  # other types are also supported l.popmatrixn()  

    
    print('Starting python console. Type "cont" to finish.')
    print("""
    try the followings:
    v=m.vector3() 
    v.x=3
    lua.out(v)
    v=m.vectorn()
    v.assign([1,2,3,4,5])
    lua.out(v)
    v.set(0,4)
    lua.out(v)
    ...
    """)
    pdb.set_trace()
    print('...')
    print('Starting python interactive console. Type "ctrl+d" to finish.')
    print('Try the same examples above:')
    code.interact(local=dict(globals(), **locals())) 
    print('Starting lua console. Type "cont" to finish.')
    m.getPythonWin().dostring('dbg.console()')



if __name__=="__main__":
    main()

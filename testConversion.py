import os,sys, pdb, math, random
import numpy as np
from libcalab_ogre3d import RE,m, lua,control

#RE.createLuaEnvOnly() # this is necessary only for using lua functions. The following code actually doesn't use lua

print("testing conversion between libcalab and numpy types")
a=lua.mat([[1,2,3,4],[4,5,6,7]])
b=a.vec3ViewCol(0) # view a as vector3N (starting from column 0)
c=a.vec3ViewCol(1) # view a as vector3N (starting from column 1)
d=a.quatViewCol(0) # view a as quaterN (starting from column 0)

print(a, b,c,d)
print('-'*10)
a.row(1).ref()[:]=0
print(a, b,c,d)
print('-'*10)
print('a.vec3ViewCol(0).matView()=', b.matView())
print(a.row(0).toVector3(1))
print(a.row(0).toQuater(0))
print(m.vector3(1,2,3).numpy()) # numpy function copies the data unlike the ref function
print(m.vector3(1,2,3).ref()) # .ref() === .array 
print(m.vector3(1,2,3).array)

np_e=np.random.rand(100,100)
print(np_e)
e=lua.mat(np_e) # convert from numpy or torch or [[]] to matrixn
print(type(e), e)
print(type(e.array), e.array)

# see also lua.vec, lua.ivec which convert to vectorn or intvectorn or boolN
print('finished')

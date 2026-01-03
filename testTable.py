# test lua-like table

import os,sys, pdb, math, random
import numpy as np
from libcalab_ogre3d import RE,m, lua, control

RE.createLuaEnvOnly() # this is necessary only if you are actually using lua functions. Many of the following lines don't use lua

table1=lua.Table(1,2,3,7, entry1=3, data=[1,2,3]) # implemented as a dictionary (+optionally a list)
table1.entry3=4

print(table1)

print('entry1', table1.entry1)
print('entry2:', table1.entry2) # doesn't throw error  unlike python dict
lua.F('printTable', table1)  # conversion to lua table (deepcopy)

lua.dostring("""
             table2={ 1,2,3, options={b=4}}
             function getTable2()
                return table2
             end
             """)
lua.F('printTable', lua.instance('table2'))


table3=lua.Table(1,2,3, options=lua.Table(b=4))
print('print(table3) in python:', table3)

print('printTable(table3) in lua:')
lua.F('printTable', table3) # conversion to lua table (deepcopy)


table2_fromLua=lua.F('getTable2')  # conversion to python table (deepcopy)
print(table2_fromLua)
print('table2_fromLua(1)==', table2_fromLua(1)) # lua.Table uses 0-indexing
table2_fromLua.set(1, 10)
print('table2_fromLua(1)==', table2_fromLua(1)) # lua.Table uses 0-indexing
print('table2_fromLua.array==', table2_fromLua.array)  # to access only the list part of the table.




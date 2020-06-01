
from numpy import * 
import pdb
a=arange(16).reshape((4,2,2))

b=array([[[ 0,  1],
        [ 2,  3]],

        [[ 4,  5],
        [ 6,  7]],

        [[ 8,  9],
        [10, 11]],

        [[12, 13],
        [14, 15]]])


print 'a=', a
print

print('a[0]=')
print(a[0])
print


print 'a.reshape(2,2,2,2)='
print a.reshape(2,2,2,2)
print

print 'a.reshape(2,2,4)='
print a.reshape(2,2,4)
print

c=a.reshape(4,4)
print 'c=', c
print

d=a.reshape(2,2,2,2).swapaxes(1,2).reshape(4,-1)
print 'd=', d
print

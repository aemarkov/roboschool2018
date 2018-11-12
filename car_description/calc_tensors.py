# Calculate inertia tensors

import math
import numpy as np

def calc_box():
    print('=== Box ===')
    m = float(input('mass   > '))
    w = float(input('width  > '))
    h = float(input('height > '))
    d = float(input('depth  > '))

    I = np.zeros([3,3])
    I[0,0] = 1/12*m*(h*h + d*d)
    I[1,1] = 1/12*m*(w*w + d*d)
    I[2,2] = 1/12*m*(w*w + h*h)    

    return I

def calc_cylinder():
    print('=== Cylinder ===')
    m = float(input('mass   > '))
    r = float(input('radius > '))
    h = float(input('height > '))    

    I = np.zeros([3,3])
    I[0,0] = 1/12*m*(3*r*r + h*h)
    I[1,1] = 1/12*m*(3*r*r + h*h)
    I[2,2] = 1/2*m*r

    return I
    
bodies = {
    'box': calc_box,
    'cylinder': calc_cylinder
}

print('Body type:')
for b in bodies:
    print('  %s' % b)


t = input('> ')
if t in bodies:
    I = bodies[t]()
    i_lower = np.tril_indices(3, -1)
    I[i_lower] = I.T[i_lower]
    print(I)
else:
    print('Invalid body type')
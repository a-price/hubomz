import matplotlib.pyplot as plt
import numpy as np
import sys

data = np.genfromtxt(sys.argv[1], delimiter=',')
print data

time = data[:,0]
zref = data[:,1]
coms = data[:,2]
zmps = data[:,3]

plt.plot(time, zref, label='ZMP Reference')
plt.plot(time, coms, label='COM pos')
plt.plot(time, zmps, label='ZMP')
plt.legend(loc='lower left', prop={'size':10})
plt.show()







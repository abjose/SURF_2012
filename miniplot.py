#!/usr/bin/env python

from pylab import *
import numpy as np

print sys.argv
for i,fname in enumerate(sys.argv[1:]):
    r = []
    f = open(fname, 'r')
    for line in f: r.append(float(line))

    # for polar plot
    step = 0.007
    theta = np.arange(-1,(step*len(r)-1),step)
    polar(theta,r, ',')

    # for ...linear? plot
    #scatter(range(len(r)),r)

    savefig("test"+str(i)+".png")
    clf()
    #show()

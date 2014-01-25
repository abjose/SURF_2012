import numpy as np
import math

p_0    = 0.6 #0.7 
p_occ  = 0.8 #0.9
p_free = 1-p_occ
p_max = 10#p_occ+0.1#5
p_min = 0#p_free-0.1#5

l_0    = math.log(p_0/(1-p_0))
l_occ  = math.log(p_occ/(1-p_occ))
l_free = math.log(p_free/(1-p_free))
l_max = p_max#math.log(p_max/(1-p_max))
l_min = p_min#math.log(p_min/(1-p_min))

class OccGrid:
    def __init__(self, corner, size, res, start,step,scanmin,scanmax, forget=True):
        self.scanstart,self.scanstep = start,step  # laser parameters
        self.scanmax,self.scanmin = scanmax,scanmin
        self.cx,self.cy = corner # 'lower left' point of space
        self.botX,self.botY,self.botH = (0,0,0) # bot initial position
        self.dim = size//res
        self.res = res # length units per grid cell (e.g. meters per cell)
        self.grid = np.zeros((self.dim,self.dim))
        self.grid.fill(l_0)
        self.skip = 5  # downsample to 1 every (skip) readings
        self.maxgrid = np.ones((self.dim,self.dim))*l_max
        self.mingrid = np.ones((self.dim,self.dim))*l_min
        self.good = lambda x: self.scanmin+0.1 < x < self.scanmax-0.1
        self.occupied = 3#p_max-0.01#l_occ # for external access...
        self.forget = 0.05 # forgetting factor
        self.forgetful = forget

    def setPose(self, x,y,h):
        self.botX,self.botY, self.botH = x,y,h
    
    def scanToGrid(self, scan):
        """ Convert array of ranges to grid-sized binary occupancy matrix """
        th = np.arange(0,len(scan),self.skip)*self.scanstep+self.scanstart
        scan = np.array(scan[::self.skip])

        # convert from ranges to points in bot's frame
        p = scan*np.array([np.cos(th),np.sin(th)])

        # rotate and translate into world frame
        R = np.matrix([[np.cos(self.botH),-np.sin(self.botH)],
                       [np.sin(self.botH), np.cos(self.botH)]])
        p = (R*p).T + [self.botX-self.cx,self.botY-self.cy]
         
        # scale into grid points
        r,c = p[:,1]//self.res, p[:,0]//self.res
        
        # now return grid-like structure with '1's at occupied places
        a = np.zeros((self.dim,self.dim))
        for i in range(len(scan)):
            if self.good(scan[i]):
                if r[i,0] > 0 and c[i,0] > 0:
                    try: a[r[i,0],c[i,0]] = 1
                    except Exception,e: pass
        return a

    def scanInterior(self, scan):
        """ Return list of ranges interior to those in scan """
        # could also add 'surroundings' in addition to specific point
        scan = np.array(scan)
        iters = int(max(scan)//self.res)
        inside = np.zeros((self.dim,self.dim))
        
        for r in range(iters):
            shell = scan - r*self.res
            inside += self.scanToGrid(shell)

        return np.minimum(inside, np.ones((self.dim,self.dim))) 

    def insertScan(self, scan):
        edge = self.scanToGrid(scan)
        ins = self.scanInterior(scan)
        ins = np.maximum(ins-edge, np.zeros((self.dim,self.dim)))
        free = (np.ones((self.dim,self.dim))-edge-ins)*self.grid

        prcpt = np.zeros((self.dim,self.dim))
        prcpt += edge*l_occ - edge*l_0
        if self.forgetful:
            # for dynamic environments
            prcpt += ins*l_free - ins*l_0
            prcpt += self.forget*(l_0-free)
        self.grid += prcpt
        self.grid = np.maximum(np.minimum(self.grid,self.maxgrid),self.mingrid)
        #print np.max(self.grid)

    def printGrid(self):
        """ ascii printout of grid """
        for row in self.grid:
            print "|".join(['*' if c > l_occ else ' ' for c in row])          

    def getScatterVectors(self):
        """ Return arrays for x and y of occupied points """
        xl,yl = [],[]
        #for i in range(len(self.grid)):
        for r,row in enumerate(self.grid):
            for c,col in enumerate(row):
                if col > self.occupied:
                    xl.append(c)
                    yl.append(r)
        #return xl,yl
        return np.array(xl)*self.res+self.cx,np.array(yl)*self.res+self.cy

if __name__ == '__main__':
    import cProfile

    # (following tests on 1000 scans at 0.1 res, 5 skip)
    # with vectors: ~ 1.692 seconds, 280,000 function calls
    # with grids: ~ 0.855 seconds, 160,000 function calls
    maxdist = 9.9
    start = -np.pi/2 
    step = 0.001
    scan = np.random.rand(1000)*maxdist
    g = OccGrid((-10,-10), 20,.1, start,step, 0, maxdist)
    
    print 'starting insertion'
    cProfile.run('g.insertScan(scan)')
    print 'done with insertion'

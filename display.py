import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.backends.backend_agg as agg
import numpy as np
import pygame
from tesseral import TessTools
from pygame.locals import *


class Display:
    def __init__(self, mapsize=None):
        """Initialize 2D display for occupancy and pose data.

        mapsize defines the region of interest, currently meaning how
        the axis limits are set; default is not to set axis limits (None).
        """
        self.edge = 400
        fig = plt.figure(figsize=[4, 4], dpi=100,)
        self.ax = fig.gca()
        self.marker = '.'
        self.mapsize = mapsize
        self.canvas = agg.FigureCanvasAgg(fig)
        self.renderer = self.canvas.get_renderer()
        window = pygame.display.set_mode((800, self.edge), DOUBLEBUF)
        self.screen = pygame.display.get_surface()
        self.size = self.canvas.get_width_height()
        pygame.init()
        self.update()

    def clear(self):
        self.ax.clear()

    def blit(self):
        self.canvas.draw()
        raw_data = self.renderer.tostring_rgb()
        surf = pygame.image.fromstring(raw_data, self.size, "RGB")
        self.screen.blit(surf, (self.edge,0))
        self.update()

    def pose2d(self, p):
        """ Overlay the robot on both the scatter and quadtree drawing
        p is a NumPy ndarray of the form [x,y,theta].
        """
        arrow_len = .1
        self.ax.plot([p[0], p[0]+arrow_len*np.cos(p[2])],
                     [p[1], p[1]+arrow_len*np.sin(p[2])], '-')
        self.ax.plot(p[0], p[1], 'o')
        self.ax.axis('equal')
        # and on quadtree
        if self.mapsize:
            circ_rad = 10
            cx,cy = self.mapsize[0],self.mapsize[2],
            s = self.mapsize[1]-cx # assumes square...
            qx,qy = [self.edge-int(self.edge-((p[0]-cx)/float(s))*self.edge),
                     int(self.edge-((p[1]-cy)/float(s))*self.edge)]
            line = (((qx,qy)),
                    (qx+circ_rad*np.cos(p[2]),qy-circ_rad*np.sin(p[2])))
            pygame.draw.circle(self.screen,(255,0,0),(qx,qy),circ_rad)
            pygame.draw.aaline(self.screen,(0,0,255),line[0],line[1],1)

    def scatter(self,x,y):
        """ Draw a scatter plot of the occupancy grid """
        self.ax.scatter(x,y,color='black')#,marker=self.marker)
        self.ax.axis('equal')
        if self.mapsize is not None:
            self.ax.axis(self.mapsize)

    def quad(self,t):
        """ Draw the quadtree """
        # expected format: [tessaddr0|mrk0,tessaddr1|mrk1...]

        #desc = str(t).split('\n')
        pygame.draw.rect(self.screen,(0,0,0),[0,0,self.edge,self.edge])
        for n in t.tree:#desc:
            addr,mrk=n.addr,n.mark()#addr.split('|')
            (llx,lly),(urx,ury) = TessTools.bound(addr,self.edge)
            color = (255,255,255)
            if mrk == 'occ': color = (0,0,0)
            elif mrk == 'loc': color = (0,0,255)
            elif mrk == 'path': 
                if n.marks['hood'] != 0: color = (204,102,102)
                else: color = (32,178,170)
            elif mrk == 'hood': color = (255,160,122)
            elif mrk == 'chg': color = (165,42,42)
            elif mrk == 'goals': 
                if n.marks['path'] != 0: color = (32,255,170)
                else: color = (0,255,0)
            # shift so there's a black border...oy vey
            rect = np.array([lly+1,self.edge-urx+1,
                             abs(llx-urx)-1, abs(lly-ury)-1])
            pygame.draw.rect(self.screen,color,rect)

    def update(self):
        pygame.display.flip()
        
if __name__=='__main__':
    d = Display()

    addrs = ['00|2','001|0','02|1','03|0']
    for i in range(50):
        n=1000
        x = np.random.rand(n)*(4+i)
        y = np.random.rand(n)*4
        d.scatter(x,y)
        d.quad(addrs)
        d.update()

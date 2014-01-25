from PIL import Image, ImageDraw

class RNode:
    ''' A rectangular'region' in space, meant to be used as nodes in a Qtree .

    Note: An RNode contains points from its start up to (not including)
    its size, i.e. x s.t. self.x <= x < self.x+self.size. '''

    # Class-scope variables
    #----------start-----------#
    # Smallest space a single node can cover
    minSize = 5.
    # Nearness cutoff
    nearDist = min(minSize/2.0, 1.0)
    #-----------end------------#

    def __init__(self, (x,y),size, mark, corner):
        ''' Initialize node '''
        assert size >= RNode.minSize
        self.x, self.y = x,y
        self.s = size
        self.m = mark
        self.corner = corner

    def split(self):
        ''' Splits node into quadrants.

        Returns ordered list: [up-right,up-left,down-left,down-right].
        Feel somewhat justified forcing 4, as specifically (2D)node '''
        n = self.s/2.0
        try:
            q1 = RNode((self.x + n, self.y), n, self.m, 1)
            q2 = RNode((self.x, self.y), n, self.m, 2)
            q3 = RNode((self.x, self.y + n), n, self.m, 3)
            q4 = RNode((self.x + n, self.y + n), n, self.m, 4)
        except AssertionError:
            #print 'Stopping split: Children too small'
            return None
        return [q1,q2,q3,q4]

    def contains(self, (x,y), closed=False):
        ''' Return true if this node contains this point, false otherwise '''
        if closed:
            return (self.x < x < self.x+self.s) and (self.y < y < self.y+self.s)
        return (self.x <= x < self.x+self.s) and (self.y <= y < self.y+self.s) 

    def setMark(self, mark):
        self.m = mark

    def getMark(self):
        return self.m

    def getSize(self):
        return self.s

    def getCenter(self):
        m = self.s/2.0
        return (self.x+m, self.y+m)

    def getOrigin(self):
        return (self.x,self.y)

    def getDescription(self):
        ''' Get string description of node '''
        return str(self.m)
        

    def isNear(self, node):
        ''' Return true if passed node is adjacent to this node '''
        if  node.getSize() < self.s:
            return node.isNear(self)
        x,y,s = self.x,self.y,self.s
        corners = [(x,y),(x+s,y),(x,y+s),(x+s,y+s)]
        contact = [node.isNearCorner(c) for c in corners]
        # verify adjacent and not diagonal
        adjacent = contact.count(0) != 4
        diagonal = contact.count(2) == 1 and contact.count(1) == 0
        return adjacent and not diagonal

    def isNearCorner(self, (cx,cy)):
        ''' Check if corner c is adjacent, according to nearDist. '''
        x,y,s = self.x, self.y, self.s
        a = x <= cx <= x+s and min(abs(y-cy),abs(y+s-cy)) < RNode.nearDist
        b = y <= cy <= y+s and min(abs(x-cx),abs(x+s-cx)) < RNode.nearDist
        return a+b

    def printNode(self):
        ''' For debugging -- print node contents '''
        print str(self.x) + ", " + str(self.y) + " :: " + str(self.s)

    def drawNode(self, im, color = None):
        ''' For debugging -- draw node in image. '''
        x,y,s = self.x,self.y,self.s
        w = (255,255,255)
        r = (255,0,0)
        g = (0,255,0)
        b = (0,0,0)
        if color:
            im.rectangle([x,y,x+s,y+s], outline = b, fill = color)
        elif self.m == 0:
            im.rectangle([x,y,x+s,y+s], outline = b, fill = w)
        elif self.m == 1:
            im.rectangle([x,y,x+s,y+s], outline = b, fill = b)
        elif self.m == 9:  # for path-drawing purposes
            im.rectangle([x,y,x+s,y+s], outline = b, fill = g)
        else:
            im.rectangle([x,y,x+s,y+s], outline = b, fill = r)

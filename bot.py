from quadtree import QTree
import random as r
import math

class Bot: 
    def __init__(self, x,y, cells):
        ''' initialize bot '''
        self.loc = x,y
        self.scale = 5.
        self.quadsize = 500#self.scale*cells
        self.quad = QTree(self.quadsize)  # proper size?
        self.path = []
        self.goals = []
        self.currGoal = None
        self.i = 0

    def getLoc(self):
        ''' Return current location as a tuple '''
        return self.loc

    def setLoc(self, loc):
        ''' Sets current location, expects a tuple '''
        self.loc = loc

    def setGoals(self, goals):
        ''' Set internal goals representation '''
        self.goals = goals
        if goals:
            print "GOAL EXISTS"
            self.currGoal,val = goals[0]

    def move(self):
        ''' Move to the next grid cell on the current path '''
        if self.gridpath:
            self.setLoc(self.gridpath.pop())

    def update(self, obstacles):
        ''' Update occupancy grid based on surrounding obstacles '''
        
        for obs in obstacles:
            (x,y),val = obs
            self.quad.setMark(self.scale*x,self.scale*y,val)
        self.i += 1
        self.quad.drawTree("botTree"+str(self.i))
        print len(self.quad.Adj.nodes())
        # SHOULD PLAN HERE, BUT ONLY IF ENVIRONMENT CHANGED!!!!!!!!
        self.plan()
    
    def getNextMove(self):
        ''' Return how current loc should be changed to move towards goal '''
        # Should perhaps do a 'get next step" function instead.
        # just get the center of the next node in the path, then
        # see which difference is largest and move in that direction
        #  Once close enough, pop from path
        x,y = self.loc
        g = self.currGoal
        if g:
            gx,gy = g
            if gx == x and gy == y: return (0,0)
            if (abs(gx-x)>abs(gy-y)):  # should move horizontally
                return (math.copysign(1,gx-x),0)
            else:
                return (0,math.copysign(1,gy-y))

    def plan(self):
        ''' Update current path. Handle planning and patching current plan. '''
        x,y = self.loc
        if not self.currGoal:
            print "NO GOAL!"
            return None
        gx,gy = self.currGoal
        currNode = self.quad.getNode(x,y)
        goalNode = self.quad.getNode(gx,gy)
        #if currNode == goalNode:
        #    self.currGoal = self.goals.pop[0]
        self.path = self.quad.getPath(currNode, goalNode)
    
    
if __name__ == '__main__':
    b = Bot(0,0,40)
    # test quadtree...
    for i in range(50):
        x,y = r.random()*(b.quadsize-3)+1,r.random()*(b.quadsize-3)+1
        b.quad.setMark(x,y,1)
        b.quad.drawTree('test'+str(i))

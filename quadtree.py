from node import RNode
from PIL import Image, ImageDraw
import random as r  # for testing purposes
import networkx as nx

class QTree:
    ''' A quadtree with constant-time accessible weighted adjacency graph

    Defined by two NetworkX graphs:
    1) A graph defining connectivity between all leaf nodes
    2) A digraph defining genealogy between all nodes
    By maintaining both, we can quickly access the quadtree's adjacency graph,
    and quickly modify (increase and decrease) the tree's resolution. '''

    # Class-scope variables
    #----------start-----------#
    # Number of children to give a parent node (4 for quadtree)
    numChildren = 4
    # Lower bound on coordinates covered by quadtree (eg: x,y = 0,0)
    lowBound = (0.,0.)
    # Mark meaning 'unmarked' (i.e. nothing here)
    clearMark = 0
    # Mark meaning 'obstacle'
    obstacleMark = 1
    #-----------end------------#

    def __init__(self, size):
        ''' Initialize quadtree with a root node '''
        self.Gen = nx.DiGraph()  # genealogy graph
        self.Adj = nx.Graph()    # adjacency graph
        self.root = RNode(QTree.lowBound,size, QTree.clearMark, "root")
        self.reset()

    def reset(self):
        self.Gen.clear()
        self.Adj.clear()
        self.Gen.add_node(self.root)
        self.Adj.add_node(self.root)

    def setMark(self, x,y, val):
        ''' Mark node containing point x,y with val. Refines graph. 

        Attempts to reduce resolution if all children share same mark.'''
        node = self.refine(self.root, x,y, True)
        node.setMark(val)
        if val == QTree.obstacleMark:
            self.disconnect(node)
        while node != None: # change to while node:?
            node = self.tryJoin(node)
    
    def getMark(self, x,y):
        ''' Return mark value of smallest node containing point x,y '''
        return self.getNode().getMark()

    def getNode(self, x,y):
        ''' Return smallest node containing x,y '''
        return self.refine(self.root, x,y)
   
    def refine(self, parent,  x,y, force=False):
        ''' Refines quadtree around x,y.  If force, will 'dig' until size limit.

        Returns the final "found" node '''
        children = self.getChildren(parent)
        if children:
            bestkid = filter(lambda c: c.contains((x,y)), children)[0]
            return self.refine(bestkid, x,y, force)
        if force:
            if self.setChildren(parent):
                children = self.getChildren(parent)
                bestkid = filter(lambda c: c.contains((x,y)), children)[0]
                return self.refine(bestkid, x,y, True)
        return parent
        
    def disconnect(self, node):
        ''' Disconnect target node from its neighbors in Adj graph '''
        self.Adj.remove_edges_from(self.Adj.edges(node))

    def tryJoin(self, node):
        ''' Try to join/merge a node with its siblings '''
        sib = self.getSiblings(node)
        # if sib is non-empty and all siblings have the same mark...
        if sib and not True in [s.getMark() != node.getMark() for s in sib]:
            # and no sibling has kids...
            sibKids = [self.getChildren(s) for s in sib]
            if True in [k != [] for k in sibKids]: return None
            # kill the siblings
            parent = self.getParent(node)
            self.killChildren(parent)
            return parent
        return None

    def getParent(self, node):
        ''' Return a node's parent (empty if none) '''
        return self.Gen.predecessors(node)[0]

    def getSiblings(self, node):
        ''' Return nodes with same parent as passed node (inclusive) '''
        # return None if at root
        if self.Gen.in_degree(node) == 0: return None
        # Otherwise, return siblings
        return self.getChildren(self.getParent(node))

    def getAdjacent(self, node):
        ''' Return nodes adjacent to passed node according to Adj '''
        return self.Adj.neighbors(node)

    def getChildren(self, node):
        ''' Return successors (empty if none) '''
        return self.Gen.successors(node)

    def setChildren(self, node):
        ''' Update graphs s.t. passed node has 4 children (if large enough) '''
        children  = node.split()
        # Make sure Node doesn't already have children, and...is fertile.
        if self.Gen.out_degree(node) == 0 and children:
            q1,q2,q3,q4 = children
            # Get and connect children:
            self.Gen.add_edges_from([(node,c) for c in children])
            self.Adj.add_edges_from([(q1,q2),(q2,q3),(q3,q4),(q4,q1)])
            # connect if adjacent -- fairly inefficient
            #neighbors = self.Adj.neighbors(node)
            try: self.Adj.remove_node(node)
            except nx.exception.NetworkXError: print "Couldn't remove parent"
            neighbors = self.Adj.nodes()
            for n in neighbors:
                for c in children:
                    if n.isNear(c):
                        # If using weights, set here (and when connect children)
                        self.Adj.add_edge(n,c)
            return True
        return False

    def killChildren(self, node):
        ''' "Reverse refine" by removing children of node, updating Adj '''
        children = self.getChildren(node)
        if children:
            node.setMark(children[0].getMark())  # all should be same...
            for c in children:
               if c in self.Adj.nodes():
                    kidEdges = [(node,t) for t in self.Adj.neighbors(c)]
                    self.Adj.add_edges_from(kidEdges)
                    self.Adj.remove_node(c)
                    self.Gen.remove_node(c)

    def getPath(self, start, end):
        ''' Return a list containing a path over Adj '''
        try: path = nx.shortest_path(self.Adj, source=start, target=end)
        except nx.exception.NetworkXNoPath:
            return None
        
    def drawPath(self, start, end, i):
        ''' Draw a path in green from the start to the end node '''
        markList = []
        path = nx.shortest_path(self.Adj, source=start, target=end)
        print ":::BEGIN PATH:::"
        for n in path:
            print(n.getCenter())
            markList.append(n.getMark())
            n.setMark(9)  # force green
        # for debugging...
        self.drawTree('path'+str(i))
        # now reset path
        for j,n in enumerate(path):
            n.setMark(markList[j])

    def drawTree(self, name):
        ''' Save a picture of the quadtree.  For testing. '''
        w = (255,255,255)
        s = self.root.getSize()+1
        im = Image.new("RGB", (s,s), w)
        imdraw = ImageDraw.Draw(im)
        for n in self.Adj.nodes():
            n.drawNode(imdraw)
        im.save(name+".png", "PNG")
        del imdraw

    def __str__(self):
        """ Return string representation of QTree """
        # NOTE: right now, only saves non-empty squares
        final = ''
        for n in self.Adj.nodes():
            if n.getMark() == QTree.clearMark: continue 
            out = '0|' + n.getDescription() + '\n'
            while n.corner != 'root':
                out = str(n.corner) + '.' + out
                n = self.getParent(n)
            final += out
        return final

    def fromString(self, string):
        ''' Reset the QTree so it's like the structure specified in string '''
        self.reset()
        # THIS IS PRETTY UGLY!
        for line in string.splitlines():
            path = [int(c) for c in line.split('|')[0].split('.')]
            mark = int(line.split('|')[1][0])
            node = self.root
            if len(path) <= 1:
                self.root.setMark(mark)
            else:
                for i,c in enumerate(path):
                    if path[i] == QTree.clearMark:
                        node.setMark(mark)
                    else:
                        children = self.getChildren(node)
                        if not children:
                            self.setChildren(node)
                            children = self.getChildren(node)
                        node = [n for n in  children if n.corner == c][0]

if __name__ == '__main__':
    
    size = 200
    t = QTree(size)

    
    # t.drawTree('imgA0')
    for i in range(0,100,6):
        t.setMark(10+i,10+i, 1)
        #t.drawTree('imgA'+str(i))
    for i in range(0,100,20):
        t.setMark(10+i+7,10+i, 2)
        #t.drawTree('imgB'+str(i))
    print t
    '''
    for i in range(0,110,6):
        t.setMark(10+i,10+i, 0)
        t.drawTree('imgC'+str(i))
    for i in range(0,100,20):
        t.setMark(10+i+7,10+i, 0)
        t.drawTree('imgD'+str(i))
    '''

    '''
    t.drawTree('imgA')
    for i in range(10,100,5):
        t.setMark(i,50, 1)
        t.setMark(10+i,10+i, 1)
        t.drawPath(t.getNode(10,10), t.getNode(10+i,20+i), i)
        t.drawTree('imgA'+str(i))
    t.drawPath(t.getNode(0,0), t.getNode(120,120), 1)
    '''
    
    '''
    for i in range(50):
        x,y = r.random()*195+1,r.random()*195+1
        t.setMark(x,y,1)
    output = str(t)
    t.drawTree('treeBefore')
    for i in range(10):
        x,y = r.random()*195+1,r.random()*195+1
        t.setMark(x,y,1) 
    t.drawTree('treeModified')
    t.fromString(output)
    t.drawTree('treeAfter')
    '''

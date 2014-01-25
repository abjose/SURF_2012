from tesseral import TessTools, TessNode as node
import numpy as np
import copy

# note: consider modifying s.t. on initialization pass a function for converting
# points from world frame to quadtree frame, so that points could always
# be passed in world frame?

class TessTree:
    """ Track a list of TessNodes and maintain a size for them """
    def __init__(self, size=100, maxdepth=4):
        self.d = max(0, int(maxdepth)) 
        self.s = float(size)
        self.old = None
        self.tree = []
        self.update(node())

    def __str__(self):
        out = ''
        for n in self.tree: out += str(n) + '\n'
        return out[:-1]

    def insert(self, x,y, mark='occ', lvl=None):
        """ Insert TessNode(s) s.t. (x,y) has value mark """
        if lvl == None: lvl = self.d
        n = node(self.getAddr(x,y)[:lvl])
        n.setMark(mark)
        oldtree = self.tree[:] #copy
        self.update(n)

    def insertRegion(self, (x,y), lvl, mark):
        """ Find cell at spec'd lvl of refinement containing passed point.
        Then attempt to insert passed mark into found cell/children.
        """
        self.insert(x,y, mark, lvl)
        
    def query(self, x,y):
        """ Return highest-priority mark at (x,y) """
        target = self.getMatches(self.getAddr(x,y), exact=True)
        if len(target) == 1:
            return target[0].mark()
        raise Exception("[Query] Found no or multiple matches")

    def clear(self):
        """ Cleans occgrid, preserving any set goals. """
        self.update(node(addr=''), merge=False, ignore=['goals'])

    def getMatches(self, addr, exact=False, either=False):
        """ Return list of nodes in TessTree equal to or inside addr 
        If either, can subset or superset"""
        sim = []
        for n in self.tree:
            if TessTools.subset(sub=n.addr,sup=addr):
                if not exact or (exact and len(addr) == len(n)):
                    sim.append(n)
            elif either and not exact:
                if TessTools.subset(sub=addr,sup=n.addr):
                    sim.append(n)
        return sim
    
    def getMarked(self, mark):
        """ Return list of nodes with passed mark set """
        def f(n):
            if mark == 'goals': return n.marks[mark] != []
            else: return n.marks[mark] > 0
        return filter(f, self.tree)
        #return filter(lambda n: n.marks[mark] > 0, self.tree)
        #return [k for k in self.tree if k.mark() == mark]

    def getAddr(self, x,y, corner=False, exists=False):
        """ Get addr of the cell containing (x,y) (origin at 'bottom-left') 
        Normally 'digs' all the way down 
        If corner is true, returns the x,y location of the bottom corner of cell
        If 'exists' is True, will truncate to match a pre-existing cell
        (Note: corner and exists both being true will yield misleading results)
        """
        # should make behavior less confusing, clean up description
        cells = np.array([['0','1'],['2','3']])
        cx, cy = 0.0, 0.0
        addr = ''
        for i in range(1,self.d+1):
            minsize = self.s / (2**i)
            dx,dy = x > cx+minsize, y > cy+minsize
            cx,cy = cx+minsize*dx, cy+minsize*dy
            addr += cells[dx,dy]
        if corner:
            return addr, (cx,cy)
        if exists:
            if self.getMatches(addr): return addr
            for i in range(1,len(addr)+1):
                if self.getMatches(addr[:-i]):
                    return addr[:-i]
            return False
        return addr
    
    def getAdj(self, addr, addrs=False):
        """ Return list of nodes in tree adjacent to passed address """
        adjacent = []
        for n in self.tree:
            if TessTools.isAdj(n.addr,addr): 
                if addrs: 
                    adjacent.append(n.addr)
                else: 
                    adjacent.append(n)
        return adjacent

    def adjSet(self, addrList, addrs=False):
        """ Return set of adjacent nodes/addrs for all addrs in addrList """
        addrList = set(addrList)
        adj = set()
        for a in addrList:
            adj |= set(self.getAdj(a, addrs))
        return adj

    def goallist(self):
        """ Generates list of lists of cells belonging to specific goals """
        d = dict()
        for n in self.tree:
            for g in n.marks['goals']:
                if g in d.keys(): d[g].append(n)
                else: d[g] = [n]

        # To preserve order, convert to list based on goal labeling
        l, i = [], 1
        while 'g'+ str(i) in d.keys():
            l.append(d['g'+str(i)])
            i += 1
        if i < len(d.keys()):
            print "Warning! Might not have found all goals in goallist. Make sure goals are sequential, in format 'g#' starting at # == 1"
        return l#d

    def insertOG(self, og, l_occ, exp=1):
        """ Given a square occupancy grid, attempts to 'insert' into quadtree.
        Expands occupied cells by exp (i.e. replace single occupied grid cell
        with exp by exp square).
        """
        # does not expand yet
        for r,row in enumerate(og):
            for c,col in enumerate(row):
                if og[r,c] > l_occ: 
                    self.insert(c,r)

    def worldToQuad(self, (x,y), (start,end)):
        """ Takes a point x,y in a square 'space' with edges from start to end, 
        converts to QuadTree coordinates (i.e. point (2,1) in world with edges
        from -3 to 7...
        """
        qx = ((x-start)/float(end-start))*self.s
        qy = ((y-start)/float(end-start))*self.s
        #return int(qx),int(qy)
        return qx,qy

    def quadToWorld(self, (x,y), (start,end)):
        """Takes a point x,y in quadtree, converts to coordinates in a square
        'space' with edges from start to end.
        """
        # modify to take lists?
        wx = x/float(self.s)*(end-start)+start
        wy = y/float(self.s)*(end-start)+start
        return wx,wy

    def treeDiff(self):
        """ Find differences between passed list of TessNodes and
        current list of TessNodes. Return as a dict with keys being modified
        addrs in oldtree, and their values being associated lists of new addrs
        """
        diffdict = dict()
        if self.old == None: 
            print 'No reference tree, skipping diff and saving.'
        else:
            oldtree = set(self.old)
            newtree = set(self.tree)
        
            mod_addrs = oldtree - newtree
            add_addrs = newtree - oldtree
        
            for mod in mod_addrs:
                modlist = []
                for add in add_addrs:
                    if add.addr == mod.addr:
                        if add.marks['occ'] != mod.marks['occ']:
                            #if add.mark() != 'occ':
                            modlist.append(add.addr)
                    elif mod.isAncestor(add) or add.isAncestor(mod):
                        #modlist.append(add.addr)
                        #if add.mark() != 'occ':
                        modlist.append(add.addr)
                if modlist != []: diffdict[mod.addr] = modlist
        self.old = copy.deepcopy(self.tree)#self.tree[:]
        return diffdict

    def update(self, n, ignore=[], merge=True, isSib=False):
        """ Insert TessNode and clean tree s.t. no identical siblings, etc. 
        Merge will 'add' marks, otherwise will overwrite.
        If not merging, can add marks to 'ignore' to not overwrite them.
        (Like if you wanted to reset all occupancies without changing goals)
        Return true if resulted directly in insertion, false otherwise
        """
        dup,anc,sib,ch = [],[],[],[]
        # get duplicates, siblings, children
        for t in self.tree:
            if t.isAncestor(n): anc.append(t)
            elif n.isAncestor(t): ch.append(t)
            elif n.isSibling(t): sib.append(t)
            elif n.addr == t.addr: dup.append(t)

        # handle duplicates
        for d in dup: 
            if merge: n.merge(d)
            self.tree.remove(d)

        # handle ancestors, 'lazy breeding'
        for a in anc:
            if len(a) < len(n)-1:
                for b in a.breed(): self.tree.append(b)
                self.tree.remove(a)
                self.update(n)
                return False
            if merge: n.merge(a)
            self.tree.remove(a)
       
        # if children, apply self to children and delete self
        if ch != []:
            for c in ch:
                if merge: c.merge(n)
                else: c.apply(n, ignore)
                self.trymerge(c)
            return False
        
        if isSib: 
            self.tree.append(n)
            return True

        for s in n.siblings(): 
            for a in anc: s.merge(a) 
            self.update(s, isSib=True)
            #if self.update(s, isSib=True): sib.append(s)

        #print 'trying insertion'
        if not self.trymerge(n):
            self.tree.append(n)
            return True

    def trymerge(self, n):
        sib = []
        for t in self.tree:
            if t.isSibling(n): sib.append(t)
        sib.append(n)
        #print 'merging: ' + str([sibl.addr for sibl in sib])
        merged = None
        if len(sib) == 4 and len(n) != 1:
            merged = node(addr=n.addr[:-1])
            for s in sib:
                if merged != None and s.mergeable(n): merged.merge(s)
                else: merged = None
         
        if merged != None:
            for s in sib:
                if s in self.tree: self.tree.remove(s)
            self.tree.append(merged)
            self.trymerge(merged)
            return True
        return False

    #def grow(self, c):
    #    """ 'Grow' the quadtree, making current tree corner c of new """
    #    Purposefully not implemented (to avoid 'feature creep') but might be
    #    useful, and would be very easy to implement with tesseral addresses.

if __name__ == '__main__':


    # !!!!!!
    # should write down some ideas for functionality tests of quadtree
    # (so can make sure changes don't introduce bugs)...

    t = TessTree()
    #for i in range(1,50,4):
    #    t.insert(5+i,51)
    #print t
    
    #print
    #t.clear()
    #print t
    
    print t.getAddr(50,50)
    
    """
    t = TessTree()
    #print t
    n1 = node(addr='11111')
    #n1.setMark('occ')
    t.update(n1)
    print
    print t
   
    n2 = node(addr='1111')
    #n2.setMark('occ')
    t.update(n2)
    print
    print t
    #n1.clear()
    #t.update(n1, merge=False)
    #print
    #print t
    """

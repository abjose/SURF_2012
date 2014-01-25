import numpy as np

# Tables from http://www.csc.liv.ac.uk/~frans/OldResearch/dGKBIS/tesseral.html
# Lookup table for tesseral addition
add = np.array([['00','01','02','03'],
                ['01','10','03','12'],
                ['02','03','20','21'],
                ['03','12','21','30']])

# Lookup table for tesseral subtraction
sub = np.array([['00','01','02','03'],
                ['11','00','13','02'],
                ['22','23','00','01'],
                ['33','22','11','00']])

class TessNode:

    # Should move some stuff (like isAdj?) from TessTools to TessNode?
    # Could redefine equality to check for same addrs and same occ + goals?

    """ Contains a Tesseral address and a dictionary of values associated with
    it.  The dict ('marks')  
    """
    def __init__(self, addr='0', marks=None):
        assert TessTools.isTess(addr)
        self.addr = addr
        self.pri = ['occ','goals','loc','chg','path','hood'] # marks by priority
        self.marks = marks
        if marks == None:
            self.marks = dict(zip(self.pri, [0 for i in range(len(self.pri))]))
            self.marks['goals'] = set()

    def __str__(self):
        return self.addr + '|' + self.mark()

    def __len__(self):
        return len(self.addr)

    def mark(self):
        """ Returns highest-priority mark with non-zero value. """
        for m in self.pri:
            if  m != 'goals' and self.marks[m] > 0:
                return m
            elif m == 'goals' and len(self.marks[m]) > 0:
                return m
        return 'free'

    def setMark(self, m): 
        """ Set mark m to 'true' if m recognized as mark, else add as goal. """
        if m in self.pri and m != 'goals':
            self.marks[m] = 1
        else: # add to goal list
            if m[0] == 'g' and m[1:].isdigit():
                self.marks['goals'].add(m)
            else:
                "GOAL '"+m+"' NOT ADDED. Expects  sequential 'g#' format, starting with # == 1."

    def isAncestor(self, n):
        """ Check if self is ancestor of n """
        return len(self) < len(n) and self.addr == n.addr[:len(self)]

    def isSibling(self, n):
        """ Check if self is sibling of n """
        return self.addr[:-1] == n.addr[:-1] and self.addr != n.addr

    def siblings(self):
        """ Return list of 3 sibling objects """
        sibaddrs = TessTools.expand(self.addr[:-1]+'*')
        return [TessNode(a) for a in sibaddrs if a != self.addr]

    def breed(self):
        """ Return list of children of this TessNode (with same marks) """
        kidaddrs = TessTools.expand(self.addr+'*')
        return [TessNode(a,dict(self.marks)) for a in kidaddrs]

    def mergeable(self, n):
        """ Check to see if TessNode n could be merged with this one """
        if self.addr[:-1] == n.addr[:-1]:
            if self.marks['occ'] == n.marks['occ']:
                if self.marks['goals'] == n.marks['goals']:
                    # other marks don't matter, will 'infect'
                    return True
        return False

    def merge(self, n):
        """ 'Merge' n to self: Take union of current marks and n's marks. """
        d = n.marks
        for k,v in d.items():
            if k in self.pri:
                self.marks[k] = self.marks[k] | v

    def apply(self, n, ignore=[]):
        """ 'Apply' n to self: overwrite self with n's marks """
        for m in self.pri:
            if m not in ignore:
                self.marks[m] = n.marks[m]

    def clear(self):
        """ Clears/cleans the node, preserving goals """
        d = TessNode().marks
        d['goals'] = self.marks['goals']
        self.marks = d

class TessTools:
    """ A class for working with Tesseral addresses """
    def __init__(self): pass
    
    @staticmethod
    def isTess(addr):
        """ Return true if the (list of) address(es) are valid quaternary """
        chars = [c for a in TessTools.toList(addr) for s in a for c in s]
        try: quaternary = [0 <= int(num) < 4 for num in chars]
        except: return False
        return not False in quaternary
    
    @staticmethod
    def toList(addr):
        if type(addr) != type(list):
            return [addr]
        return addr
    
    @staticmethod
    def expand(addr):
        """ 'Expand' the address/address list (i.e. fill in '*'s) """
        addr = TessTools.toList(addr)
        while '*' in [c for a in addr for s in a for c in s]:
            a = addr.pop(0)
            if '*' in a:
                ind = len(a)-1-a[::-1].find('*')
                for k in range(4):
                    addr.append(a[:ind] + str(k) + a[ind+1:])
            else: addr.append(a)
        return addr

    @staticmethod
    def adj(addr, diag=False):
        """ Return neighboring addresses of addr (4-connected) """
        adjacent = set()
        
	# prepend a 0 so that we can determine if any addrs are invalid
        adjacent.add(TessTools.tessMath('0'+addr,'+','1')) # right
        adjacent.add(TessTools.tessMath('0'+addr,'-','1')) # left
        adjacent.add(TessTools.tessMath('0'+addr,'+','2')) # up
        adjacent.add(TessTools.tessMath('0'+addr,'-','2')) # down
        if diag:
            adjacent.add(TessTools.tessMath('0'+addr,'+','3')) #ur
            adjacent.add(TessTools.tessMath('0'+addr,'-','3')) #dl
            adjacent.add(TessTools.tessMath(TessTools.tessMath('0'+addr,'+','1'),'-','2')) # dr
            adjacent.add(TessTools.tessMath(TessTools.tessMath('0'+addr,'-','1'),'+','2')) # ul

	# remove added 0, trimming invalid addrs
        return [a[1:] for a in adjacent if a[0] == '0']

    @staticmethod
    def isAdj(addr1, addr2):
        """ Return true if addr1 is adjacent to addr2 """
        s,l = sorted([addr1,addr2],key=len)
        adjacent = TessTools.adj(l) # user 'more refined' address
        for a in adjacent:
            #if TessTools.isTess(a) and TessTools.subset(sub=a,sup=s):
            if TessTools.subset(sub=a,sup=s):
                return True
        return False

    @staticmethod
    def subset(sub='', sup=''): 
        """ Return true if address sub is a 'member' of (or equal to) sup """
        return len(sub) >= len(sup) and sup in sub[:len(sup)]
    
    @staticmethod
    def bound(addr,size):
        """ Returns bounding points (bl,tr) of addr node in tree with size """
        # note: very similar to tesstree's getAddr...
        # should this be moved to tessnode?
        cells = np.array([(0,0),(1,0),(0,1),(1,1)])
        cx, cy = 0.0, 0.0
        for c in addr:
            size /= 2.0
            dx,dy = cells[int(c)]
            cx,cy = cx+size*dx, cy+size*dy
        return (cx,cy),(cx+size,cy+size)
        
    @staticmethod
    def tessMath(b,op,a):
        """ Add or subtract with tesseral arithmetic. All args are strings """
        a = ('0'*(len(b)-len(a)) + a)[::-1]
        b = ('0'*(len(a)-len(b)) + b)[::-1]
        fn = add if op == '+' else sub
        carry,out = '0',''
        for i in range(len(a)):
            narry,result = fn[a[i],b[i]]
            carry,result = fn[carry,result]
            carry = str(int(carry)+int(narry))
            out = result+out
        if carry != '0': out = carry+out     
        return out

if __name__ == '__main__':
    print TessTools.tessMath('0222','+','1')
    #print TessTools.expand('*2*')
    #print TessTools.adj('1')
    #print TessTools.subset('02033','23')
    #print TessTools.isTess('0132223')
    #print TessTools.isAdj('2','022')
    #print TessTools.bound('23', 100)

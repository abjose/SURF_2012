from tesstree import TessTree
from tesseral import TessTools
from tulip.spec import GRSpec
import tulip.gr1cint
from tulip.grsim import grsim
import networkx as nx
import numpy as np
import time

# Prefix for variables. Make sure there are no numbers in pfix.
pfix = 'test'  

class TreeUtils:

    """ Contains utilities for extracting stuff from quadtrees, and for doing
    stuff with that stuff, y'know.
    """

    def __init__(self, TessTree):
        # should verify TessTree is of type TessTree
        self.tt = TessTree 
        self.aut = None
        self.state = None
        self.goal = None
        self.last_mode = 0

    def dumpGraph(self):
        """ Return adjacency graph (as a NetworkX graph) of tree """
        G = nx.Graph()
        for n in self.tt.tree:
            G.add_edges_from([(n,n2) for n2 in self.tt.getAdj(n.addr)])
        for n in self.tt.getMarked('occ'):
            G.remove_edges_from(G.edges(n))
        return G

    def graphToFormula(self, G, initAddr=None):
        """ Converts NetworkX graph to inputs for spec. """
        # Assumes graph of TessNodes. Clean this up!
        forms = {'sys_vars':[], 'sys_trans':[], 'sys_excl':[],
                'sys_init':[], 'sys_prog':[]}
        excl_full = ''

        for n in G.nodes():
            name = pfix+n.addr
            forms['sys_vars'].append(name)
            trans = name + ' -> (' + name + "'"
            for m in G.neighbors(n):
                trans += ' | ' + pfix + m.addr + "'"
            forms['sys_trans'].append(trans+')')
            
            excl = '(' + name + "'"
            for m in G.nodes():
                if m != n:
                    excl += ' & !' + pfix + m.addr + "'"
            excl_full += excl + ') | '
            
        forms['sys_excl'].append(excl_full[:-2]) # exclude last |
                
        goals = []
        for g in self.tt.goallist():
            goals.append('|'.join([pfix+m.addr for m in g]))
        #print "goal addresses: " + str(goals)
        
        if initAddr == None:
            # find loc node
            occ = self.tt.getMarked('loc')
            if occ == []:
                raise Exception("Couldn't find initial address")
            initAddr = occ[0].addr

        initial = '(' + pfix+initAddr
        for n in G.nodes():
            if n.addr != initAddr:
                initial += ' & !' + pfix+n.addr
        
        forms['sys_init'].append(initial+')')
        forms['sys_prog'] = goals
        return forms, initAddr

    def formulaToAut(self, f, makeDOT=True, checkspec=False):
        """ Use GRSpec to get an automaton based on formula f """
        #print 'Attempting to get spec.'
        spec = GRSpec(sys_vars=f['sys_vars'],
                      sys_init=f['sys_init'],
                      sys_prog=f['sys_prog'],
                      sys_safety=f['sys_trans']+f['sys_excl'])
        #print 'Got spec, attempting to synthesize.'
        if not checkspec or tulip.gr1cint.check_realizable(spec, verbose=1):
            if checkspec:
                print "Spec is realizable, synthesizing automaton..."
            aut = tulip.gr1cint.synthesize(spec)
            if aut == None: return None
            #print "Done synthesizing."
            # dot -> png ==> "dot -Tpng -O (name).dot"
            if makeDOT:
                aut.writeDotFileColor("test.dot", hideZeros=True) 
            return aut
        else:
            print "Spec not realizable in formulaToAut."
            return None

    def getTrans(self, bb1, bb2, verbose=0):
        """ Return the midpoint of smaller of shared edges, given two bbs """
        close = 0.01
        (x1a,y1a),(x2a,y2a) = bb1 # (lower left), (upper right)
        (x1b,y1b),(x2b,y2b) = bb2

        small,large = None,None
        if (abs(x1a-x2a)<abs(x1b-x2b)): small,large = bb1,bb2
        else: small,large = bb2,bb1

        # this is awful
        (llx,lly),(urx,ury) = ll,ur = small
        lr,ul = (urx,lly),(llx,ury)
        small_edges = [(ll,lr),(lr,ur),(ur,ul),(ul,ll)]
        (llx2,lly2),(urx2,ury2) = ll2,ur2 = large
        lr2,ul2 = (urx2,lly2),(llx2,ury2)
        large_edges = [(ll2,lr2),(lr2,ur2),(ul2,ur2),(ll2,ul2)]
        for (x1,y1),(x2,y2) in large_edges:
            for (x3,y3),(x4,y4) in small_edges:
                if abs(x1-x3)<close and abs(x1-x4)<close:
                    if y1<=y3<=y2 and y1<=y4<=y2:
                        return ((x3+x4)/2.,(y3+y4)/2.)
                if abs(y1-y3)<close and abs(y1-y4)<close:
                    if x1<=x3<=x2 and x1<=x4<=x2:
                        return ((x3+x4)/2.,(y3+y4)/2.)
        if verbose > 0:
            print "No center found in getTrans."
        return None

    def getNextGoal(self, aut, pos, close, n=5, forceAddr=None, verbose=0):
        """ Returns point to move towards and a future 'path' of addrs.
        aut is automaton, pos is position in quadtree frame. 
        Close should be a function that takes a distance over the quadtree
        and returns true if it's small enough to transition to new state.
        """
        if verbose > 0:
            print 'TRYING TO GET NEXT STATE'
        addr = self.tt.getAddr(pos[0],pos[1], exists=True)
        if aut is not self.aut:
            if verbose > 0:
                print 'THINGS ARE DIFFERENT'
                aut.writeDotFile("aut-snapshot-"+time.strftime("%Y%m%d-%H%M%S")+".dot",
                                 hideZeros=True)
            # aut was changed, might need to change addr
            self.aut = aut#.copy()
            addr = self.tt.getAddr(pos[0],pos[1]) # note exists=False
            addr = self.matchToExistingAddr(aut,addr)
            if verbose > 0:
                print 'found addr: ' + addr
            for node in aut.nodes_iter():
                for k in aut.node[node]['state'].keys():
                    if aut.node[node]['state'][k] != 0:
                        print k
            
            states = aut.findAllAutPartState({pfix+addr:1})
            if len(states) == 1:
                if verbose > 0:
                    print 'FOUND ONLY ONE MATCHING STATE'
                self.state = states[0]
            elif len(states) > 1:
                if verbose > 0:
                    print 'FOUND MULTIPLE MATCHING STATES, CHECKING MODE MATCHES'
                # try to get state with matching mode, otherwise choose any
                matchingstates = filter(lambda s: aut.node[s]["mode"] == self.last_mode, states)
                if matchingstates != []:
                    if verbose > 0:
                        print 'FOUND A MODE MATCH'
                    self.state = matchingstates[0]
                else:
                    if verbose > 0:
                        print 'NO MODE MATCHES FOUND'
                    self.state = states[0]
            else:
                # Nothing matches - shouldn't happen.
                import pdb; pdb.set_trace()
                raise Exception("[getNextGoal] There is no state, Neo.")
            self.last_mode = self.aut.node[self.state]["mode"]

        # attempt to get next state in aut based on current state
        nextstate = aut.findNextAutState(self.state)
        if nextstate == -1:
            raise Exception("[getNextGoal] There is no next state.")
        nextaddr = self.extractAddr(nextstate)

        # get point, and check to see if close - if so, transition
        bb1 = TessTools.bound(addr,self.tt.s)
        if forceAddr != None:
            bb1 = TessTools.bound(forceAddr,self.tt.s)
        bb2 = TessTools.bound(nextaddr,self.tt.s)
        pt = self.getTrans(bb1,bb2)
        if pt == None:
            pt = self.goal
        self.goal = pt
        diff = [pt[0]-pos[1],pt[1]-pos[0]]  #hmm...
        if close(np.linalg.norm(diff)):
            self.state = nextstate
            self.last_mode = self.aut.node[self.state]["mode"]
            return self.getNextGoal(aut, pos, close, n=n, forceAddr=nextaddr)
         
        # and get addr path
        path = []
        tempstate = self.state
        for i in range(n):
            tempstate = aut.findNextAutState(tempstate)
            path.append(self.extractAddr(tempstate))
        return pt, path

    def extractAddr(self, state):
        """ Given a state (dictionary of vars:vals), finds and returns 
        address of 'true' variable (in this case, occupied cell in quadtree
        """
        #print self.aut.node[state]["state"]
        addr = [k for k,v in self.aut.node[state]["state"].iteritems() if v == 1][0]
        return addr[len(pfix):]

    def matchToExistingAddr(self, aut, addr):
        """ Given an address and an automaton, finds the address in the 
        automaton best matching (i.e smallest and containing) the given addr.
        NOTE: Assumes that passed addr is as refined as possible in the qtree
        (i.e. only needs to look for 'larger' cells).
        """
        bestAddr = addr
        addr_list = []
        for node in aut.nodes_iter():
            addr_list.extend([k for (k,v) in aut.node[node]['state'].items() if v != 0])
        #for a in [a[len(pfix):] for a in aut.node[0]["state"].keys()]:
        for a in [b[len(pfix):] for b in addr_list]:
            if TessTools.subset(sub=addr,sup=a) and len(a) < len(bestAddr):
                bestAddr = a
        return bestAddr

    def checkdiff(self, diff, aut=None):
        """ Return dict of addrs in treediff that are 'relevant' to current path
        over quadtree (i.e. affect aut and merit resynthesis).
        """
        if aut == None:
            if self.aut != None: 
                aut = self.aut
            else:
                print 'NO AUT IN CHECKDIFF!'
                return {}
        return dict([(k,v) for k,v in diff.items()
                     if aut.findAllAutPartState({pfix+k:1}) != []])

    def getHoods(self, diff, r=1):
        """ GET DEM HOODS REAL QUICK
        """
        hoods = []
        
        # get all possible, then make a set, then getMatches
        for addr in [a for new in diff.values() for a in new]:
            #if addr in [c2 for c1 in hoods for c2 in c1]:
            #    continue

            near = []
            if r <= 1:
                near = TessTools.adj(addr, diag=True)
                near.append(addr)
            elif r > len(addr)+1:
                near = TessTools.adj(addr[0], diag=True)
                near.append(addr[0])
            else:
                near = TessTools.adj(addr[:1-r], diag=True)
                near.append(addr[:1-r])

            hood = []
            for addr in near:
                hood.extend(self.tt.getMatches(addr, either=True))
            
            hood = set([h.addr for h in hood])

            for h in hoods:
                if not h.isdisjoint(hood):
                    h |= hood
                    hood = None
                    break
            if hood != None:
                hoods.append(hood)

        return hoods


    def tryPatch(self, spec, aut, diff, r_min=1, r_max=2, verbose=0):
        """ Passed a new spec, old aut, and a diff from old tree to current one
        """
        # extend sys_vars with new cells
        new_vars = [pfix+a for new in diff.values() for a in new]
        old_vars = [pfix+a for a in list(set(diff.keys()))]
        print old_vars, " -> ", new_vars
        spec.sys_vars.extend(new_vars)
        
        # modify sys_safety so new cells won't be visited
        spec.sys_safety.extend(["!"+nv+"'" for nv in new_vars])
        spec.sys_safety.extend(["!"+nv for nv in new_vars])

        # update aut's nodes to represent new cells
        for n in aut.nodes_iter():
            aut.node[n]['state'].update([(k,0) for k in new_vars])

        # make base state. Make sure not to put before aut updates.
        base_state = dict([(k,0) for k in aut.node[0]["state"].keys()])

        # block old cells
        block = []
        for old in diff.keys():
            tempstate = base_state.copy()
            tempstate[pfix+old] = 1
            block.append(('blocksys', [tempstate])) # can put all in one list?

        # relax new cells to old's neighbors (both directions)
        relax = []
        for new in diff.values():
            for addr in new:
                if self.tt.getMatches(addr, exact=True)[0].mark() == 'occ':
                    continue
                if verbose > 0:
                    print "LINKING", addr, 'TO:'
                s1 = base_state.copy()
                s1[pfix+addr] = 1
                for adj in self.tt.getAdj(addr):
                    if adj.mark() != 'occ':
                        s2 = base_state.copy()
                        s2[pfix+adj.addr] = 1
                        if verbose > 0:
                            print adj.addr
                        relax.append(('relax', [s1,s2]))
                        relax.append(('relax', [s2,s1]))

        # make PHANNNTOOM NOOOODES if init cells changed
        # need to see if loc is in neighbors
        # if so, for all modes, need to add phantom
        init_state = base_state.copy()
        occ = self.tt.getMarked('loc')
        if occ == []:
            raise Exception("Couldn't find initial address")
        initAddr = occ[0].addr
        initAddr = self.matchToExistingAddr(aut,initAddr)
        init_state[pfix+initAddr] = 1
        phantom_state = dict([(k,1) for k in base_state.keys()])


        num_modes = 0
        for n in aut.nodes_iter():
            if aut.node[n]["mode"] > num_modes-1:
                num_modes = aut.node[n]["mode"]+1
        if verbose > 0:
            print "Found "+str(num_modes)+" goal modes in automaton."

        for r in range(r_min, r_max+1):
            N = []
            hoods = self.getHoods(diff, r=r)
            print hoods
            for hood in hoods:
                for addr in hood:
                    s = base_state.copy()
                    s[pfix+addr] = 1
                    N.append(s)
            # add 'old' nodes
            for addr in diff.keys():
                s = base_state.copy()
                s[pfix+addr] = 1
                N.append(s)

            #print 'initAddr: ' + initAddr
            #print 'diff: ' + str(diff)
            #print 'neighbors: ' + str([a for hood in hoods for a in hood])

            max_n = max(aut.nodes())
            if initAddr in [a for hood in hoods for a in hood]:
                print "Found init in changed hood" 
                states = aut.findAllAutPartState({pfix+initAddr:1})
                if len(states) == 0:
                    for m in range(num_modes):
                        s = base_state.copy()
                        s[pfix+initAddr] = 1
                        max_n += 1
                        aut.add_node(max_n, state=s, mode=m, rgrad=100)
                        states.append(max_n)
                print 'states: ' + str(states)
                # check if care about mode m
                for m in range(num_modes):
                    matches = filter(lambda s: aut.node[s]['mode'] == m, states)
                    if matches != []:
                        print "Inserting a phantom node"
                        for match in matches:
                            max_n += 1
                            aut.add_node(max_n, state=phantom_state.copy(), mode=m, rgrad=-1)
                            aut.add_edge(max_n, match)

            patched_aut = tulip.gr1cint.patch_localfixpoint(spec, aut, N, block+relax, verbose)
            if patched_aut is not None:
                break
            if verbose > 0:
                print "tryPatch: radius "+str(r)+" failed."

        if patched_aut is not None:
            phantom_n = []
            for n in patched_aut.nodes_iter():
                if patched_aut.node[n]["state"] == phantom_state:
                    phantom_n.append(n)
                for ov in old_vars:
                    del patched_aut.node[n]['state'][ov]
            while len(phantom_n) > 0:
                pn = phantom_n.pop()
                max_n=max(patched_aut.nodes())
                patched_aut.remove_node(pn)
                if max_n != pn:
                    patched_aut.add_node(pn, state=patched_aut.node[max_n]["state"].copy(),
                                         mode=patched_aut.node[max_n]["mode"],
                                         rgrad=patched_aut.node[max_n]["rgrad"])
                    patched_aut.add_edges_from([(e_in,pn) for e_in in patched_aut.predecessors(max_n)])
                    patched_aut.add_edges_from([(pn,e_out) for e_out in patched_aut.successors(max_n)])
                    patched_aut.remove_node(max_n)
                    if max_n in phantom_n:
                        phantom_n.remove(max_n)
                        phantom_n.append(pn)
        return patched_aut
                

if __name__ == '__main__':
    t = TessTree(maxdepth = 4)
    u = TreeUtils(t)
    print t.treeDiff()
    for i in range(1,50,4):
    #for i in range(1):
        t.insert(5+i,51)
    diff = t.treeDiff()
    #print
    import cProfile
    #cProfile.run('u.getHoods(diff, r=3)')
    print diff
    print
    #cProfile.run('u.oldgetHoods(diff, r=1)')
    print
    cProfile.run('u.getHoods(diff, r=3)')
    print
    print u.getHoods(diff, r=3)
    #print u.oldgetHoods(diff, r=1)
    #print
    #print u.getHoods(diff, r=1)


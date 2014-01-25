#!/usr/bin/env python
import roslib; roslib.load_manifest('slamsynth_ros')
import rospy
import sys
import time
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from tesstree import TessTree
from occgrid import OccGrid
from display import Display
from treeutils import TreeUtils
from trackem_ros.msg import MTCalPoints
from landroid_murraylab.msg import ldr_tracks
from tulip.spec import GRSpec

class BotInterface:
    def __init__(self, passive=False):
        self.ranges = []
        self.lstart = 0
        self.lend = 0
        self.lstep = 0
        self.lmax = 0
        self.lmin = 0
        self.pose = None #x,y,heading
        self.pts = None
        self.count = 0
        rospy.init_node('bot_int', anonymous=True)
        pubbed_topics = rospy.get_published_topics()
        # Prefer forwarded topics, if available.
        if "/hforward/scan" in [tinfo[0] for tinfo in pubbed_topics]:
            self.laser_topicname = "/hforward/scan"
        else:
            self.laser_topicname = "/hokuyod_client_node/scan"
        if passive:
            self.mtr_cont = None
            # N.B., it suffices to set mtr_cont to None because any
            # attempt to use mtr_cont as a Publisher object should
            # fail in the form of an uncaught exception, i.e., it is
            # incorrect to instantiate BotInterface in "passive mode"
            # and then try to drive somewhere.
        else:
            self.mtr_cont = rospy.Publisher('/track_input', ldr_tracks)
        rospy.sleep(.1)  # apparently need to wait for >0.05 'seconds'

    def readLaser(self,data, dump=False):
        self.ranges = list(data.ranges)
        self.lstart = data.angle_min
        self.lend = data.angle_max
        self.lstep = data.angle_increment
        self.lmax = data.range_max
        self.lmin = data.range_min
        if dump:
            # write to a file, for debugging
            f = open('scandump' + str(self.count) + '.dmp', 'w+')
            for r in list(data.ranges):
                f.write("%s\n" % r)
            f.close()
            self.count += 1

    def readPose(self,data):
        pts = []
        for i in range(len(data.points)):
            pts.append((data.points[i].x,data.points[i].y))
        self.pts = pts
        self.pose = self.poseFromLEDs(self.pts)

    def updateLaser(self):
        try: 
            data = rospy.wait_for_message(self.laser_topicname, LaserScan, timeout=1.)
            if data: 
                self.readLaser(data)
        except Exception,e:
            print e

    def updatePose(self):
        try:
            self.pose = None
            while not self.pose:
                data=rospy.wait_for_message("/trackem/calpoints",MTCalPoints,timeout=.1)
                if data: self.readPose(data)
        except Exception,e:
            print e

    def setMotors(self, l,r):
        try: self.mtr_cont.publish(ldr_tracks(left=l,right=r))
        except Exception,e: print e

    def stop(self):
        self.setMotors(0,0)
    def setLin(self,l):
        self.setMotors(l,l)
    def setRot(self,r):
        self.setMotors(-r,r)
    
    def turnToPoint(self, (x2,y2), timeout=10):
        # consider cleaning up
        maxErr = 0.15 # radians
        speed = 25
        
        #self.stop()
        self.updatePose()
        x1,y1,theta = self.pose
        alpha = math.atan2(y2-y1,x2-x1)
        alpha -= theta
        if alpha > math.pi: alpha -= 2*math.pi
        elif alpha < -math.pi: alpha += 2*math.pi 
        
        st = time.time()
        while abs(alpha) > maxErr and time.time()-st < timeout:
            direction = math.copysign(1,alpha)
            self.setRot(speed*direction)
            self.updatePose()
            #print self.pts
            #print self.pose
            if not self.pose: continue
            x1,y1,theta = self.pose
            alpha = math.atan2(y2-y1,x2-x1) - theta
            if alpha > math.pi: alpha -= 2*math.pi
            elif alpha < -math.pi: alpha += 2*math.pi
            rospy.sleep(.25) # sleeping some seems to make for fewer mux errors
        if time.time()-st < timeout:
            return False
        self.setRot(0)
        #print "Done turning"
        return True
    
    def goToPoint(self, (x2,y2)):
        # consider cleaning up
        maxErr = 0.15 # meters
        speed = 25
        got_to_point = False
                
        #while not got_to_point:# and failcount < 20:
        if not self.turnToPoint((x2,y2)):
            return False
        #self.setLin(speed)
        runcount = 0
        while runcount < 10:
            runcount += 1
            self.setLin(speed)
            self.updatePose()
            x1,y1,theta = self.pose
            distErr = math.sqrt((x2-x1)**2+(y2-y1)**2)
            #print distErr
            if distErr < maxErr:
                got_to_point = True
                self.stop()
                return True#break
            rospy.sleep(.25)
        #self.stop()
        return False
        #print "Got to point: " + str((x2,y2))

    def dist(self,p1,p2):
        # SHOULD REPLACE CALLS TO THIS WITH NP.LINALG.NORM, THEN DELETE THIS
        x1,y1 = p1
        x2,y2 = p2
        return math.sqrt((x2-x1)**2+(y2 - y1)**2)    

    def poseFromLEDs(self, ledpts):
        # consider cleaning up...could lift from scott's demo.py code
        """ Get bot heading from row of pts """
        if len(ledpts) == 3:
            a,b,c = ledpts[0], ledpts[1], ledpts[2]
            # these are terrible names.
            # SHOULD REPLACE WITH NP.LINALG.NORM
            dl = [self.dist(a,b), self.dist(b,c), self.dist(a,c)]
            pl = [(a,b), (b,c), (a,c)]
            bl = sorted(zip(dl,pl),key=lambda x: x[0])
            # find front and back
            bx,by = [p for p in bl[0][1] if p in bl[1][1]][0] # intersect
            fx,fy = [p for p in bl[1][1] if p != (bx,by)][0]
            # calculate things
            h = math.atan2(fy-by,fx-bx)
            x = (bx+fx)/2 # not float...
            y = (by+fy)/2
            return x,y,h
        else:
            print "Got " + str(len(ledpts)) + " instead of 3."
            return None
        

if __name__ == '__main__':
    if len(sys.argv) > 1 and "-p" in sys.argv:
        passive_mode = True  # No actuation, but print next goal at terminal.
        print "Starting in passive mode (no robot motion)"
    else:
        passive_mode = False
    
    bi = BotInterface(passive=passive_mode)
    d = Display([0,3,0,3])
    bi.updateLaser()
    # consider changing to take a list of laser values instead of all these?
    lattr = [bi.lstart,bi.lstep,bi.lmin,bi.lmax]
    occ = OccGrid((0,0),3,.05,bi.lstart,bi.lstep,bi.lmin,bi.lmax,forget=False)
    t = TessTree(size=len(occ.grid),maxdepth=4) #good with maxdepth 3 or 4 
    tutil = TreeUtils(t)

    g2 = t.worldToQuad((2.9,2.9),(0,3))
    g1 = t.worldToQuad((.1,.1),(0,3))
    #g3 = t.worldToQuad((2.9,.1),(0,3))
    t.insertRegion(g1, 4, 'g1')
    t.insertRegion(g2, 4, 'g2')
    #t.insertRegion(g3, 2, 'g3')

    closeThresh = np.linalg.norm(t.worldToQuad((0,0.2),(0,3)))
    qclose = lambda d: d < closeThresh
    aut = None
    verbose = 0
    startiter = 5
    spec = None
    r_min = 1
    r_max = 4
    i = 0
    while not rospy.is_shutdown():
        i += 1
        
        bi.updateLaser()
        bi.updatePose()
        if not bi.pose: 
            if verbose > 0:
                print 'NO POSE, CONTINUING'
            continue
        x,y,h = bi.pose
        occ.setPose(x,y,h)
        if verbose > 0:
            print 'starting insertion'
        occ.insertScan(bi.ranges) 
        t.clear()
        t.insertOG(occ.grid, occ.occupied)

        # set location
        wx,wy = t.worldToQuad((x,y),(0,3))
        loc = t.getMatches(t.getAddr(wx,wy,exists=True),exact=True)[0]
        loc.setMark('loc')

        diff = t.treeDiff()
        if len([v for v in diff.values() if len(v) == 1]) > 0:
            f,init = tutil.graphToFormula(tutil.dumpGraph())
            spec = GRSpec(sys_vars=f['sys_vars'],
                          sys_init=f['sys_init'],
                          sys_prog=f['sys_prog'],
                          sys_safety=f['sys_trans']+f['sys_excl'])
            aut = tutil.formulaToAut(f, makeDOT=False)
            print "Initialized automaton; size is "+str(len(aut))
        elif len(diff) > 0:
            if aut is not None:
                hoods = tutil.getHoods(tutil.checkdiff(diff), r=r_min)
                if verbose > 0:
                    print 'diff:'
                    print diff
                    print 'checkdiff:'
                    print tutil.checkdiff(diff, aut)                
                    print 'hoods:'
                    print hoods
                for hood in hoods:
                    for addr in hood:
                        t.getMatches(addr,exact=True)[0].setMark('hood')
                for k,v in diff.items():
                    for addr in v:
                        t.getMatches(addr,exact=True)[0].setMark('chg')
                print 'force redraw'
                d.quad(t)
                d.pose2d([x,y,h])
                d.blit()
                print 'trying patch'
                aut.writeDotFile("pre.dot", hideZeros=True)
                patched_aut = tutil.tryPatch(spec, aut, diff,
                                             r_min=r_min, r_max=r_max, verbose=1)
                if patched_aut is None:
                    raise ValueError("patching failed.")
                else:
                    if aut == patched_aut:
                        print "="*20 + "    NO CHANGE"
                    aut = patched_aut
                    aut.writeDotFile("post.dot", hideZeros=True)
                    print "Patching success!  New size is "+str(len(aut))
                    f,init = tutil.graphToFormula(tutil.dumpGraph())
                    spec = GRSpec(sys_vars=f['sys_vars'],
                                  sys_init=f['sys_init'],
                                  sys_prog=f['sys_prog'],
                                  sys_safety=f['sys_trans']+f['sys_excl'])

        #if False:
        if i > startiter:
            #if (i==startiter+1 or i%6==0) and (aut is None):
            if i == startiter+1 and (aut is None):
                f,init = tutil.graphToFormula(tutil.dumpGraph())
                spec = GRSpec(sys_vars=f['sys_vars'],
                              sys_init=f['sys_init'],
                              sys_prog=f['sys_prog'],
                              sys_safety=f['sys_trans']+f['sys_excl'])
                aut = tutil.formulaToAut(f, makeDOT=False)
                print "Initialized automaton; size is "+str(len(aut))
            if aut == None:
                if verbose > 0:
                    print 'NO AUT, CONTINUING'
                continue
            
            (px,py),path = tutil.getNextGoal(aut, (wx,wy), qclose, n=5, verbose=1)
            goal = t.quadToWorld((py,px),(0,3))
            if verbose > 0:
                print "Goal position is " + str(goal)

            if verbose > 0:
                print 'DRAWING PATH:', path
            for addr in path:
                matches = t.getMatches(addr,exact=True)
                if matches: matches[0].setMark('path')

            d.quad(t)
            d.pose2d([x,y,h])
            d.blit()
            if not passive_mode:
                bi.goToPoint(goal)
                
        # calling quad more than necessary for sake of prettiness...
        d.quad(t)
        xl,yl = [],[]
        xl,yl = occ.getScatterVectors()
        if xl != []:
            d.clear()
            d.scatter(xl,yl)
        d.pose2d([x,y,h])
        d.blit()

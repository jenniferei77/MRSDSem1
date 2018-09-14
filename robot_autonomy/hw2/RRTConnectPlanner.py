import numpy as np, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
	
	self.planning_env.SetGoalParameters(goal_config)
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        fplan = []
        rplan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        fplan.append(start_config)
        rplan.append(goal_config)
        
        dist_to_goal = 5
        radius = 0.5
        
        
        while dist_to_goal > radius:
          
            qR = self.planning_env.GenerateRandomConfiguration()
            print "qR", qR
            fsid, fposition = ftree.GetNearestVertex(qR)    
            rsid, rposition = rtree.GetNearestVertex(qR)
            fdist2rand = np.linalg.norm(abs(qR-fposition))
            rdist2rand = np.linalg.norm(abs(qR-rposition))
            if fdist2rand > rdist2rand:
                newVertex = self.planning_env.Extend(rposition, qR)
                if newVertex is not None:
                    reid = rtree.AddVertex(newVertex)
                      #pdb.set_trace()
                    rtree.AddEdge(rsid,reid)   
                    self.planning_env.PlotEdge(rtree.vertices[rsid], rtree.vertices[reid])
                    dist_to_goal = fdist2rand
                    if dist_to_goal <= radius:
                        last = 1
                
            else:
                newVertex = self.planning_env.Extend(fposition, qR)
                if newVertex is not None:
                    feid = ftree.AddVertex(newVertex)
                      #pdb.set_trace()
                    ftree.AddEdge(fsid,feid)              
                    self.planning_env.PlotEdge(ftree.vertices[fsid], ftree.vertices[feid])
                    dist_to_goal = rdist2rand
                    if dist_to_goal <= radius:
                        last = 0
            
          
            print "distance", dist_to_goal  
        
        fcurr_vert = ftree.vertices[-1]  
        fcurr_id = len(ftree.vertices) - 2
        finter_plan = []
        finter_plan.append(ftree.vertices[-1])
        
        while fcurr_vert is not start_config:
            fprev_id = ftree.edges[fcurr_id]
            fprev_vert = ftree.vertices[fprev_id]
            finter_plan.append(fprev_vert)
            fcurr_vert = fprev_vert
            fcurr_id = fprev_id
        
        plan = finter_plan[::-1]
        
        rcurr_vert = rtree.vertices[-1]
        rcurr_id = len(rtree.vertices) - 2
        print "rcurr_id: ", rcurr_id, "\n"
        rinter_plan = []
        rinter_plan.append(rtree.vertices[-1])
        
        while rcurr_id != 0:
            print "rtree.edges: ", rtree.edges, "\n"
            rprev_id = rtree.edges[rcurr_id]
            print "rprev", rprev_id, "\n"
            rprev_vert = rtree.vertices[rcurr_id]
            print "rprev_vert", rprev_vert, "\n"
            rinter_plan.append(rprev_vert)
            rcurr_vert = rprev_vert
            rcurr_id = rprev_id
            
        plan += rinter_plan    
        
        
        return plan





import numpy as np
from RRTTree import RRTTree
import pdb

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan.append(start_config)
        dist_to_goal = np.linalg.norm(goal_config - start_config)
        radius = 0.5
        
        
        while dist_to_goal > radius:
            qR = self.planning_env.GenerateRandomConfiguration()
            print "qR", qR
            sid, position = tree.GetNearestVertex(qR)    
            newVertex = self.planning_env.Extend(position, qR)
            
            if newVertex is not None:
                eid = tree.AddVertex(newVertex)
            	#pdb.set_trace()
                tree.AddEdge(sid,eid)              
                self.planning_env.PlotEdge(tree.vertices[sid], tree.vertices[eid])
            	dist_to_goal = np.linalg.norm(newVertex-goal_config)  
                print "distance", dist_to_goal  
        
        curr_vert = tree.vertices[-1]
        curr_id = len(tree.vertices) - 1
        inter_plan = []
        inter_plan.append(tree.vertices[-1])
        while curr_vert is not start_config:
            prev_id = tree.edges[curr_id]
            prev_vert = tree.vertices[prev_id]
            inter_plan.append(prev_vert) 
            curr_vert = prev_vert
            curr_id = prev_id
        
        plan = inter_plan[::-1]
        plan.append(goal_config)
        
        return plan


    #breadth first search
    def bfs_paths(graph, start, goal):
        queue = [(start, [start])]
        while queue:
            (vertex, path) = queue.pop(0)
            for next in graph[vertex] - set(path):
                if next == goal:
                    yield path + [next]
                else:
                    queue.append((next, path + [next]))




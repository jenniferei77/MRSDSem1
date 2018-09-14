from Queue import * 
from collections import deque
import numpy as np
import pdb

class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def SearchBFS(self, start_node, goal_node):


    # start config is the root node I guess 
        nodes = deque()
        size =1
        for idx in range(self.planning_env.discrete_env.dimension):
            size = size*self.planning_env.discrete_env.num_cells[idx]
        visited_nodes = [0] * int(size)
        path = dict() 
    
        root = start_node
        path[root] = (None)
        nodes.append(root)
        visited_nodes[0] = 1


        while nodes:
            #print("start", start_node)
            #print("goal", goal_node)
            subtree = nodes.popleft()
            print("subtree: ", subtree) 
            #print(subtree)

            if subtree is goal_node:
                return path

            values = self.planning_env.GetSuccessors(subtree)
            #print("children", values)
            #pdb.set_trace()
            for child in self.planning_env.GetSuccessors(subtree):

                if visited_nodes[child] is 1:
                    continue

                if child not in nodes:
                    nodes.append(child)
                    path[child] = subtree

                    new = self.planning_env.discrete_env.NodeIdToConfiguration(child)
                    old = self.planning_env.discrete_env.NodeIdToConfiguration(subtree)
                    self.planning_env.PlotEdge(new, old)
                    
                    #print("nodes", nodes)
            visited_nodes[subtree] = 1
            
                    #pdb.set_trace()
    # now we make the path 
        print(path)
        print(goal_node)
        print("Reached the Goal")
        return path

        
    def Plan(self, start_config, goal_config):
        
        plan = []
        self.planning_env.InitializePlot(goal_config)

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        path = self.SearchBFS(start_node, goal_node)

        plan_node = [] 
        plan_node.append(goal_node)
        curr_node = goal_node


        while curr_node is not start_node:
            #print(path[curr_node])
            prev_node = path[curr_node]
            plan_node.append(prev_node)
            curr_node = prev_node
        plan_node = plan_node[::-1]
        print(plan_node)

        for i in plan_node:
            plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(i))
        return plan


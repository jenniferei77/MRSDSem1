import numpy as np
import pdb
import math
import random
import time
from Queue import * 
from collections import deque


class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Path(self, start_node, goal_node):
        
        nodes = deque()
        size =1
        for idx in range(self.planning_env.discrete_env.dimension):
            size = size*self.planning_env.discrete_env.num_cells[idx]
        #pdb.set_trace()
        visited_nodes = [0] * int(size)
        path = dict() 
    
        root = start_node
        path[root] = (None)
        nodes.append(root)
        visited_nodes[0] = 1

        while nodes:
            subtree = nodes.pop()
            values = self.planning_env.GetSuccessors(subtree)

            values[0],values[1] = values[1],values[0]
            # if len(values) == 4:
            #     values[2],values[3] = values[3],values[2]

            if subtree is goal_node:
                print("here1")
                return path

            for child in values:

                if visited_nodes[child] is 1:
                    if child == goal_node:
                        return path
                    continue
                if child not in nodes:
                    path[child] = subtree
                    nodes.append(child)
                    # new = self.planning_env.discrete_env.NodeIdToConfiguration(child)
                    # old = self.planning_env.discrete_env.NodeIdToConfiguration(subtree)
                    # self.planning_env.PlotEdge(new, old)
            visited_nodes[subtree] = 1

        print(path)
        print("child",child)
        return path


    # def dfs_paths(self, graphs, start, goal):
    #     stack = [(start, [start])]
    #     pdb.set_trace()
    #     while stack:
    #         (vertex, path) = stack.pop()
    #         for next in graphs[vertex]:
    #             if next == goal:
    #                 yield path + [next]
    #             else:
    #                 stack.append((next, path + [next]))


    def Plan(self, start_config, goal_config):
        
        plan = []
        #self.planning_env.InitializePlot(goal_config)

        start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        path = self.Path(start_node, goal_node)
        # paths = self.dfs_paths(path_new,goal_node,start_node)
        # paths = list(paths)
        plan_node = [] 
        plan_node.append(goal_node)
        curr_node = goal_node

        while curr_node != start_node:
            #print(path[curr_node])
            prev_node = path[curr_node]
            plan_node.append(prev_node)
            curr_node = prev_node
        plan_node = plan_node[::-1]
        print(plan_node)
        for i in plan_node:
            plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(i))
        return plan



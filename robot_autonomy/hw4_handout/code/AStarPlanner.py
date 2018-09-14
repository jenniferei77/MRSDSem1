import numpy
class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Astar(self, start_node, goal_node):
	# adapted from Wikipedia sudo code

        path = dict()
        path_actions = dict()
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration spac
        
        size =1
        for idx in range(self.planning_env.discrete_env.dimension):
            size = size*self.planning_env.discrete_env.num_cells[idx]
        #visited_nodes = [0] * int(size) #nodes that have been seen
        visited_nodes = set()
         
        open_list = set()
        open_list.add(start_node)
        curr_node = start_node

        #fcost = [float('inf')] * int(size)
        fcost = dict()
        fcost[start_node] = self.planning_env.ComputeHeuristicCost(curr_node,goal_node)

        #gcost = [float('inf')] * int(size)
        gcost = dict()
        gcost[curr_node] = 0

        path[curr_node] = (None)
        
        while len(open_list) is not 0:
        #while curr_node != goal_node:
            #print(len(open_list))
            best = float('inf')
            for k in open_list:
                
                if fcost[k] < best:
                    best = fcost[k]
                    curr_node = k

            distnow = self.planning_env.ComputeDistance(curr_node, goal_node)
           
            print "curr node", curr_node
            print "goal node", goal_node
            if curr_node == goal_node:
                return path, path_actions
            
            #print(curr_node)
            open_list.remove(curr_node)

            #visited_nodes[curr_node] = 1
            visited_nodes.add(curr_node)
            #closed_list.add(curr_node)
            
            
            options = self.planning_env.GetSuccessors(curr_node)
            print "options", len(options)

            curr_cost = self.planning_env.ComputeHeuristicCost(curr_node,goal_node)
            best_cost = curr_cost
            first = 1
            for i in options:
                #if visited_nodes[i] == 1:
                #print("Node in successor", i[0])
                if i[0] in visited_nodes:
                    continue
                if i[0] not in open_list:
                    open_list.add(i[0])
                
                opt_cost = self.planning_env.ComputeHeuristicCost(curr_node,i[0]) + gcost[curr_node]
                try: 
                    gcost[i[0]]
                    True
                except:
                    gcost[i[0]] = float('inf')

                if opt_cost >= gcost[i[0]]:
                    continue

                path[i[0]] = curr_node
                path_actions[i[0]] = i[1]
                #print(path)
                gcost[i[0]] = opt_cost
                fcost[i[0]] = gcost[i[0]] + self.planning_env.ComputeHeuristicCost(i[0],goal_node)
                #new = self.planning_env.discrete_env.NodeIdToConfiguration(i)
                #old = self.planning_env.discrete_env.NodeIdToConfiguration(curr_node)
                #self.planning_env.PlotEdge(new, old)
                #print "current node", curr_node
        
        return [0], [0]

    
    def Plan(self, start_config, goal_config):
        
        plan = []
        Aplan =[]
        #self.planning_env.InitializePlot(goal_config)

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        print "start node", start_node
        print "goal_node", goal_node
        path, path_actions = self.Astar(start_node, goal_node)
        #print "path actions", path_actions[goal_node]

        plan_node = [] 
        plan_actions = []

        plan_node.append(goal_node)
        #plan_actions.append(plan_actions[goal_node])
        curr_node = goal_node

        print "path", path
        print "pathactions", path_actions

        print "curr_node", curr_node
        print "start_node", start_node
        print "goal_node", goal_node

        while curr_node is not start_node:
            #print(path[curr_node])
            prev_node = path[curr_node]
            plan_node.append(prev_node)
            plan_actions.append(path_actions[curr_node])
            print "path_actions", path_actions[curr_node]
            curr_node = prev_node
        plan_node = plan_node[::-1]
        plan_actions = plan_actions[::-1]
        print(plan_node)



        #for i in plan_actions:
        #     plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(i))
        #     print "i", i 
        #     Aplan.append(self.planning_env.discrete_env.NodeIdToConfiguration(i))
        return plan_actions





        


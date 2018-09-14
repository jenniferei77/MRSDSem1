class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Astar(self, start_node, goal_node):

        path = dict()
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration spac
        
        size =1
        for idx in range(self.planning_env.discrete_env.dimension):
            size = size*self.planning_env.discrete_env.num_cells[idx]
        visited_nodes = [0] * int(size) #nodes that have been seen
         
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
        
        while len(open_list) != 0:
            #print(len(open_list))
            best = float('inf')
            for k in open_list:
                
                if fcost[k] < best:
                    best = fcost[k]
                    curr_node = k

            if curr_node == goal_node:
                return path
            
            #print(curr_node)
            open_list.remove(curr_node)

            visited_nodes[curr_node] = 1
            #closed_list.add(curr_node)
            
            
            options = self.planning_env.GetSuccessors(curr_node)
            curr_cost = self.planning_env.ComputeHeuristicCost(curr_node,goal_node)
            best_cost = curr_cost
            first = 1
            for i in options:
                if visited_nodes[i] == 1:
                    continue
                if i not in open_list:
                    open_list.add(i)
                
                opt_cost = self.planning_env.ComputeHeuristicCost(curr_node,i) + gcost[curr_node]
                try: 
                    gcost[i]
                    True
                except:
                    gcost[i] = float('inf')

                if opt_cost >= gcost[i]:
                    continue

                path[i] = curr_node
                print(path)
                gcost[i] = opt_cost
                fcost[i] = gcost[i] + self.planning_env.ComputeHeuristicCost(i,goal_node)
                new = self.planning_env.discrete_env.NodeIdToConfiguration(i)
                old = self.planning_env.discrete_env.NodeIdToConfiguration(curr_node)
                self.planning_env.PlotEdge(new, old)
        
        return failure

    
    def Plan(self, start_config, goal_config):
        
        plan = []
        self.planning_env.InitializePlot(goal_config)

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        path = self.Astar(start_node, goal_node)
        print(path)

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





        


import numpy 
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []
	
		#get 4 connected world 
        coord = self.discrete_env.NodeIdToGridCoord(node_id)

        for idx in range(self.discrete_env.dimension):
            temp = numpy.array(coord)
			
            if coord[idx]+1 < self.discrete_env.num_cells[idx]:
                temp[idx]= coord[idx]+1
                move = self.discrete_env.GridCoordToConfiguration(temp)

                Pose = numpy.array([[ 1, 0, 0, move[0]],
                                [ 0, 1,  0, move[1]],
                                [ 0, 0,  1, 0],
                                [ 0, 0,  0, 1]])
                self.robot.SetTransform(Pose)

                if self.robot.GetEnv().CheckCollision(self.robot) == False:
                    successors.append(self.discrete_env.GridCoordToNodeId(temp))
                    #print("Success1", self.discrete_env.GridCoordToNodeId(temp))
           

            if coord[idx]-1 > 0:
                temp2 = numpy.array(coord)
                temp2[idx]= coord[idx]-1
                move2 = self.discrete_env.GridCoordToConfiguration(temp2)
                #print("move2", move2)
                Pose2 = numpy.array([[ 1, 0, 0, move2[0]],
				            [ 0, 1,  0, move2[1]],
				            [ 0, 0,  1, 0],
				            [ 0, 0,  0, 1]])
                self.robot.SetTransform(Pose2)

                if self.robot.GetEnv().CheckCollision(self.robot) == False:
                    successors.append(self.discrete_env.GridCoordToNodeId(temp2))
                    #print("success2", self.discrete_env.GridCoordToNodeId(temp2))
		
	    
        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring

        #print("node", node_id)
        #print("success", successors)
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0
	
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        dist = numpy.linalg.norm(numpy.array(end_config)-numpy.array(start_config))

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id, goal_id)

        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

        


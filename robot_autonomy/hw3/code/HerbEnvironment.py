import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):

        successors = []

        config = [0] * len(self.robot.GetActiveDOFIndices())
        coord = self.discrete_env.NodeIdToGridCoord(node_id)

        for idx in range(len(self.lower_limits)):
            temp = numpy.array(coord)
            
            if coord[idx]+1 < self.discrete_env.num_cells[idx]:
                temp[idx]= coord[idx]+1
                move = self.discrete_env.GridCoordToConfiguration(temp)

                targetPose = numpy.array([move[0], move[1], move[2], move[3], move[4], move[5], move[6]])
                self.robot.SetActiveDOFValues(targetPose)

                if self.robot.GetEnv().CheckCollision(self.robot) == False:
                    successors.append(self.discrete_env.GridCoordToNodeId(temp))
                    #print("Success1", self.discrete_env.GridCoordToNodeId(temp))
           

            if coord[idx]-1 > 0:
                temp2 = numpy.array(coord)
                temp2[idx]= coord[idx]-1
                move2 = self.discrete_env.GridCoordToConfiguration(temp2)
                #print("move2", move2)
                targetPose = numpy.array([move2[0], move2[1], move2[2], move2[3], move2[4], move2[5], move2[6]])
                self.robot.SetActiveDOFValues(targetPose)

                if self.robot.GetEnv().CheckCollision(self.robot) == False:
                    successors.append(self.discrete_env.GridCoordToNodeId(temp2))

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        
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



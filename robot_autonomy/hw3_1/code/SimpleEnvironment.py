import numpy
import matplotlib.pyplot as pl
import pdb
import math

global sample

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0
	#self.sample = 0
	global sample
	sample = 0
	
        

    def SetGoalParameters(self, goal_config, p = 0.8):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
	global sample
        lower_limits, upper_limits = self.boundary_limits
        
        config = numpy.random.uniform(lower_limits, upper_limits, 2)
	
	
     	   
        targetPose = numpy.array([[ 1, 0, 0, config[0]],
                                  [ 0, 1,  0, config[1]],
                                  [ 0, 0,  1, 0],
                                  [ 0, 0,  0, 1]])
	
	
	if sample%100 == self.p:
		targetPose = numpy.array([[ 1, 0, 0, 2],
		                          [ 0, 1,  0, -0.8],
		                          [ 0, 0,  1, 0],
		                          [ 0, 0,  0, 1]])
	
	

        self.robot.SetTransform(targetPose)
        if self.robot.GetEnv().CheckCollision(self.robot):
            self.GenerateRandomConfiguration()
        
  	sample+=1
        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        #d = (start_config - end_config) ** 2
        #distance = math.sqrt(abs(d[0]+d[1]))
        distance = numpy.linalg.norm(end_config-start_config)

        return distance

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
        step = 0.1
        m = (end_config[1]-start_config[1])/(end_config[0]-start_config[0])
        b = start_config[1]-m*start_config[0]
        stepped = numpy.linspace(start_config[0]+step, end_config[0]-step, num=(abs(start_config[0]-end_config[0]))/step)
        #pdb.set_trace()
        x_prev= start_config[0]
        y_prev = start_config[1]
        
        for x in stepped:
            y = m*x+ b
            #pdb.set_trace()
            targetPose = numpy.array([[ 1, 0, 0, x],
                                      [ 0, 1,  0, y],
                                      [ 0, 0,  1, 0],
                                      [ 0, 0,  0, 1]])
            self.robot.SetTransform(targetPose)
            
            if self.robot.GetEnv().CheckCollision(self.robot):
                if x-step == start_config[0]:
                    return None
                else:
                    
                    return numpy.array([x_prev,y_prev])
            elif self.ComputeDistance(start_config, [x,y]) > 0.5:
                return numpy.array([x_prev,y_prev])
            else:
                y = m*(x-step) + b           
                #pdb.set_trace()
                if x == stepped[-1]:
                    return numpy.array([x,y]) 
            y_prev = y
            x_prev = x        


    def ShortenPath(self, path, timeout=5.0):

        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
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








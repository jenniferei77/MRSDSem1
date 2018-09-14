import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import pdb
from math import pi

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
        L=0.5
        r=0.2

              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):

            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = numpy.array(self.discrete_env.GridCoordToConfiguration(grid_coordinate))

            #get forward action?! So confused 
            #made up a dt 
            control_for = Control(1, 1, 2)
            footprint_for = self.GenerateFootprintFromControl(start_config, control_for)
            self.actions[idx].append(Action(control_for, footprint_for))

            #print(footprint_for)

            #get backward action
            control_back = Control(-1, -1, 2)
            footprint_back = self.GenerateFootprintFromControl(start_config, control_back)
            self.actions[idx].append(Action(control_back, footprint_back))

            #right 
            control_right = Control(1, -1, 1)
            footprint_right = self.GenerateFootprintFromControl(start_config, control_right)
            self.actions[idx].append(Action(control_right, footprint_right))

            #left 
            control_left = Control(-1, 1, 1)
            footprint_left = self.GenerateFootprintFromControl(start_config, control_left)
            self.actions[idx].append(Action(control_left, footprint_left))

            # print(control_left)
            # print("Yo\n")
            # print(footprint_for)


            
            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #
         
            

    def boundAngles(self, angle):
        if angle > pi:
            angle -= 2*pi
        if angle < -pi:
            angle += 2*pi

        return angle

    def GetSuccessors(self, node_id):

        successors = []

        L=0.5
        r=0.2
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
       # print "don't be negative", coord
        confignow = self.discrete_env.NodeIdToConfiguration(node_id)

        possible_succ = self.actions[coord[2]]
        for action in possible_succ:  #this will have 4 options
            collision = False
            for currentspot in action.footprint:
                #print("move2", move2)
                Pose = numpy.array([[1,0, 0, confignow[0]+currentspot[0]],
                            [ 0, 1,  0, confignow[1]+currentspot[1]],
                            [ 0, 0,  1, 0],
                            [ 0, 0,  0, 1]])
                # Pose = numpy.array([[numpy.cos(currentspot[2]), -numpy.sin(currentspot[2]), 0, currentspot[0]],
                #             [numpy.sin(currentspot[2]), numpy.cos(currentspot[2]),  0, currentspot[1]],
                #             [ 0, 0,  1, 0],
                #             [ 0, 0,  0, 1]])
                self.robot.SetTransform(Pose)

                if self.robot.GetEnv().CheckCollision(self.robot):
                    collision = True
            
            #pdb.set_trace()
            #print "currentspot", currentspot
            #print "confignow",  confignow

            final =[]
            for l in range(len(currentspot)-1):
                final.append(currentspot[l] + confignow[l])
            #print "final", final
            final.append(currentspot[2])
            inrange = True
            if collision == False:
                for k in range(len(self.discrete_env.upper_limits)-1):
                    #currentspot[k] = round(currentspot[k], 5)
                    #print("currentspotnow", currentspot[k])
                    if final[k] > (self.discrete_env.upper_limits[k]):
                        #print("I'm greater than")
                        #final[k] = self.discrete_env.upper_limits[k]
                        inrange = False
                    if final[k] < (self.discrete_env.lower_limits[k]):
                        #print("I'm less than")
                        inrange = False
                        #final[k] = self.discrete_env.lower_limits[k]

                if inrange == True:
                    #print("currentspot", currentspot)
                    final[2] = self.boundAngles(final[2])
                    #nodefinal = self.discrete_env.ConfigurationToNodeId(final)
                    nodefinal = self.discrete_env.ConfigurationToNodeId(final)
                    successors.append([nodefinal, action])


        

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0
    
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        ind = numpy.array([0,1])
        dist = numpy.linalg.norm(numpy.array(end_config[ind])-numpy.array(start_config[ind]))

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        cost = self.ComputeDistance(start_id, goal_id)

        return cost

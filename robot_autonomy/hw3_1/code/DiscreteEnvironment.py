import numpy as np
import pdb

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = np.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
    
        node_id = 0
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension
        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
    
        for idx in range(self.dimension):
           coord[idx] = np.floor(self.num_cells[idx]*(config[idx]-self.lower_limits[idx])/(self.upper_limits[idx]-self.lower_limits[idx]))

        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        coord=np.array(coord)
        cells = np.array(self.num_cells)
        #res = np.array(self.resolution)
    #cell = (self.upper_limits - self.lower_limits)/self.resolution
        for idx in range(self.dimension):
            config[idx] = ((coord[idx]/self.num_cells[idx])*(self.upper_limits[idx]-self.lower_limits[idx]))+ self.lower_limits[idx]
        #print("config",config)
        #config = [0, 0]
        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0

        coord = [int(p) for p in coord]
        cells = [int(k) for k in self.num_cells]

        node_id=np.ravel_multi_index(coord, cells)

        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension

        #print("nodeid", node_id)
        cells = [int(k) for k in self.num_cells]
        coord = np.unravel_index(node_id, cells)
        #print("id to grid", coord)
        return coord
        

    


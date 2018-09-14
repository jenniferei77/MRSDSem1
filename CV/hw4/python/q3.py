import numpy as np
import skimage.color
import skimage.io
from scipy import linalg



# Q 3.1
def computeH(l1, l2):
    H2to1 = np.eye(3)
    # YOUR CODE HERE
    
    return H2to1    

# Q 3.2
def computeHnorm(l1, l2):
    H2to1 = np.eye(3)
    # YOUR CODE HERE
    
    return H2to1   

# Q 3.3
def computeHransac(locs1, locs2):
    bestH2to1, inliers = None, None
    # YOUR CODE HERE
    
    return bestH2to1, inliers

# Q3.4
def compositeH( H2to1, template, img ):
    compositeimg = img
    # YOUR CODE HERE
    
    return compositeimg


def HarryPotterize():
    # we use ORB descriptors but you can use something else
    from skimage.feature import ORB,match_descriptors
    # YOUR CODE HERE
    
    return 
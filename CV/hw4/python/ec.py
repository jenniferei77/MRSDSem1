import numpy as np
# USE A SPARSE MATRIX
import scipy.sparse
from scipy.sparse.linalg import spsolve



# return an image with the correct stitching applied
def poissonStitch(fg,bg,mask):
    # here we select the region of interest for you
    res = np.copy(bg)
    yb, xb = np.where(mask[:,:,0] == 255)
    region_size = [yb.max()-yb.min(), xb.max()-xb.min()]
    elem_num = np.prod(region_size)
    # and create an appropriate sized matrix 
    # (technically you could use a smaller, non-square region)
    # (but we found this easier)
    A = scipy.sparse.identity(np.prod(region_size), format='lil')
    # YOUR CODE HERE
    
    return res
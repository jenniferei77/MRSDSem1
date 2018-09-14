import numpy as np
import scipy.io as sio
import skimage.feature
import matplotlib.pyplot as plt
import cv2 
from scipy.ndimage import gaussian_filter1d 

# Q2.1
# create a 2 x nbits sampling of integers from to to patchWidth^2
# read BRIEF paper for different sampling methods
def makeTestPattern(patchWidth, nbits):
    #res = None
    # YOUR CODE HERE
    #patch = np.zeros(patchWidth,patchWidth)
    #origin = patch[np.ceil(patchWidth/2), np.ceil(patchWidth/2)]
    #compareX = np.zeros(nbits,1)
    #compareY = np.zeros(nbits,1)
    sigma = float(1)/float(25)*(np.square(patchWidth))
    print np.transpose(np.random.normal(41,sigma,nbits)) 
    compareX = np.ceil(np.transpose(np.random.normal(41, sigma, nbits)))
    compareY = np.ceil(np.transpose(np.random.normal(41, sigma, nbits)))  
    
    #print np.size(compareX,1) #, np.size(compareX,2)
    #print compareX    
    #return res 
    return compareX, compareY
    
# Q2.2
# im is 1 channel image, locs are locations
# compareX and compareY are idx in patchWidth^2
# should return the new set of locs and their descs
def computeBrief(im,locs,compareX,compareY):
    desc = None
    # YOUR CODE HERE
    width = np.size(im,0)
    height = np.size(im,1)
    half = 5;
    #print locs
    #print locs > half
    #print np.transpose(locs[:,0]) < height-5
    #print np.transpose(locs[:,1]) < width-5    
    #mask = np.logical_and(np.logical_and(np.less(np.logical_and((locs > half), (np.transpose(locs[:,0])), (height-5))), (np.transpose(locs[:,1]) < (width-5))
    #print mask
    #index = np.argwhere(mask == 0)
    #print locs
    #print index
    #index = index[:,1]
    #np.delete(locs, index, axis = 0)
    patch_x = locs[:,0] - 9/2
    patch_y = locs[:,1] - 9/2
    patches = np.zeros((9,9))
    desc = np.zeros((np.size(locs,0),256))
          
    for i in range(0,len(patch_x)):
        truths = locs[i,:] < half
        single = np.all(truths)
        if single | locs[i,0] > height-5 | locs[i,1] > width-5:
            #locs = np.delete(locs, i, axis=0)
            True 
        else:	    
    	    patches = im[patch_x[i]:patch_x[i]+9, patch_y[i]:patch_y[i]+9]
            patch_line = np.reshape(patches, (1,81))
            for j in range(0,len(compareX)):
                desc[i,j] = patch_line[0,int(compareX[j])] > patch_line[0,int(compareY[j])]
     
    return locs, desc

# Q2.3
# im is a 1 channel image
# locs are locations
# descs are descriptors
# if using Harris corners, use a sigma of 1.5
def briefLite(im):
    locs, desc = None, None
    # YOUR CODE HERE
    compareX, compareY = makeTestPattern(9, 256)
    im = gaussian_filter1d(im,1)
    locs = skimage.feature.corner_peaks(skimage.feature.corner_fast(im), min_distance=1)   
    locs, desc = computeBrief(im, locs, compareX, compareY) 
    #print desc
    
    return locs, desc

# Q 2.4
def briefMatch(desc1,desc2,ratio=0.8):
    # okay so we say we SAY we use the ratio test
    # which SIFT does
    # but come on, I (your humble TA), don't want to.
    # ensuring bijection is almost as good
    # maybe better
    # trust me
    matches = skimage.feature.match_descriptors(desc1,desc2,'hamming',cross_check=True)
    return matches

def plotMatches(im1,im2,matches,locs1,locs2):
    fig, ax = plt.subplots(nrows=1, ncols=1)
    skimage.feature.plot_matches(ax,im1,im2,locs1,locs2,matches,matches_color='r')
    plt.show()
    return

def testMatch():
    # YOUR CODE HERE
    img1 = skimage.io.imread('../data/chickenbroth_01.jpg')
    img2 = skimage.io.imread('../data/model_chickenbroth.jpg')
    im1 = skimage.color.rgb2gray(img1)
    im2 = skimage.color.rgb2gray(img2)
    locs1, desc1 = briefLite(im1)
    locs2, desc2 = briefLite(im2)
    print "here i am", desc1, desc2
    matches = briefMatch(desc1, desc2, ratio=0.8)

    plotMatches(im1, im2, matches, locs1, locs2)
    return


# Q 2.5
# we're also going to use this to test our
# extra-credit rotational code
def briefRotTest(briefFunc=briefLite):
    # you'll want this
    import skimage.transform
    # YOUR CODE HERE
    
    return

# Q2.6
# YOUR CODE HERE


# put your rotationally invariant briefLite() function here
def briefRotLite(im):
    locs, desc = None, None
    # YOUR CODE HERE
    
    return locs, desc

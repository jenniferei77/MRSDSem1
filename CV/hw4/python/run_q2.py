from q2 import makeTestPattern, computeBrief, briefLite, briefMatch, testMatch, briefRotTest, briefRotLite

import scipy.io as sio
import skimage.color
import skimage.io
import skimage.feature
from scipy.ndimage import gaussian_filter1d

# Q2.1
compareX, compareY = makeTestPattern(9,256)
sio.savemat('testPattern.mat',{'compareX':compareX,'compareY':compareY})

# Q2.2
img = skimage.io.imread('../data/chickenbroth_01.jpg')
im = skimage.color.rgb2gray(img)

# YOUR CODE: Run a keypoint detector, with nonmaximum supression
# locs holds those locations n x 2
im = gaussian_filter1d(im,1)
locs = skimage.feature.corner_peaks(skimage.feature.corner_fast(im), min_distance=1)


locs, desc = computeBrief(im,locs,compareX,compareY)

# Q2.3
locs, desc = briefLite(im)

# Q2.4
testMatch()

# Q2.5
briefRotTest()

# EC 1
briefRotTest(briefRotLite)

# EC 2 
# write it yourself!

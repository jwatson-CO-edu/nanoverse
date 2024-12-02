########## INIT ####################################################################################

import os

import numpy as np

import cv2
import scipy.io



########## HELPER FUNCTIONS ########################################################################

def load_images_from_folder( folder ):
    """ Load all images in a `folder` """
    # Author: derricw, https://stackoverflow.com/a/30230738
    images = []
    for filename in os.listdir( folder ):
        img = cv2.imread( os.path.join( folder, filename ) )
        if img is not None:
            images.append( img )
    return images



########## SETUP ###################################################################################

# Load images
img_path = "images/"
img_data = load_images_from_folder( img_path )

# Load camera-pose data
rot_cam2ast = scipy.io.loadmat( 'rot_cam2ast.mat' )
baselines   = scipy.io.loadmat( 'baselines.mat'   )
n_poses     = baselines.shape[2]

# Compute relative pose between consecutive camera views
rel_rot_cam = np.zeros( (3,3,n_poses-1,) )
rel_pos     = np.zeros( (3,n_poses-1,)   )


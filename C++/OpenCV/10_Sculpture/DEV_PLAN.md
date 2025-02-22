# Guide
* Avoid serialization! YAGNI!
* Start with a sequence that has EASY keypoints, Do smooth examples LATER!


# `DEV_PLAN`

## Dependencies
* `[N]` Recompile OpenCV w/ image display, 2025-02-1X: Didn't work!
* `[Y]` Build Point Cloud Library
    - [Point Cloud Library (PCL) Build Instructions](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html)

## 2 Image Sequence, 2025-02-15: COMPLETE!
* `[Y]` Load images into nodes, 2025-02-02: Same as it ever was!
    - `[Y]` Find AKAZE (Again), 2025-02-02: Same as it ever was!
* `[Y]` Keypoint registration, 2025-02-03: Copy and Paste!
    - `[Y]` Fetch Claude response, Compare to ChatGPT response, 2025-02-03: Copy and Paste!
    - `[Y]` KNN, 2025-02-03: Copy and Paste!
    - `[N]` Save matching as a file and view it, 2025-02-15: Avoid serialization! YAGNI!
* `[Y]` Calc relative camera pose previous --to-> current, 2025-02-02: Relative!
    - `[Y]` Decide what the transform means, relative or absolute!, 2025-02-02: Relative!
* `[Y]` Generate PCD for registered image pair, 2025-02-15: Stored!
    - `[Y]` Fetch Claude response, 2025-02-05: THERE WERE PROBLEMS
    - `[Y]` ISSUE: SIZE AND TYPE MISMATCH IN GENERATED CODE, 2025-02-05: Just use `double`/`CV_64F`
        * `[N]` Fetch human example of `cv::triangulatePoints`, 2025-02-05: Just use `double`/`CV_64F`
        * `[N]` Step-by-Step rebuild, 2025-02-05: Just use `double`/`CV_64F`
    - `[Y]` Visualize the PCD! Does it make sense?, 2025-02-13: IT SURE FUCKING DOES! YEAH!!
        * [PCL Cloud Viewer](http://pointclouds.org/documentation/classpcl_1_1visualization_1_1_cloud_viewer.html)
        * [PCL Viewer Example](https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/pcl_visualizer/pcl_visualizer_demo.cpp)
        * `[Y]` Install Point Cloud Library (PCL), 2025-02-12: INSTALLED and TESTED!
        * `[Y]` Load into PCL data struct, 2025-02-13: Need to add some typecasts to silence warnings
        * `[Y]` Display, Does it look like the sculpture at all?, 2025-02-13: IT SURE FUCKING DOES! YEAH!!
    - `[Y]` Store the result struct in the SECOND node!, 2025-02-15: Stored!

## 6 Image Sequence
```C++
Mat undistortedImg1, undistortedImg2;
undistort(img1, undistortedImg1, cameraMatrix, distCoeffs);
undistort(img2, undistortedImg2, cameraMatrix, distCoeffs);
```  
* `[Y]` Calc absolute camera pose previous --to-> current, 2025-02-13: YEAH!!

* `[ ]` Keypoint Enhancement, ISSUE: THERE ARE BROAD REGIONS ON EACH IMAGE WITHOUT KEYPOINTS
    - `[ ]` Try reducing any threshold params for `AKAZE`
    - `{?}` Can you calc a keypoint for an arbitrary point on the image?
        * `[ ]` "Geo.hpp" comes before "SfM.hpp"
        * `[ ]` Tringulate existing keypoints
        * `[ ]` Identify sparse patches
        * `[ ]` Calc PCA and Centroid for each patch
        * `[ ]` **Subdivide** (by divisor, *not* by unit) patch with a grid of PCA-aligned points, Calc in image space
        * `[ ]` Append enhanced points to the above-threshold points

* `[ ]` ICP for PCD
    - `[ ]` [ICP Registration Ref](https://docs.opencv.org/4.x/dc/d9b/classcv_1_1ppf__match__3d_1_1ICP.html)
    - `[ ]` Try N-dim ICP that includes the coordinates of the associated keypoints

* `[P]` Cleanup, 2025-02-22: PAUSED
    - `[Y]` !! UNDISTORT !!, 2025-02-16: Marginal improvement
        * `[Y]` GET distortion params!, 2025-02-16: Marginal improvement
        * `[Y]` UNDISTORT EVERY IMAGE!!, 2025-02-16: Marginal improvement
    - `[Y]` Trim each local PCD to a Z-limit (distance from camera), 2025-02-14: Tune manually for now
    - `[Y]` Try skipping blurry images, but only if the registration succeeds, 2025-02-16: Less distortion, but weird scaling issue!
    - `[Y]` Eliminate the ground plane points, 2025-02-22: Removing the planes seems pointless as many pairs have very few points on the item of interest
        * `[Y]` Display resulting clouds, 2025-02-22: Removing the planes seems pointless as many pairs have very few points on the item of interest
        * `[N]` Try removing planes until they are small (Assumption: If it's a big plane, it's probably the ground or a building!), , 2025-02-22: Removing the planes seems pointless as many pairs have very few points on the item of interest
        * `{?}` Does this harm ICP performance?
        * `{?}` Does finding the plane help align for ICP?
    - `[N]` Try skipping images with insufficient disparity, but only if the registration succeeds, 2025-02-16: So far skipping for blurriness does not improve the distortion issue, So leave this one for now

    

## Troubleshooting
* `{?}` Poor Point Cloud Alignment
    - `{?}` [Find out what Homography is and if you can use it](https://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html)
    - `{?}` Extended points with {3D Position, Color, Keypoint, ?Color Gradient?}
        * `{?}` Is PCL able to do ICP with these points? What can?
    - `{?}` Try to match meshes instead of points
        * `{?}` Score = (Matched Area) * (1.0 / Distance)
        * `{?}` Match {curvature, color}, Look at archaeological reconstruction papers?

## 12 Image Sequence
* `[ ]` 1st Pass: Poses from registration
* `[ ]` 2nd Pass: Pose refinement from ICP
* `[ ]` 3rd Pass: Point merge and cleanup
* `[ ]` How to account for drift?

## N Image Sequence
* `[ ]` Consider loop closure

## Textured Model
* `{?}` Revive VFN?
    - `{?}` `DynVFN`: Queues of triples instead of matrices?
* `[ ]` Cluster total PCD into "surfaces"
    - `[ ]` Surfaces **should** share edges!
* `[ ]` Per PCD Surface
    - `[ ]` Choose the best camera node
    - `[ ]` Delaunay Triangulation
    - `[ ]` Compute texture coords for each 
* `[ ]` OBJ File Output
    - `{?}` Is there an OBJ library for C++?
    - `[ ]` How to save the textures?
    - `[ ]` Normalize image luminosity
    - `[ ]` Make models water tight
    - `[ ]` Test OBJ loading!

## Sculpture Garden 1
* `[ ]` Create 3 models
* `[ ]` HTML5 Canvas + [Three.js](https://threejs.org/)
* `[ ]` Large Grid
* `[ ]` Load OBJ
* `[ ]` 1st Person Nav
* `[ ]` Flat lighting
* `[ ]` Smooth lighting

## Sculpture Garden 2
* `[ ]` Create all available models
    - `[ ]` New Field Trip?
* `[ ]` Pleasant skybox
* `[ ]` Gentle green hills
* `[ ]` Grass shader

## Curriculum Vitae
* `[ ]` Title Card
* `[ ]` Dirt Paths
* `[ ]` Sign Post
    - `[ ]` Contact Me
    - `[ ]` Directions to Sculptures
    - `[ ]` Shift-Click for the Impatient
* `[ ]` Clickable Sculptures
    - `[ ]` Resume
    - `[ ]` Projects
    - `[ ]` GitHub
    - `[ ]` Logos
    - `[ ]` How It Was Made




## OpenGL Interface

# Resources
* [Raspi Capture Module](https://www.robotshop.com/products/arducam-12mp2-synchronized-stereo-camera-bundle-kit-for-raspberry-pi)
* [Raspi 4B](https://www.robotshop.com/products/yahboom-raspberry-pi-4b-board)

# Future Directions
* Loop closure? (2024 photos NOT taken with loop closure in mind!)
* Day/Night based on local user time
* Cast shadows
* Agents 
    - Boids (Animals)
    - Robots!
* Easter Eggs
    - My own sculptures --> Instagram Links
    - Rare Events
    - Holiday Events


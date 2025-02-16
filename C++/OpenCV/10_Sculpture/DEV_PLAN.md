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
* `[Y]` Calc absolute camera pose previous --to-> current, 2025-02-13: YEAH!!
* `[>]` Cleanup
    - `[Y]` Trim each local PCD to a Z-limit (distance from camera), 2025-02-14: Tune manually for now
    - `[>]` Try skipping blurry images, but only if the registration succeeds
    - `[ ]` Try skipping images with insufficient disparity, but only if the registration succeeds
* `[ ]` ICP for PCD
    - `[ ]` [ICP Registration Ref](https://docs.opencv.org/4.x/dc/d9b/classcv_1_1ppf__match__3d_1_1ICP.html)
    - `[ ]` Try N-dim ICP that includes the coordinates of the associated keypoints
    - `{?}` Consider eliminating the ground plane points, Does this harm ICP performance?

## 12 Image Sequence
* `[ ]` 1st Pass: Poses from registration
    - `{?}` Populate synthetic keypoints in medium gaps? How to ensure they are close together?
    - `{?}` Revive VFN?
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

## Troubleshooting

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


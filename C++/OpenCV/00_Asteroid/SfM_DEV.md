##### Sources #####
* Image Features
    - https://docs.opencv.org/4.x/db/d70/tutorial_akaze_matching.html
* Structure from Motion
    - https://imkaywu.github.io/tutorials/sfm/
    - https://docs.opencv.org/4.9.0/d9/d0c/group__calib3d.html#ga59b0d57f46f8677fb5904294a23d404a
    - https://en.wikipedia.org/wiki/Bundle_adjustment


##### DEV PLAN #####
* `[Y]` Load images in a dir, 2024-02-02: Req'd `pkg-config`
* `[Y]` 00 SURF Example, 2024-02-02: Finally, Finally, Finally
* `[Y]` ORB Example, Compute ORB features for one image, 2024-02-0X: Can specify how many. More might offer 
    greater match opportunities for matches, but take longer to compute correspondence. Do not forget 
    the AKAZE alternative: https://docs.opencv.org/4.x/db/d70/tutorial_akaze_matching.html
* `[>]` Basic SfM Tutorial: https://imkaywu.github.io/tutorials/sfm/
    - `[Y]` Compute ORB for all images, 2024-02-02: Easy!
    - `[P]` Feature matching
        * `[P]` Match features between each pair of images in the input image set,
            - `[Y]` Brute force, 2024-02-08: Not the longest wait ever
            - `[P]` Approximate nearest neighbour library, such as FLANN, ANN, Nanoflann
            https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
        * `[Y]` Bi-directional verification
            - `[Y]` Brute force, 2024-02-08: Have to wait quite a while to get mutual matches from brute force,
                5321 matched keypoint pairs for two images with 50000 features each.
    - `[>]` Relative pose estimation
        * Extrinsic and Intrinsic: 
            - The extrinsic parameters of a camera depend on its location and orientation and have nothing to do with 
              its internal parameters such as focal length, the field of view, etc. 
            - The intrinsic parameters of a camera depend on how it captures the images. 
              Parameters such as focal length, aperture, field-of-view, resolution, etc 
              govern the intrinsic matrix of a camera model.
        * K: Instrinsic Matrix, from calibration
        * F: Fundamental Matrix, 7DOF
            - The Fundamental matrix can be estimated using the 7-point algorithm + RANSAC
        * E: Essential Matrix, E = (K^T)FK
        * The fundamental matrix F is a generalization of the essential matrix E
        * One way to get a 3D position from a pair of matching points from two images is to take the fundamental matrix, 
          compute the essential matrix, and then to get the rotation and translation between the cameras from the 
          essential matrix. This, of course, assumes that you know the intrinsics of your camera. 
          Also, this would give you up-to-scale reconstruction, with the translation being a unit vector.
        * `[Y]` For each pair of images with sufficient number of matches, compute relative pose, 2024-02-19: OBTAINED
            * 2024-02-08: For now, assume that pictures with consecutive alpha filenames are related by virtue
                          of being taken in burst fashion while orbiting the subject. Close the loop. No search needed.
            - `[Y]` Compute the fundamental matrix F from match coordinates, 2024-02-10: Very fast, once you have matches!
            - `[Y]` Compute the essential matrix E from intrinsic matrix K and fundamental matrix F, 2024-02-19: OBTAINED
        * `[>]` N-view triangulation
            - `[Y]` Compute correspondences for each pair of images, 2024-02-19: OBTAINED
            - `[>]` Compute the extrinsics of each camera shot: 
                * `[Y]` Translation, 2024-02-19: OBTAINED
                * `[Y]` Orientation, 2024-02-19: OBTAINED
                * `[N]` Q: What is the relationship to the essential matrix?
                * `[>]` Render all relative poses as a sanity check
                * `[>]` Serialize all relative poses
            - `[>]` Find correspondence tracks across `ShotPair`s: class `StructureNode`
                * `[>]` Track, 3D Location, Color
                   - `[>]` Need to trace back to keypoints?
                   - `[>]` ISSUE: Something is VERY slow about building tracks!
                        * `[>]` Print out number of tracks vs how fast they are growing
                * `[ ]` Need to store error info?
                * Ref: `Structure_Point`: https://github.com/imkaywu/open3DCV/blob/master/src/sfm/structure_point.h
            - `[ ]` class `Graph`: https://github.com/imkaywu/open3DCV/blob/master/src/sfm/graph.h
                * `[ ]` Q: Does this offer anything that `StructureNode` does not?
            - `[ ]` Nonlinear Triangulation
                * `[ ]` pick graph g in graphs having maximum number of common tracks with the global graph
                * `[ ]` merge tracks of global graph with those of graph g
                * `[ ]` triangulate tracks
        * `[ ]` Bundle adjustment: Optimize a reprojection error with respect to all estimated parameters
    - `[ ]` N-view SfM
        * `[ ]` iterative step that merges multiple graphs into a global graph, for graph in graphs
            Loop:
            - `[ ]` perform bundle adjustment
            - `[ ]` remove track outliers based on baseline angle and reprojection error
        * `[ ]` Render PC 
            - `[ ]` Point Cloud
            - `[ ]` Camera poses
            - `[ ]` Fly around
        * `[ ]` Improvements
            - `[ ]` Filter the max distance for keypoint matches
            - `[ ]` Approximate nearest neighbour matching for keypoints
* `[ ]` Advanced Textured SfM
    - `[ ]` SfM Point Cloud
    - `[ ]` Iterative Delaunay at each camera view
       * `[ ]` Order points by increasing distance from the camera location for this shot
       * `[ ]` For each point
            - `[ ]` Project onto camera plane
            - `[ ]` Run Iterative Delaunay 
            - `[ ]` Compute points occluded by facets and remove from this view
       * `[ ]` Store candidate triangulation
       * `[ ]` Score Triangles: Higher penalty is worse
            - `[ ]` Angle between normal and ray to triangle center
            - `[ ]` Narrowness
    - `[ ]` Consolidate triangulations
       * `[ ]` Prefer low-penalty triangles
       * `[ ]` Find and fix holes: Complicated?
    - `[ ]` Consolidate textures @ facet objects
       * `[ ]` Project textures at each camera view
       * `[ ]` Prefer textures with sharp pixels
    - `[ ]` Write individual textures to a single image file
    - `[ ]` OBJ Output
    - `[ ]` Render OBJ (or other appropriate format)
* `[ ]` Virtual Sculpture Garden (Dangerous!)
    - `[ ]` Place sculptures tastefully
    - `[ ]` Add FPV
    - `[ ]` Post walkthrough to IG
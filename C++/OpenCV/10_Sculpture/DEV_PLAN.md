# Guide
* Avoid serialization! YAGNI!
* Start with a sequence that has EASY keypoints, Do smooth examples LATER!


# `DEV_PLAN`

## 2 Image Sequence
* `[Y]` Load images into nodes, 2025-02-02: Same as it ever was!
    - `[Y]` Find AKAZE (Again), 2025-02-02: Same as it ever was!
* `[Y]` Keypoint registration, 2025-02-03: Copy and Paste!
    - `[Y]` Fetch Claude response, Compare to ChatGPT response, 2025-02-03: Copy and Paste!
    - `[Y]` KNN, 2025-02-03: Copy and Paste!
* `[Y]` Calc relative camera pose previous --to-> current, 2025-02-02: Relative!
    - `[Y]` Decide what the transform means, relative or absolute!, 2025-02-02: Relative!
* `[>]` Generate PCD for registered image pair
    - `[Y]` Fetch Claude response, 2025-02-05: THERE WERE PROBLEMS
    - `[Y]` ISSUE: SIZE AND TYPE MISMATCH IN GENERATED CODE, 2025-02-05: Just use `double`/`CV_64F`
        * `[N]` Fetch human example of `cv::triangulatePoints`, 2025-02-05: Just use `double`/`CV_64F`
        * `[N]` Step-by-Step rebuild, 2025-02-05: Just use `double`/`CV_64F`
    - `[ ]` Visualize the PCD! Does it make sense?
    - `[ ]` Store the result struct in the SECOND node!
    - `{?}` Is there an **EXISTING** struct/class that I SHOULD use?

## 3 Image Sequence
* `[ ]` ICP for PCD
    - `[ ]` Try N-dim ICP that includes the coordinates of the associated keypoints
* `[ ]` Calc absolute camera pose previous --to-> current

## N Image Sequence
* `[ ]` PCD consensus across pairs?
* `[ ]` How to account for drift?
* `[ ]` Consider loop closure

## Troubleshooting
* `{?}` IF poor quality, THEN switch all OpenCV `Mat`s and calcs to `double`

## OpenGL Interface
* `[ ]` Switch homegrown vectors to GLM

## Textured Model
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


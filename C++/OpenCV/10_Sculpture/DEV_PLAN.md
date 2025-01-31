# Guide
* Avoid serialization! YAGNI!
* Start with a sequence that has EASY keypoints, Do smooth examples LATER!


# `DEV_PLAN`

## 2 Image Sequence
* `[>]` Load images into nodes
* `[ ]` Keypoint registration
* `[ ]` Calc relative camera pose previous --to-> current
* `[ ]` Generate PCD for registered image pair
    - `[ ]` Decide which node the PCD belongs to!
    - `{?}` Is there an **EXISTING** struct/class that I SHOULD use?

## N Image Sequence
* `[ ]` ICP for PCD
    - `[ ]` Try N-dim ICP that includes the coordinates of the associated keypoints
* `[ ]` Calc absolute camera pose previous --to-> current


# Future Directions
* Loop closure? (2024 photos NOT taken with loop closure in mind!)
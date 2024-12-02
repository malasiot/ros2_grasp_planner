## GraspNet service node

The service calls grasp-net model to obtain a list of grasp candidates given a color and depth image and a mask.

### Node parameters:

- **score_thresh**: Each candidate is annotated with a confidence score from 0 to 1 (the higher the better). Keep candidates with score greater than the threshold (default: 0.35)
- **collision_thresh**: The distance in meters that a grasp candidate will be considered as colliding with the point cloud (default: 0.05)
- **factor_depth**: This is a multiplication factor (in milimeter) to apply to 3D coordinates before running the model. (default: 4500.0)


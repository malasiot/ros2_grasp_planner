## GraspBox service node

The service detects boxes from an RGBD image pair and returns a list of associated grasp candidates.

### Node parameters:

- **gripper_offset**: How much to push each candidate away from the box (default: 0.05)
- **clearance**: The distance (in meters) of the gripper fingers from the sides of the box (default: 0.01)



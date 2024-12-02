## Grasp Planning exposed services

### grasp_candidates_service:

#### Input: 

- *algorithm*: Grasp candidate detection algorithm. One of 0 = grasp net, 1 = box detector

#### Output:
- *result*: 0 (OK), 1 (none found), 2(error)
- *grasps*: list of grasp candidates in world coordinate frame

#### Parameters:

- **viz_topic**: Topic to visualize found candidates (default: visual_grasps)
- **camera_frame**: The TF frame of the camera used

#### Other services/nodes used:
- [GraspNet] (../../../src/graspnet_service/graspnet_service/README.md)
- [BoxDetector] (../../../src/grasp_box/README.md)

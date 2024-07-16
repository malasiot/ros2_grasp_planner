
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from grasp_planner_interfaces.srv import GraspNet as GraspNetInterface
from grasp_planner_interfaces.msg import Grasp as GraspInterface
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import os
import sys
import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image

import torch
from graspnetAPI import GraspGroup, Grasp

ROOT_DIR =  os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from graspnet import GraspNet, pred_decode
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

def rotationMatrixToQuaternion3(m):
    #q0 = qw
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if(t > 0):
        t = np.sqrt(t + 1)
        q[3] = 0.5 * t
        t = 0.5/t
        q[0] = (m[2,1] - m[1,2]) * t
        q[1] = (m[0,2] - m[2,0]) * t
        q[2] = (m[1,0] - m[0,1]) * t

    else:
        i = 0
        if (m[1,1] > m[0,0]):
            i = 1
        if (m[2,2] > m[i,i]):
            i = 2
        j = (i+1)%3
        k = (j+1)%3

        t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k,j] - m[j,k]) * t
        q[j] = (m[j,i] + m[i,j]) * t
        q[k] = (m[k,i] + m[i,k]) * t

    return q

class GraspNetService(Node):
    def get_net(self):
        # Init the model
        num_view = self.get_parameter('num_view').value
        checkpoint_path = self.get_parameter('checkpoint_path').value
        
        net = GraspNet(input_feature_dim=0, num_view=num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        net.to(device)
        # Load checkpoint
        checkpoint = torch.load(checkpoint_path)
        net.load_state_dict(checkpoint['model_state_dict'])
        start_epoch = checkpoint['epoch']
        print("-> loaded checkpoint %s (epoch: %d)"%(checkpoint_path, start_epoch))
        # set model to eval mode
        net.eval()
        return net
    
    def get_and_process_data(self, color, depth, mask, camera_info):
        
        num_point = self.get_parameter('num_point').value 
        factor_depth = self.get_parameter('factor_depth').value 
        
        color = color.astype(np.float32)/255.0 ;

        height, width, _ = color.shape ;
        
        mask_path = os.path.join(get_package_share_directory("graspnet_service"), "data/mask.png")
        if os.path.isfile(mask_path):
            workspace_mask = np.array(Image.open(mask_path), dtype=bool)
            assert workspace_mask.shape[0] == height and workspace_mask.shape[1] == width
        else:
            workspace_mask = np.ones((height, width), dtype=bool)

        workspace_mask = np.array(mask, dtype=bool) | workspace_mask 
   
        k = camera_info.k.reshape(3, 3);
       
        # generate cloud
        camera = CameraInfo(width, height, k[0][0], k[1][1], k[0][2], k[1][2], factor_depth)
        cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

        # get valid points
        mask = (workspace_mask & (depth > 0))
        cloud_masked = cloud[mask]
        color_masked = color[mask]

        # sample points
        if len(cloud_masked) >= num_point:
            idxs = np.random.choice(len(cloud_masked), num_point, replace=False)
        else:
            idxs1 = np.arange(len(cloud_masked))
            idxs2 = np.random.choice(len(cloud_masked), num_point-len(cloud_masked), replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        cloud_sampled = cloud_masked[idxs]
        color_sampled = color_masked[idxs]

        # convert data
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
        cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
        end_points = dict()
        cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cloud_sampled = cloud_sampled.to(device)
        end_points['point_clouds'] = cloud_sampled
        end_points['cloud_colors'] = color_sampled

        return end_points, cloud
    
    def get_grasps(self, end_points):
    # Forward pass
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        gg_array = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(gg_array)
        return gg

    def collision_detection(self, gg, cloud):
        voxel_size = self.get_parameter('voxel_size').value
        collision_thresh = self.get_parameter('collision_thresh').value
        
        mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=voxel_size)
        collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=collision_thresh)
        gg = gg[~collision_mask]
        return gg
    
    def __init__(self):
        super().__init__('graspnet_service_node')
        
        self.declare_parameter('checkpoint_path', os.path.join(get_package_share_directory("graspnet_service"), "weights/checkpoint-rs.tar")) 
        self.declare_parameter('num_point', 75000)
        self.declare_parameter('num_view', 300)
        self.declare_parameter('collision_thresh', 0.01)
        self.declare_parameter('voxel_size', 0.005)
        self.declare_parameter('factor_depth', 3500.0)
        self.declare_parameter('score_thresh', 0.2)
    
        self.net = self.get_net()
        self.srv = self.create_service(GraspNetInterface, 'graspnet', self.graspnet_callback)

      


    def graspnet_callback(self, request, response):
        bridge = CvBridge()
        rgb_image = bridge.imgmsg_to_cv2(request.rgb, desired_encoding='bgr8')
        depth_image = bridge.imgmsg_to_cv2(request.depth, desired_encoding='mono16')
        mask_image = bridge.imgmsg_to_cv2(request.mask, desired_encoding='mono8')
        camera_info = request.camera_info
        
        end_points, cloud = self.get_and_process_data(rgb_image, depth_image, mask_image, camera_info)
        gg = self.get_grasps(end_points)
        
        if self.get_parameter('collision_thresh').value > 0:
            gg = self.collision_detection(gg, np.array(cloud.points))

        gg.nms()
        gg.sort_by_score()
    
        score_threshold = self.get_parameter("score_thresh").value ;
        grippers = []
       
        scale_factor = self.get_parameter('factor_depth').value / 1000.0

        for grasp in gg.grasp_group_array:
            if grasp[0] > score_threshold:
                msg = GraspInterface() ;
                msg.score = float(grasp[0]) ;
                msg.width = float(grasp[1] * scale_factor) ;
                msg.height = float(grasp[2] * scale_factor) ;
                msg.depth = float(grasp[3] * scale_factor) ;
            
                r = np.reshape(grasp[4:13].tolist(), (3, 3));
                rot = rotationMatrixToQuaternion3(r) ;
                msg.rotation.x = rot[0] ; 
                msg.rotation.y = rot[1] ; 
                msg.rotation.z = rot[2] ; 
                msg.rotation.w = rot[3] ; 
                t = grasp[13:16].tolist() ;
                msg.translation.x = t[0] * scale_factor ;
                msg.translation.y = t[1] * scale_factor ;
                msg.translation.z = t[2] * scale_factor ;
                
                grippers.append(Grasp(grasp).to_open3d_geometry()) ;
                response.grasps.append(msg)
       # grippers = ggg.to_open3d_geometry_list()
       # o3d.visualization.draw_geometries([cloud, *grippers])
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = GraspNetService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

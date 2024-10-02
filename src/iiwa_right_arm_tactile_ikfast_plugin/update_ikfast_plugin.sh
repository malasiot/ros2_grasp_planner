search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=iiwa.srdf
robot_name_in_srdf=iiwa
moveit_config_pkg=iiwa_moveit_config
robot_name=iiwa
planning_group_name=iiwa_right_arm_tactile
ikfast_plugin_pkg=iiwa_iiwa_right_arm_tactile_ikfast_plugin
base_link_name=iiwa_right_base
eef_link_name=iiwa_right_tactile_ee
ikfast_output_path=/home/malasiot/source/ros2_grasp_planner/src/iiwa_iiwa_right_arm_tactile_ikfast_plugin/src/iiwa_iiwa_right_arm_tactile_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path

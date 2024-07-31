FROM malasiot/sestosenso-certh-grasp-planner:v1

ARG ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"] 

# Dev container arguments
ARG USERNAME=ros
ARG UID=1000
ARG GID=${UID} 

WORKDIR /ws

COPY . /ws/

RUN chown -R ${UID}:${GID} /ws/

RUN vcs import src < src/grasp_planner/grasp_planner.repos && vcs import src < src/iiwa_ros2/iiwa_ros2.repos

RUN rosdep update && rosdep install --from-paths src -y --ignore-src 

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --merge-install

COPY /docker/entrypoint.sh /
  
ENTRYPOINT [ "/entrypoint.sh" ]

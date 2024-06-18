FROM malasiot/sestosenso-certh-grasp-planner:latest

RUN apt-get update ; apt-get install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

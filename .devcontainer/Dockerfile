FROM malasiot/ros2-humble-cuda-11.8:latest
WORKDIR /app

RUN apt-get update ; apt-get -y install python3-pip ros-humble-moveit ros-humble-moveit-servo ros-humble-controller-manager ros-humble-gazebo-dev ros-humble-gazebo-ros ros-humble-octomap-mapping

ARG TORCH_CUDA_ARCH_ARGS='6.0 6.1 6.2 8.0 8.6'
ENV TORCH_CUDA_ARCH_LIST=${TORCH_CUDA_ARCH_ARGS}

COPY requirements.txt .
RUN pip3 install -r requirements.txt

COPY pnet2 pointnet2
RUN cd pointnet2 ; python3 setup.py install
COPY knn knn
RUN cd knn ; python3 setup.py install
RUN git clone https://github.com/malasiot/graspnetAPI.git;cd graspnetAPI;pip3 install .

RUN apt-get update ; apt-get -y install libharfbuzz-dev libglfw3-dev
RUN git clone https://github.com/malasiot/xviz.git
RUN cd xviz; mkdir build; cd build; cmake ..; make install

#ARG USERNAME=ros
#ARG USER_UID=1000
#ARG USER_GID=$USER_UID

# Create a non-root user
#RUN groupadd --gid $USER_GID $USERNAME \
#  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # Add sudo support for the non-root user
#  && apt-get update \
#  && apt-get install -y sudo \
#  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
#  && chmod 0440 /etc/sudoers.d/$USERNAME \
#  && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
#RUN apt-get update && apt-get install -y git-core bash-completion \
#  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
#  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
#  && rm -rf /var/lib/apt/lists/* 

#RUN apt-get update ; apt-get -y install build-essential gdb

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
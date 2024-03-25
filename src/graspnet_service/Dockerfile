FROM malasiot/ros-humble-cuda:latest
WORKDIR /app

ARG TORCH_CUDA_ARCH_ARGS=8.6
ENV TORCH_CUDA_ARCH_LIST=${TORCH_CUDA_ARCH_ARGS}

COPY requirements.txt .
RUN apt-get update ; apt-get -y install python3-pip 
RUN pip3 install -r requirements.txt

COPY pointnet2 pointnet2
RUN cd pointnet2 ; python3 setup.py install
COPY knn knn
RUN cd knn ; python3 setup.py install
RUN git clone https://github.com/malasiot/graspnetAPI.git;cd graspnetAPI;pip3 install .

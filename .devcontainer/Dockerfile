FROM malasiot/sestosenso-certh-grasp-planner:latest

RUN mkdir "/home/ros/.gdb-eigen/" ; touch "/home/ros/.gdb-eigen/__init__.py"; wget -P "/home/ros/.gdb-eigen/" "https://gitlab.com/libeigen/eigen/-/raw/master/debug/gdb/printers.py"; echo 'python\nimport sys\nsys.path.insert(0, "/home/ros/.gdb-eigen/")\nfrom printers import register_eigen_printers\nregister_eigen_printers(None)\nend' > /home/ros/.gdbinit
RUN apt-get update ; apt-get install -y ros-humble-image-transport-plugins
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

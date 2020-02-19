FROM osrf/ros:kinetic-desktop-full

RUN /bin/bash -c "apt update"
RUN /bin/bash -c "apt install -y vim nano emacs"

RUN /bin/bash -c "mkdir -p catkin_ws/src/"

RUN /bin/bash -c "cd catkin_ws/src"

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_init_workspace && cd ../.."
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -j -C catkin_ws/"
FROM ros:noetic
ENTRYPOINT [ ]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update \
 && apt install -y --no-install-recommends ros-noetic-moveit ros-noetic-vision-msgs ros-noetic-tf2-sensor-msgs

RUN apt install -y python3-catkin-tools git python3-pip

RUN pip install -U numpy open3d

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source /opt/ros/noetic/setup.bash

RUN mkdir -p /root/catkin_ws/src

# clone and build message and service definitions
RUN /bin/bash -c 'cd /root/catkin_ws/src; \
                  git clone https://github.com/ichores-research/motion_msgs.git; \
                  git clone https://github.com/ichores-research/table_plane_extractor_msgs.git'
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin build && source devel/setup.bash'
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN source /root/catkin_ws/devel/setup.bash


WORKDIR /root/motion

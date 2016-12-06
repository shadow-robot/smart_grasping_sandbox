FROM osrf/ros:kinetic-desktop-full

# using bash instead of sh to be able to source
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update && \
    apt-get install -y python-catkin-tools && \
    mkdir -p /workspace/src && \
    cd /workspace/ && \
    source /opt/ros/kinetic/setup.bash && \
    catkin init

COPY . /workspace/src/

ENV TERM vt100

RUN source /opt/ros/kinetic/setup.bash && \
    cd /workspace/src && \
    git clone https://github.com/shadow-robot/pysdf.git && \
    git clone -b F_add_moveit_funtionallity https://github.com/shadow-robot/gazebo2rviz.git && \
    cd .. && catkin build && \
    rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

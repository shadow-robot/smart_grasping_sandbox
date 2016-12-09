FROM osrf/ros:indigo-desktop-full

# using bash instead of sh to be able to source
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update && \
    apt-get install -y python-catkin-tools ros-indigo-moveit-full wget && \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get remove -y gazebo2 && \
    apt-get install -y gazebo7 ros-indigo-gazebo7-ros-pkgs ros-indigo-gazebo7-ros-control ros-indigo-controller-manager python-pip && \
    mkdir -p /workspace/src && \
    cd /workspace/ && \
    source /opt/ros/indigo/setup.bash && \
    catkin init

COPY . /workspace/src/

ENV TERM xterm

RUN source /opt/ros/indigo/setup.bash && \
    cd /workspace/src && \
    git clone -b indigo-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git && \
    git clone https://github.com/shadow-robot/pysdf.git && \
    git clone -b F_add_moveit_funtionallity https://github.com/shadow-robot/gazebo2rviz.git && \
    git clone https://github.com/ros-industrial/universal_robot.git && \
    cd universal_robot && \
    rm -rf ur_kinematics && \
    cd /workspace/src && \
    wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gazebo7/00-gazebo7.list -O /etc/ros/rosdep/sources.list.d/00-gazebo7.list && \
    rosdep update && \
    rosdep install --default-yes --all --ignore-src && \
    catkin build

 COPY model_urdf.config /workspace/src/universal_robot/ur_description/model.config

# installing gzweb
RUN curl -sL https://deb.nodesource.com/setup_6.x | bash - && \
    apt-get install -y libjansson-dev nodejs libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential xvfb

RUN nohup Xvfb :1 -screen 0 1024x768x16 &> xvfb.log &

RUN cd ~ && \
    hg clone -r gzweb_2.0.0 https://bitbucket.org/osrf/gzweb && \
    DISPLAY=:1.0 && \
    export DISPLAY && \
    source "/workspace/devel/setup.bash" && \
    source "/usr/share/gazebo-7/setup.sh" && \
    cd /root/gzweb && \
    ls && \
    GAZEBO_MODEL_PATH=/workspace/src:/workspace/src/universal_robot:${GAZEBO_MODEL_PATH} ./deploy.sh

RUN apt-get install -y byobu nano

# cleanup
RUN rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

EXPOSE 8080 7681

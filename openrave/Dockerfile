FROM smart_grasping_sandbox

# Add the personalrobotics APT repository
RUN apt-get update && apt-get install -y curl
RUN sh -c 'echo "deb http://packages.personalrobotics.ri.cmu.edu/public trusty main" \
        > /etc/apt/sources.list.d/personalrobotics.list' \
    && curl https://www.personalrobotics.ri.cmu.edu/files/personalrobotics.gpg | sudo apt-key add -

# Install OpenRAVE and clear cache.
RUN apt-get update \
    && apt-get install -y pr-openrave python-scipy libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/* && \
    cd /workspace/src && \
    git clone https://github.com/personalrobotics/or_urdf.git && \
    cd .. && \
    source devel/setup.bash && \
    catkin build

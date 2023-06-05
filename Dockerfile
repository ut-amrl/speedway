FROM registry.hub.docker.com/library/ros:noetic

# install apt deps
RUN apt-get update && \
    apt-get install -y git libgflags-dev libpopt-dev \
                       libgoogle-glog-dev liblua5.1-0-dev \
                       libboost-all-dev libqt5websockets5-dev \
                       python-is-python3 libeigen3-dev sudo tmux

# install ros apt deps
RUN apt-get install -y ros-noetic-tf ros-noetic-angles

ARG UID=1000
RUN useradd dev -m -s /bin/bash -u $UID -G sudo && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER dev
WORKDIR /home/dev

RUN rosdep update

# clone deps
RUN git clone https://github.com/ut-amrl/amrl_maps.git && \
    git clone https://github.com/ut-amrl/amrl_msgs.git && \
    git clone https://github.com/ut-amrl/ut_automata.git --recurse-submodules

# set up .bashrc
RUN echo "source /opt/ros/noetic/setup.sh\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/ut_automata\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/speedway\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_maps\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.profile
RUN echo "source /opt/ros/noetic/setup.bash\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/ut_automata\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/speedway\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_maps\n" \
"export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.bashrc


# build deps (pre text messages)
RUN /bin/bash -lc "cd amrl_msgs && git pull && make"
RUN /bin/bash -lc "cd ut_automata && git pull && make"

# add launcher
ENV CS378_DOCKER_CONTEXT 1
COPY --chown=dev:dev ./tmux_session.sh /home/dev/tmux_session.sh
RUN chmod a+x /home/dev/tmux_session.sh
CMD [ "/home/dev/tmux_session.sh" ]
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
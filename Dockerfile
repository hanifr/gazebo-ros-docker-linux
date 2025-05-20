# Dockerfile
FROM gazebo:11

# Install ROS Noetic
RUN apt-get update && apt-get install -y lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Create workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Entry point
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
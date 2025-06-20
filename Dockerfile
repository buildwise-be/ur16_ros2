FROM ros:jazzy-ros-base

# Install ur_robot_driver
RUN apt-get update && apt-get install -y ros-jazzy-ur

# Install utilities
RUN apt-get update && apt-get install -y \
    vim \
    iputils-ping \
    net-tools \
    python3-pip \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-moveit \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-joy \
    ros-jazzy-control-toolbox \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-control \ 
    ros-jazzy-ros2-controllers \
    dos2unix


# Make ROS2 Workspace Dirss
RUN mkdir -p /home/dev_ws/src

# Copy ROS2 packages into workspace
COPY ./ros2_packages/ /home/dev_ws/src

#Check out ROS-TCP-Endpoint, ROS2 version
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint /home/dev_ws/src/ros_tcp_endpoint -b ROS2v0.7.0

# Reference script with commands to source workspace
COPY ./ros2_docker/source_ros.sh /home/dev_ws/source_ros.sh

# Change to workspace on sign in
RUN echo "cd home/dev_ws" >> ~/.bashrc

# Build the workspace
RUN cd home/dev_ws && . /opt/ros/jazzy/setup.sh && colcon build

# Source
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Source the workspace on sign in
RUN echo ". install/local_setup.bash" >> ~/.bashrc


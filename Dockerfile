# Base: Ubuntu 24.04 + ROS 2 Jazzy desktop full (RViz, Gazebo, tools)
FROM osrf/ros:jazzy-desktop-full

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Create a workspace directory inside the container
WORKDIR /ros2_ws

# Copy only the source code from your repo into the image
COPY src/ src/

# Use bash for subsequent RUN commands
SHELL ["/bin/bash", "-c"]

# Install required ROS 2 packages (add more if needed)
RUN apt-get update && \
    apt-get install -y \
        ros-jazzy-robot-localization \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build

# Automatically source ROS and your workspace when a shell starts
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Default command when someone runs the container
CMD ["bash"]


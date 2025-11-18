# Base: Ubuntu 24.04 + ROS 2 Jazzy desktop (RViz, etc.)
FROM ros:jazzy-desktop

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Create workspace directory
WORKDIR /ros2_ws

# Copy only the source code into the image
# (we don't copy build/install/log from host)
COPY src/ src/

# Use bash for RUN commands
SHELL ["/bin/bash", "-c"]

# Install dependencies (uncomment + adjust if you use rosdep)
# RUN apt-get update && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y && \
#     apt-get clean && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build

# Auto-source ROS + your workspace for every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Default command when container starts
CMD ["bash"]

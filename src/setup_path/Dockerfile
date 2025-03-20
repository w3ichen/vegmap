FROM tiryoh/ros2-desktop-vnc:humble

# Install Ignition Gazebo
RUN apt-get update && apt-get install -y \
    ignition-fortress \
    && rm -rf /var/lib/apt/lists/*

# Set default environment variables
ENV IGN_IP=host.docker.internal
ENV IGN_PARTITION=default


# Build and run: 
# docker build -t ros2-ignition-vnc .
# docker run -p 6081:80 \
#   -e IGN_PARTITION=$(hostname) \
#   --add-host=host.docker.internal:host-gateway \
#   ros2-ignition-vnc
# Use the official ROS 2 Foxy image as a base
FROM ros:humble

SHELL ["/bin/bash", "-c"]

# Set the working directory
WORKDIR "/ros2_ws"

# Install system dependencies and your listed libraries
RUN apt-get update && apt-get install -y \
    python3-pip \
    nlohmann-json3-dev \
    libcurl4-openssl-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install the Python package
RUN pip3 install RPi.GPIO

# Copy your existing ROS 2 workspace into the image
COPY src ./src
# Copy files into a dummy ubuntu directory
RUN mkdir -p /home/ubuntu/ros2_ws/src/car_to_backend && \
    cp -r /ros2_ws/src/car_to_backend/src /home/ubuntu/ros2_ws/src/car_to_backend/

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
	apt update -y && apt upgrade -y && \
	rosdep install -i --from-path src --rosdistro humble -y && \
	colcon build

WORKDIR /ros2_ws2

COPY new_ros ./src
RUN . /opt/ros/humble/setup.sh && \
	rosdep install -i --from-path src --rosdistro humble -y && \
	colcon build

ENTRYPOINT ["/bin/bash"]

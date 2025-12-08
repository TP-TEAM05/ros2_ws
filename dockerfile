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

# Copy config files 
COPY confs /confs

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
	apt update -y && apt upgrade -y && \
	rosdep install -i --from-path src --rosdistro humble -y && \
	colcon build

WORKDIR /ros2_ws2

RUN echo 'source /opt/ros/humble/setup.sh' >> /root/.bashrc
RUN echo 'source install/setup.bash' >> /root/.bashrc

COPY new_ros ./src
RUN apt update -y && \
    . /opt/ros/humble/setup.sh && \
	rosdep install -i --from-path src --rosdistro humble -y && \
	colcon build

ENTRYPOINT ["/bin/bash"]

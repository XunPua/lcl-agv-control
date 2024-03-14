# Base image
FROM ros:humble

# Update and rm lists compulsory!
# apt-get ONLY

# install system and ros2 related packages
RUN apt-get update && \
    apt-get install -y \
    nano \
    vim \
    curl \
    pip \
    net-tools \
    iputils-ping \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    ros-humble-rviz-visual-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros2-controllers-test-nodes \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-joy \
    libserial-dev \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-plotjuggler-ros \
    ros-humble-twist-mux \
    qtbase5-dev \
    qtdeclarative5-dev \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# install specific version setuptools to prevent errors during colcon build
RUN pip install setuptools==58.2.0

# MQTT
RUN pip install paho-mqtt

## Development PC ONLY 
# Setup for Gazebo Classic
RUN curl -sSL http://get.gazebosim.org | sh

# install gazebo ros packages
RUN apt-get update && \
    apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz-sim \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

## END Development PC ONLY


# Create a non-root user
# Linux default main user with UID 1000
# USERNAME is not important. files created will still be accessable 
ARG USERNAME=ros
ARG USER_UID=1000 
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Add Dialout group for serial communication
RUN usermod -aG dialout ${USERNAME}

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]

# Good practice
USER root
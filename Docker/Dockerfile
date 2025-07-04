# Base image with full desktop and Gazebo support
FROM osrf/ros:noetic-desktop-full

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin

# Set up GPU environment variables (safe for CPU-only too)
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}

# Set TurtleBot3 model globally
ENV TURTLEBOT3_MODEL=burger

# Create a non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME && \
    apt update && apt install -y sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Install ROS tools, TurtleBot3, Gazebo tools
RUN sudo apt update && sudo apt install -y \
    git curl gdb gdbserver python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-rosinstall \
    ros-noetic-rviz \
    ros-noetic-xacro \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-navigation \
    ros-noetic-turtlebot3-slam \
    ros-noetic-turtlebot3-teleop \
    ros-noetic-moveit \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-robot-state-publisher \
    python3-numpy \
    ros-noetic-dwa-local-planner && \
    sudo apt autoremove -y && sudo apt clean

# Init rosdep - NUR UPDATE, INIT IST BEREITS DURCH DAS BASISIMAGE ERFOLGT
RUN rosdep update # <--- DIES IST DIE KORREKTE ZEILE

# Python & ROS tooling
RUN pip install --upgrade pip && \
    pip install colcon-clean torch transformers protobuf

# Source ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Create ROS 1 workspace
RUN mkdir -p /home/$USERNAME/ros_ws_prolab/src && \
    echo "source /home/$USERNAME/ros_ws_prolab/devel/setup.bash" >> ~/.bashrc

# Default working dir and shell
WORKDIR /home/$USERNAME/ros_ws_prolab

CMD ["/bin/bash"]




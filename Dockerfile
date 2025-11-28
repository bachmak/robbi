# ---- base image ----
FROM ubuntu:22.04

# ---- common environment variables ----
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=iron

# ---- system packages ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    wget \
    git \
    lsb-release \
    gnupg2 \
    apt-utils \
    build-essential \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    cmake \
    unzip \
    nano \
    vim \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ---- add ROS2 iron package sources ----
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# ---- install python-dependencies for ROS2 ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    python3-rosdistro-modules \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ---- install ROS2 ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-iron-ros-core \
    ros-iron-ros-base \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ---- initialize rosdep ----
RUN rosdep init || true
RUN rosdep update

# ---- PlatformIO ----
RUN pip3 install --no-cache-dir platformio

# ---- additional dependencies for a better workflow ----
RUN apt update && apt install -y python3-venv gh

# ---- Micro-ROS setup ----
RUN git clone -b iron https://github.com/micro-ROS/micro_ros_setup.git /opt/micro_ros_setup
WORKDIR /opt/micro_ros_setup
RUN pip3 install --no-cache-dir empy lark-parser pyserial colcon-common-extensions

# ---- ROS2 iron setup for bash ----
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/iron/setup.bash" >> /root/.bashrc

# ---- VS Code remote container port (optional) ----
EXPOSE 8888

# ---- default command ----
CMD ["/bin/bash"]

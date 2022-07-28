FROM px4io/px4-dev-ros-melodic:2021-12-11

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

RUN apt-get update --fix-missing
RUN apt-get install -y tzdata iproute2

ENV TERM xterm-256color

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-shadow.list' \
	&& apt-get update \
	&& apt-get -y --quiet --no-install-recommends install \
	ros-melodic-desktop \
	ros-melodic-tf2-geometry-msgs \
	ros-melodic-navigation \
	ros-melodic-robot-localization \
	ros-melodic-robot-state-publisher \
	ros-melodic-hector-gazebo-plugins \
    ros-melodic-compressed-image-transport

# add user account
ARG UID=1000
ARG GID=1000
RUN userdel user && groupadd -g $GID -o user && useradd -m -u $UID -g $GID -o -s /bin/bash user

RUN apt-get -y install sudo
RUN echo "%sudo    ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && echo "user:user" | chpasswd && adduser user sudo

WORKDIR /home/user
USER user
RUN touch ~/.sudo_as_admin_successful

# install prerequisites
RUN sudo apt-get update
RUN sudo apt-get install -y libgoogle-glog-dev

# Install gcc-9/g++-9
RUN sudo apt-get install -y software-properties-common \
    && sudo add-apt-repository ppa:ubuntu-toolchain-r/test \
    && sudo apt-get update && sudo apt-get install -y gcc-9 g++-9 \
    && sudo rm /usr/bin/gcc /usr/bin/g++ && sudo ln -s /usr/bin/gcc-9 /usr/bin/gcc && sudo ln -s /usr/bin/g++-9 /usr/bin/g++

# Setup terminal environment
RUN curl -s https://raw.githubusercontent.com/JeiKeiLim/my_term/main/run.sh | /bin/bash

# Install FAST-LIO2 prerequisites
RUN sudo apt-get update
RUN sudo apt-get install -y libpcl-dev libeigen3-dev ros-melodic-pcl-ros ros-melodic-eigen-conversions git libyaml-cpp-dev

# Install ceres-solver
WORKDIR /home/user
RUN sudo apt-get install -y libgflags-dev \
    && sudo apt-get install -y libatlas-base-dev \
    && sudo apt-get install -y libsuitesparse-dev
RUN wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz \
    && tar zxf ceres-solver-2.1.0.tar.gz \
    && mkdir ceres-bin && cd ceres-bin \
    && cmake ../ceres-solver-2.1.0 && make -j3 && sudo make install

# Build livox_ros_driver
RUN echo "source /opt/ros/melodic/setup.sh" >> /home/user/.bashrc
WORKDIR /home/user
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src \
    && /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/ws_livox; catkin build' 
RUN echo "source ~/ws_livox/devel/setup.bash" >> /home/user/.bashrc

# Prebuild the algorithms
WORKDIR /home/user
RUN mkdir -p ~/catkin_ws/src
COPY src/ /home/user/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> /home/user/.bashrc
RUN rm -rf src

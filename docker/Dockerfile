# ROS 2 Humble base image.
FROM osrf/ros:humble-desktop-full

RUN useradd -rm -d /home/ubuntu -s /bin/bash -g root -G sudo -u 1000 zxf
RUN  echo 'zxf:123' | chpasswd

RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list
RUN apt-get update
# RUN apt-get install -y \
#         vim \
#         wget \
#         git \
#         bash-completion \
#         clangd-15 \
#         clangd-format
#         libpcl-dev \
#         libprotobuf-dev \
#         libbullet-dev \
#         libboost-all-dev \
#         libglew-dev \
#         freeglut3-dev \
#         software-properties-common \

# RUN update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-15 100

# ROS 2 Humble shell environment.
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=6" >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]

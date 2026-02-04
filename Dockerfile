# syntax=docker/dockerfile:1.6
FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Rome

# -------------------------------------------------
# Base system deps
# -------------------------------------------------
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    unzip \
    pkg-config \
    ccache \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3-dev \
    python3-pip \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libgtk-3-dev \
    libblas-dev liblapack-dev gfortran \
    && rm -rf /var/lib/apt/lists/* \
    software-properties-common

# -------------------------------------------------
# Install cuDNN 9 for CUDA 12.8
# -------------------------------------------------
RUN apt-get update
RUN apt-get install -y cudnn9-cuda-12
RUN rm -rf /var/lib/apt/lists/*

# -------------------------------------------------
# Install ROS2 Humble
# -------------------------------------------------
RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN add-apt-repository universe
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.jammy_all.deb"
RUN dpkg -i /tmp/ros2-apt-source.deb
RUN apt update -y
RUN apt upgrade -y
RUN apt install ros-humble-desktop -y
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV PATH=$ROS_ROOT/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=$ROS_ROOT/lib/python3.10/site-packages:$PYTHONPATH

# -------------------------------------------------
# ccache setup
# -------------------------------------------------
ENV CCACHE_DIR=/ccache
ENV CCACHE_MAXSIZE=5G

# -------------------------------------------------
# Clone OpenCV
# -------------------------------------------------
ARG OPENCV_VERSION=4.12.0
WORKDIR /opt

RUN git clone --branch 4.12.0 --depth 1 https://github.com/opencv/opencv.git
RUN git clone --branch 4.12.0 --depth 1 https://github.com/opencv/opencv_contrib.git

# -------------------------------------------------
# Configure OpenCV 4.12.0
# -------------------------------------------------
WORKDIR /opt/opencv/build

RUN cmake -G Ninja \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
    -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D BUILD_EXAMPLES=OFF \
    -D WITH_CUDA=ON \
    -D CUDA_ARCH_BIN=8.6 \
    -D CUDA_ARCH_PTX= \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D WITH_CUBLAS=ON \
    -D BUILD_opencv_java=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D CMAKE_C_COMPILER_LAUNCHER=ccache \
    -D CMAKE_CXX_COMPILER_LAUNCHER=ccache \
    ..

# -------------------------------------------------
# Build OpenCV (parallelismo controllato)
# -------------------------------------------------
RUN --mount=type=cache,target=/ccache \
    ninja -j4

RUN ninja install
RUN ldconfig
ENV OPENCV_DIR=/opt/opencv/build
# -------------------------------------------------
# LibTorch installation (version 2.10.0 for CUDA 12.8)
# -------------------------------------------------
RUN curl -L -o /tmp/libtorch.zip https://download.pytorch.org/libtorch/cu128/libtorch-shared-with-deps-2.10.0%2Bcu128.zip
RUN unzip /tmp/libtorch.zip -d /opt/
RUN rm /tmp/libtorch.zip

ENV Torch_DIR=/opt/libtorch/share/cmake/Torch
ENV CMAKE_PREFIX_PATH=/opt/libtorch:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=/opt/libtorch/lib:$LD_LIBRARY_PATH
# -------------------------------------------------
# Open3D 0.19 C++ API build and installation
# -------------------------------------------------
WORKDIR /opt

RUN curl -L -o open3d.tar.xz \
    https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0.tar.xz
RUN tar -xf open3d.tar.xz && rm open3d.tar.xz
RUN mv open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0 open3d

ENV Open3D_DIR=/opt/open3d/lib/cmake/Open3D
ENV CMAKE_PREFIX_PATH=/opt/open3d:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=/opt/open3d/lib:$LD_LIBRARY_PATH



WORKDIR /root

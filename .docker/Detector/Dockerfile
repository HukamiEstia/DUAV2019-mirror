FROM nvidia/cuda:9.0-cudnn7-devel-ubuntu16.04

RUN apt update && apt install -y --no-install-recommends \
    build-essential \
    apt-utils \
    cmake \
    git \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    python-dev \
    python-numpy \
    python3 \
    python3-pip \
    python3-dev \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libjasper-dev \
    libdc1394-22-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    python-opencv \
    libopencv-dev \
    libav-tools \
    python-pycurl \
    libatlas-base-dev \
    gfortran \
    webp \
    qt5-default \
    libvtk6-dev \
    zlib1g-dev \
    wget \
    protobuf-compiler \
    libeigen3-dev 

RUN pip3 install numpy

RUN cd ~/ &&\
    git clone https://github.com/Itseez/opencv.git &&\
    git clone https://github.com/Itseez/opencv_contrib.git &&\
    cd opencv && mkdir build && cd build && cmake \
    -DWITH_QT=ON \
    -DWITH_OPENGL=ON \
    -DFORCE_VTK=ON \
    -DWITH_TBB=ON \
    -DWITH_GDAL=ON \
    -DWITH_XINE=ON \
    -DBUILD_EXAMPLES=ON .. && \
    make -j4 && make install && ldconfig

RUN ln /dev/null /dev/raw1394

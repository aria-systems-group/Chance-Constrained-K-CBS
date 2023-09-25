FROM ubuntu:jammy

# LABEL about the custom image
LABEL maintainer="juko6110@colorado.edu"
LABEL version="1.0"
LABEL description="This is a custom Dockerfile for using OMPL 1.6 on Ubuntu 22.04 (Jammy)"

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive 

# libompl-dev \
RUN apt-get update && apt-get install -y build-essential \
										cmake \
										libeigen3-dev \
										libboost-all-dev \
										libode-dev \
										screen \
										vim \
										valgrind \
                                        git \
                                        wget

# get installation script from ompl website
RUN git clone --depth 1 --branch 1.6.0 https://github.com/ompl/ompl.git
WORKDIR ompl/
RUN mkdir -p build/Release; cd build/Release; cmake ../..; make install -j 8
WORKDIR /
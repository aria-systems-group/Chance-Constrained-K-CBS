FROM ubuntu:22.04

# LABEL about the custom image
LABEL maintainer="juko6110@colorado.edu"
LABEL version="1.0"
LABEL description="This is a custom Dockerfile for using OMPL 1.6"

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
                                        git

# clone most recent OMPL Repository
RUN git clone https://github.com/ompl/ompl.git

# build OMPL
WORKDIR ompl/
RUN mkdir -p build/Release
RUN cmake .
RUN make -j 4

WORKDIR /

# container set-up: docker run -v `pwd`:/home/K-CBS -it ompl-1.4-image
FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="juko6110@colorado.edu"
LABEL version="1.0"
LABEL description="This is a custom Dockerfile for using OMPL"

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive 

RUN apt-get update && apt-get install -y build-essential \
										cmake \
										libeigen3-dev \
										libboost-all-dev \
										libode-dev \
										libompl-dev \
										screen \
										vim \
										valgrind

# container set-up: docker run -v `pwd`:/home/K-CBS -it ompl-1.4-image

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
                                        git \
                                        wget

# get installation script from ompl website
RUN wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
RUN chmod u+x install-ompl-ubuntu.sh
RUN ./install-ompl-ubuntu.sh

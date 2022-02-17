FROM ubuntu:20.04

# LABEL about the custom image
LABEL maintainer="juko6110@colorado.edu"
LABEL version="0.1"
LABEL description="This is custom Docker Image is for Kinodynamic Conflict-Based Search."

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Update Ubuntu Software repository and install required packages
RUN apt-get update && apt-get install build-essential libeigen3-dev libboost-all-dev libode-dev libompl-dev libyaml-cpp-dev git screen vim valgrind -y

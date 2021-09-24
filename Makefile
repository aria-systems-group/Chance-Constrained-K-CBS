# Justin Kottinger
# ARIA Systems Research Group

# Prior to editing this file, please read the following:

# This Makefile is specifically designed to compile C++ code on 
# Ubuntu and link the files with the OMPL library. This is a shell directory, something that you can add to 
# as your needs change and become more ambitious. One may notice that this directory holds a 
# specific format. Specifically, 

# + includes
# 		+ *.h files here
# + obj
# 		+ *.o files here
# + src
# 		+ *.cpp files here

# Of course, one is able to add directories as they see fit. For example, C++ visualization is pretty non-intuitive. So, 
# I added a "txt" and "VizTools" directory for my work. My executables output to txt/ and I have a bunch of MATLAB files 
# in VizTools/. 

# However, if one is to change the set up specified above (includes, src, obj), this makefile is no longer 
# guarenteed to work. So, try to avoid putting your header and source files in other places. 
# One should never need to open the "obj" directory. But putting them there helps keep your parent 
# directory clean. 

##############################################################################################
############################# Safe - Customizable Portion ####################################
##############################################################################################

# this is the name of the executable
EXE := Planner

##############################################################################################
################################### Caution ##################################################
##############################################################################################

SRC_DIR := src
OBJ_DIR := obj

SRC := $(wildcard $(SRC_DIR)/*.cpp)
OBJ := $(SRC:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

CPPFLAGS := -std=c++17 -I/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3 \
				-I/opt/homebrew/Cellar/boost/1.76.0/include \
				-I/opt/homebrew/Cellar/ompl/1.5.2/include/ompl-1.5 \
				-I/opt/homebrew/Cellar/ode/0.16.2/include \
				-I/opt/homebrew/Cellar/yaml-cpp/0.6.3_1/include
LDPATHS := -L/opt/homebrew/lib 
LDLIBS   := -lompl -lboost_serialization -lode -lyaml-cpp
CC = g++

##############################################################################################
####################### NOTHING BELOW HERE SHOULD CHANGE #####################################
##############################################################################################

.PHONY: all clean

all: $(EXE)


$(EXE): $(OBJ)
	$(CC) $(LDPATHS) $^ $(LDLIBS) -o $@


$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@


$(OBJ_DIR):
	@mkdir $@

clean:
	$(RM) $(OBJ)
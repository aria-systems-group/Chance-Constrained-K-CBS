/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all code relevent to turning K-CBS solutions 
* to text files for visualization. 
*********************************************************************/
 
/* Author: Justin Kottinger */

#pragma once
// #include "World.h"
#include "multiAgentSetUp.h"
#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/PathControl.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/geometry.hpp>


namespace ob = ompl::base;
namespace oc = ompl::control;
namespace fs = std::filesystem;
namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::segment<Point> Segment;


std::vector<oc::PathControl> decentralizeTrajectory(std::vector<oc::PathControl> compPlan, 
    const World *w);

void checkDisjoint(const std::vector<oc::PathControl> plan,
    const int begin, int depth, bool &intersection, bool &done);

std::vector<int> generateExplanation(const std::vector<oc::PathControl> plan);

// generate date/time information to solutions or solution directories
std::string GetCurrentTimeForFileName();

// parent function for including date/time information to files
fs::path appendTimeToFileName(const fs::path& fileName);

// write solultion to the system
void write2sys(const std::vector<oc::PathControl> plan, 
        const std::vector<Agent*> agents, const std::string problem_name,
        std::vector<int> expCosts);

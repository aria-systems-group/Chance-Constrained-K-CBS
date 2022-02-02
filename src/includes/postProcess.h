/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all code relevent to turning K-CBS solutions 
* to text files for visualization. 
*********************************************************************/
 
/* Author: Justin Kottinger */

#include "World.h"
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

// explanation 
class expPathControl
{
    public:
        expPathControl(oc::PathControl p, const std::vector<int> c): 
        path_{p}, cost_{c} {}

    private:
        oc::PathControl path_;
        const std::vector<int> cost_;
};

void checkDisjoint(const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition>>> problem,
        std::vector<oc::PathControl> plan, const int begin, int depth, 
    bool &intersection, bool &done);

std::vector<int> generateExplanation(const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition>>> problem);

// generate date/time information to solutions or solution directories
std::string GetCurrentTimeForFileName();

// parent function for including date/time information to files
fs::path appendTimeToFileName(const fs::path& fileName);

// write solultion to the system
void write2sys(const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition>>> problem, 
        const std::vector<Agent*> agents, const std::string problem_name,
        std::vector<int> expCosts);

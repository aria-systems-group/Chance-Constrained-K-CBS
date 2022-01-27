/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all code relevent to turning K-CBS solutions 
* to text files for visualization. 
*********************************************************************/
 
/* Author: Justin Kottinger */

#include "World.h"
#include <filesystem>
#include <fstream>


namespace fs = std::filesystem;

// generate date/time information to solutions or solution directories
std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y_%m_%d_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '_');
    return s;
}

// parent function for including date/time information to files
fs::path appendTimeToFileName(const fs::path& fileName)
{
    return fileName.stem().string() + "_" + GetCurrentTimeForFileName() + fileName.extension().string();
}

// write solultion to the system
void write2sys(const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition>>> problem, 
        const std::vector<Agent*> agents, const std::string problem_name)
{
    fs::path sol_dir = "solutions/" + problem_name;
    fs::create_directories(sol_dir);
    for (int i = 0; i < problem.size(); i++)
    {
        std::string fileName = agents[i]->getName() + ".txt";
        auto filePath = fs::current_path() / sol_dir / fs::path(fileName); /// appendTimeToFileName(fileName); // e.g. MyPrettyFile_2018-06-09_01-42-00.txt
        std::ofstream file(filePath);
        const oc::PathControl traj = static_cast<oc::PathControl &>
            (*problem[i].second->getSolutionPath());
        traj.printAsMatrix(file);
    }
}
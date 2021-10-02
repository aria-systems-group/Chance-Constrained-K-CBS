/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file contains all code relevent to turning K-CBS solutions 
* to text files for visualization. 
*********************************************************************/
 
/* Author: Justin Kottinger */

#include "../includes/World.h"
#include <filesystem>
#include <fstream>


namespace fs = std::filesystem;

// can include date/time information to solutions, if desired
std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y_%m_%d_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '_');
    return s;
}

// parent function for including date/time information
fs::path appendTimeToFileName(const fs::path& fileName)
{
    return fileName.stem().string() + "_" + GetCurrentTimeForFileName() + fileName.extension().string();
}

// write solultion to the system
void write2sys(const std::vector<oc::SimpleSetup> problem, const std::vector<Agent*> agents)
{
	fs::create_directories("solution");
    for (int i = 0; i < problem.size(); i++)
    {
        std::string fileName = "solutions/sol_" + agents[i]->getName() + ".txt";
        auto filePath = fs::current_path() / fs::path(fileName); /// appendTimeToFileName(fileName); // e.g. MyPrettyFile_2018-06-09_01-42-00.txt
        std::cout << filePath << std::endl;
        std::ofstream file(filePath);
        const oc::PathControl p = problem[i].getSolutionPath();
        p.printAsMatrix(file);
    }
}
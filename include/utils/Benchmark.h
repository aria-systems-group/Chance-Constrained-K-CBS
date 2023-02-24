#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <ompl/util/Console.h>
// #include "Instance.h"
// #include "multiAgentSetUp.h"
// #include "Planners/MultiRobotRRT.h"
// #include "Planners/KCBS.h"
// #include "Planners/PBS.h"
// #include <ompl/control/SpaceInformation.h>


typedef std::vector<std::pair<std::string, std::vector<std::string>>> dataStruct;

dataStruct benchmark(const std::string problem, const double planningTime,
    const int benchType, const int mergParam, const std::string bypass);

void write_csv(const char* logName, dataStruct dataset);

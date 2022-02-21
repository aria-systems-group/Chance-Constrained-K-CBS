#include "World.h"
#include "multiAgentSetUp.h"
#include "MA_RRT.h"
#include "KD_CBS.h"
#include "pbs.h"
#include <ompl/control/SpaceInformation.h>


namespace oc = ompl::control;
namespace ob = ompl::base;
typedef std::vector<std::pair<std::string, std::vector<std::string>>> dataStruct;

dataStruct benchmark(const std::string problem, const double planningTime,
    const int benchType, const int mergParam);

void write_csv(const char* logName, dataStruct dataset);
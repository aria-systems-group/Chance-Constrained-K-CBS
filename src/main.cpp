/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* The main execution file that plans using KD-CBS
*********************************************************************/
 
/* Author: Justin Kottinger */

// #include "../includes/World.h"
#include "includes/multiAgentSetUp.h"
#include "includes/KD_CBS.h"
#include "includes/postProcessing.h"


namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace oc = ompl::control;

// OMPL_INFORM("OMPL version: %s", OMPL_VERSION);  // blue font
// OMPL_WARN("OMPL version: %s", OMPL_VERSION);  // yellow font
// OMPL_ERROR("OMPL version: %s", OMPL_VERSION); // red font
// these follow syntax of printf see (https://www.cplusplus.com/reference/cstdio/printf/)
 
int main(int argc, char ** argv)
{
    std::string problem = argv[1];
    World *w = yaml2world(problem);
    const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition>>> mmpp = multiAgentSetUp(w);
    oc::KD_CBS *p = new oc::KD_CBS(mmpp); // K_CBS
    p->setWorld(w);
    ob::PlannerPtr planner(p);
    OMPL_INFORM("Set-Up Complete");
    std::cout << "Setup Complete. Press ENTER to plan: ";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    bool solved = planner->solve(30.0);
    if (solved)
    {
        problem.resize(problem.size() - 4); // remove ".yml"
        printf("Writing Solution to %s \n", problem.c_str());
        write2sys(mmpp, w->getAgents(), problem);
    }
    return 0;
}

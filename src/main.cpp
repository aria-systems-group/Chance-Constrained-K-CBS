/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* The main execution file that plans using KD-CBS
*********************************************************************/
 
/* Author: Justin Kottinger */
#include "../includes/OdeFunctions.h"
#include "../includes/World.h"
#include "../includes/OdeFunctions.h"
#include "../includes/Goals.h"
#include "../includes/collisionChecking.h"
#include "../includes/multiAgentSetUp.h"


// this follows syntax of printf see (https://www.cplusplus.com/reference/cstdio/printf/)
// OMPL_INFORM("OMPL version: %s", OMPL_VERSION);  // blue font
// OMPL_WARN("OMPL version: %s", OMPL_VERSION);  // yellow font
// OMPL_ERROR("OMPL version: %s", OMPL_VERSION); // red font
 
int main(int argc, char ** argv)
{
    std::string problem = argv[1];
    World *w = yaml2world(problem);
    std::vector<oc::SimpleSetup> allAgentSetUp = multiAgentSimpleSetUp(w);
    OMPL_INFORM("Set-Up Complete. Press ENTER to begin planning.");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');

    for (oc::SimpleSetup s: allAgentSetUp)
    {
        ob::PlannerStatus solved = s.solve(10.0);
        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            s.getSolutionPath().printAsMatrix(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;
    }
    return 0;
}










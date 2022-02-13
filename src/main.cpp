/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* The main execution file that plans using KD-CBS
*********************************************************************/
 
/* Author: Justin Kottinger */

#include "includes/KD_CBS.h"
#include "includes/pbs.h"
#include "includes/postProcess.h"
#include "includes/MA_RRT.h"
#include <ompl/control/planners/rrt/RRT.h>


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
    
    std::string planner = argv[2];
    if (planner == "K-CBS")
    {
        oc::KD_CBS *p = new oc::KD_CBS(mmpp); // K_CBS
        p->setWorld(w);
        ob::PlannerPtr planner(p);
        OMPL_INFORM("Set-Up Complete");
        std::cout << "Setup Complete. Press ENTER to plan: ";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        bool solved = planner->solve(600.0);
        if (solved)
        {
            std::vector<oc::PathControl> plan;
            for (auto m: mmpp)
            {
                oc::PathControl traj = static_cast<oc::PathControl &>
                    (*m.second->getSolutionPath());
                traj.interpolate();
                plan.push_back(traj);
            }
            std::vector<int> expCosts = generateExplanation(plan);
            // for (auto m: mmpp)
            // {
            //     oc::PathControl traj = static_cast<oc::PathControl &>
            //         (*m.second->getSolutionPath());
            //     traj.printAsMatrix(std::cout);
            // }
            problem.resize(problem.size() - 4); // remove ".yml"
            printf("Writing Solution to %s \n", problem.c_str());
            write2sys(plan, w->getAgents(), problem, expCosts);
        }
    }
    else if (planner == "PBS")
    {
        oc::PBS *p = new oc::PBS(mmpp); // PBS
        p->setWorld(w);
        ob::PlannerPtr planner(p);
        OMPL_INFORM("Set-Up Complete");
        std::cout << "Setup Complete. Press ENTER to plan: ";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        bool solved = planner->solve(600.0);
        if (solved)
        {
            std::vector<oc::PathControl> plan;
            for (auto m: mmpp)
            {
                oc::PathControl traj = static_cast<oc::PathControl &>
                    (*m.second->getSolutionPath());
                traj.interpolate();
                plan.push_back(traj);
            }
            // explain and return sol
            std::vector<int> expCosts = generateExplanation(plan);
            problem.resize(problem.size() - 4); // remove ".yml"
            printf("Writing Solution to %s \n", problem.c_str());
            write2sys(plan, w->getAgents(), problem, expCosts);
        }
    }
    else if (planner == "RRT")
    {
        OMPL_WARN("Using centralized RRT currently only plans for the first agent in the problem definition. This is only a problem if you provided a yaml file that contains multiple agents.\n");
        oc::RRT *p = new oc::RRT(mmpp[0].first); // RRT for centralized planning
        p->setProblemDefinition(mmpp[0].second);
        p->setup();
        ob::PlannerPtr planner(p);
        OMPL_INFORM("Set-Up Complete");
        std::cout << "Setup Complete. Press ENTER to plan: ";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        bool solved = planner->solve(600.0);
        if (solved)
        {
            std::vector<oc::PathControl> plan;
            oc::PathControl traj = static_cast<oc::PathControl &>
                (*mmpp[0].second->getSolutionPath());
            plan.push_back(traj);
            plan = decentralizeTrajectory(plan, w);
            for (auto mp: plan)
                mp.printAsMatrix(std::cout);
            // std::vector<int> expCosts = generateExplanation(plan);
            // problem.resize(problem.size() - 4); // remove ".yml"
            // printf("Writing Solution to %s \n", problem.c_str());
            // write2sys(plan, w->getAgents(), problem, expCosts);
        }
    }
    else if (planner == "MA-RRT")
    {
        OMPL_WARN("Using centralized RRT currently only plans for the first agent in the problem definition. This is only a problem if you provided a yaml file that contains multiple agents.\n");
        oc::MA_RRT *p = new oc::MA_RRT(mmpp[0].first); // MA-RRT for centralized planning
        p->setProblemDefinition(mmpp[0].second);
        p->setup();
        ob::PlannerPtr planner(p);
        OMPL_INFORM("Set-Up Complete");
        std::cout << "Setup Complete. Press ENTER to plan: ";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        bool solved = planner->solve(600.0);
        if (solved)
        {
            std::vector<oc::PathControl> plan;
            oc::PathControl traj = static_cast<oc::PathControl &>
                (*mmpp[0].second->getSolutionPath());
            plan = decentralizeTrajectory(plan, w);
            std::vector<int> expCosts = generateExplanation(plan);
            problem.resize(problem.size() - 4); // remove ".yml"
            printf("Writing Solution to %s \n", problem.c_str());
            write2sys(plan, w->getAgents(), problem, expCosts);
        }
    }
    
    return 0;
}

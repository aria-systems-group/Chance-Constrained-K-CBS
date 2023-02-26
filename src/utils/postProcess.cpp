#include "utils/postProcess.h"
// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/control/spaces/RealVectorControlSpace.h>
// #include <ompl/base/spaces/SO2StateSpace.h>


// std::vector<oc::PathControl> decentralizeTrajectory(std::vector<oc::PathControl> composedPlan,
//     const World *w)
// {
//     /* function that takes in a centralized system and returns
//     traj for individual agents */
//     std::vector<oc::PathControl> plan;

//     for (int i = 0; i < composedPlan.size(); i++)
//     {
//         auto traj = composedPlan[i];
//         if ((w->getAgents()[i]->getDynamics() == "Dynamic Car") ||
//             (w->getAgents()[i]->getDynamics() == "Kinematic Car") ||
//             (w->getAgents()[i]->getDynamics() == "Dynamic Unicycle"))
//         {
//             plan.push_back(composedPlan[i]);
//         }
//         else if (w->getAgents()[i]->getDynamics() == "Two Dynamic Cars")
//         {
//             // create new space information
//             w->getAgents()[i]->changeDynamics("Dynamic Car");
//             const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
//                 std::shared_ptr<ob::ProblemDefinition>>> mmpp = multiAgentSetUp(w);
//             oc::PathControl mpath1(mmpp[0].first);
//             oc::PathControl mpath2(mmpp[0].first);
//             // get centralized info
//             auto states = traj.getStates();
//             auto controls = traj.getControls();
//             auto cntrlDurations = traj.getControlDurations();

//             for (int s = 0; s < states.size(); s++)
//             {
//                 /* composed state and controls */ 
//                 auto compState = states[s]->as<ob::CompoundStateSpace::StateType>();            
//                 double compDuration = cntrlDurations[s];
//                 /* seperate agent 1 */ 
//                 auto xyState1 = compState->as<ob::RealVectorStateSpace::StateType>(0)->values;
//                 const double theta1 = compState->as<ob::SO2StateSpace::StateType>(1)->value;
            
//                 // create state for agent 1
//                 ob::ScopedState<> scoped_st1(mmpp[0].first);
//                 scoped_st1[0] = xyState1[0];
//                 scoped_st1[1] = xyState1[1];
//                 scoped_st1[2] = xyState1[2];
//                 scoped_st1[3] = xyState1[3];
//                 scoped_st1[4] = theta1;
//                 ob::State *st1 = scoped_st1.get();

//                 // create control for agent 1
//                 if (s != states.size() - 1)
//                 {
//                     auto compControl = controls[s]->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     oc::Control *cntrl1 = mmpp[0].first->getControlSpace()->allocControl();
//                     auto cntrl1_vals = cntrl1->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     if (compControl[0] != 0 && compControl[1] != 0)
//                     {
//                         cntrl1_vals[0] = compControl[0];
//                         cntrl1_vals[1] = compControl[1];
//                         // append the info to path1
//                         mpath1.append(st1, cntrl1, compDuration);
//                     }
//                 }
//                 else
//                 {
//                     // append the info to path1
//                     mpath1.append(st1);
//                 }

//                 /* seperate agent 2 */ 
//                 auto xyState2 = compState->as<ob::RealVectorStateSpace::StateType>(2)->values;
//                 const double theta2 = compState->as<ob::SO2StateSpace::StateType>(3)->value;

//                 // create state for agent 1
//                 ob::ScopedState<> scoped_st2(mmpp[0].first);
//                 scoped_st2[0] = xyState2[0];
//                 scoped_st2[1] = xyState2[1];
//                 scoped_st2[2] = xyState2[2];
//                 scoped_st2[3] = xyState2[3];
//                 scoped_st2[4] = theta2;
//                 ob::State *st2 = scoped_st2.get();

//                 // create control for agent 2
//                 if (s != states.size() - 1)
//                 {
//                     auto compControl = controls[s]->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     oc::Control *cntrl2 = mmpp[0].first->getControlSpace()->allocControl();
//                     auto cntrl2_vals = cntrl2->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     if (compControl[2] != 0 && compControl[3] != 0)
//                     {
//                         cntrl2_vals[0] = compControl[2];
//                         cntrl2_vals[1] = compControl[3];
//                         // append the info to path2
//                         mpath2.append(st2, cntrl2, compDuration);
//                     }
                    
//                 }
//                 else
//                 {
//                     // append the info to path2
//                     mpath2.append(st2);
//                 }
//             }
//             plan.push_back(mpath1);
//             plan.push_back(mpath2);
//         }
//         else if (w->getAgents()[0]->getDynamics() == "Three Dynamic Cars")
//         {
//             // create new space information
//             w->getAgents()[0]->changeDynamics("Dynamic Car");
//             const std::vector<std::pair<std::shared_ptr<oc::SpaceInformation>, 
//                 std::shared_ptr<ob::ProblemDefinition>>> mmpp = multiAgentSetUp(w);

//             oc::PathControl mpath1(mmpp[0].first);
//             oc::PathControl mpath2(mmpp[0].first);
//             oc::PathControl mpath3(mmpp[0].first);
//             // get centralized info
//             auto states = traj.getStates();
//             auto controls = traj.getControls();
//             auto cntrlDurations = traj.getControlDurations();
//             for (int s = 0; s < states.size(); s++)
//             {
//                 /* composed state and controls */ 
//                 auto compState = states[s]->as<ob::CompoundStateSpace::StateType>();            
//                 double compDuration = cntrlDurations[s];
//                 /* seperate agent 1 */ 
//                 auto xyState1 = compState->as<ob::RealVectorStateSpace::StateType>(0)->values;
//                 const double theta1 = compState->as<ob::SO2StateSpace::StateType>(1)->value;
            
//                 // create state for agent 1
//                 ob::ScopedState<> scoped_st1(mmpp[0].first);
//                 scoped_st1[0] = xyState1[0];
//                 scoped_st1[1] = xyState1[1];
//                 scoped_st1[2] = xyState1[2];
//                 scoped_st1[3] = xyState1[3];
//                 scoped_st1[4] = theta1;
//                 ob::State *st1 = scoped_st1.get();

//                 // create control for agent 1
//                 if (s != states.size() - 1)
//                 {
//                     auto compControl = controls[s]->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     oc::Control *cntrl1 = mmpp[0].first->getControlSpace()->allocControl();
//                     auto cntrl1_vals = cntrl1->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     cntrl1_vals[0] = compControl[0];
//                     cntrl1_vals[1] = compControl[1];
//                     // append the info to path1
//                     mpath1.append(st1, cntrl1, compDuration);
//                 }
//                 else
//                 {
//                     // append the info to path1
//                     mpath1.append(st1);
//                 }

//                 /* seperate agent 2 */ 
//                 auto xyState2 = compState->as<ob::RealVectorStateSpace::StateType>(2)->values;
//                 const double theta2 = compState->as<ob::SO2StateSpace::StateType>(3)->value;

//                 // create state for agent 1
//                 ob::ScopedState<> scoped_st2(mmpp[0].first);
//                 scoped_st2[0] = xyState2[0];
//                 scoped_st2[1] = xyState2[1];
//                 scoped_st2[2] = xyState2[2];
//                 scoped_st2[3] = xyState2[3];
//                 scoped_st2[4] = theta2;
//                 ob::State *st2 = scoped_st2.get();

//                 // create control for agent 2
//                 if (s != states.size() - 1)
//                 {
//                     auto compControl = controls[s]->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     oc::Control *cntrl2 = mmpp[0].first->getControlSpace()->allocControl();
//                     auto cntrl2_vals = cntrl2->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     cntrl2_vals[0] = compControl[2];
//                     cntrl2_vals[1] = compControl[3];
//                     // append the info to path2
//                     mpath2.append(st2, cntrl2, compDuration);
//                 }
//                 else
//                 {
//                     // append the info to path2
//                     mpath2.append(st2);
//                 }

//                 /* seperate agent 3 */ 
//                 auto xyState3 = compState->as<ob::RealVectorStateSpace::StateType>(4)->values;
//                 const double theta3 = compState->as<ob::SO2StateSpace::StateType>(5)->value;

//                 // create state for agent 1
//                 ob::ScopedState<> scoped_st3(mmpp[0].first);
//                 scoped_st3[0] = xyState3[0];
//                 scoped_st3[1] = xyState3[1];
//                 scoped_st3[2] = xyState3[2];
//                 scoped_st3[3] = xyState3[3];
//                 scoped_st3[4] = theta3;
//                 ob::State *st3 = scoped_st3.get();

//                 // create control for agent 3
//                 if (s != states.size() - 1)
//                 {
//                     auto compControl = controls[s]->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     oc::Control *cntrl3 = mmpp[0].first->getControlSpace()->allocControl();
//                     auto cntrl3_vals = cntrl3->as<oc::RealVectorControlSpace::ControlType>()->values;
//                     cntrl3_vals[0] = compControl[4];
//                     cntrl3_vals[1] = compControl[5];
//                     // append the info to path2
//                     mpath3.append(st3, cntrl3, compDuration);
//                 }
//                 else
//                 {
//                     // append the info to path2
//                     mpath3.append(st3);
//                 }
//             }
//             plan.push_back(mpath1);
//             plan.push_back(mpath2);
//             plan.push_back(mpath3);
//         }
//         else
//         {
//             OMPL_ERROR("Cannot Decentrlaize Trajectory for current model.");
//             exit(1);
//         }
//     }
//     return plan;
// }

// void checkDisjoint(const std::vector<oc::PathControl> plan,
//     const int begin, int depth, bool &intersection, bool &done)
// {
// 	int numDone = 0;
//     // get the states from plan at specified depth
//     std::vector<std::vector<const ob::State*>> sysStates(plan.size());
//     for (int a = 0; a < plan.size(); a++)
//     {
//         for (int d = begin; d <= begin+depth; d++)
//         {
//             // add states to vector (if possible)
//             if (d < plan[a].getStateCount())
//             {
//                 sysStates[a].push_back(plan[a].getState(d));
//             }
//             else
//             {
//             	numDone++;
//             	break;
//             }
//         }
//     }
//     if (numDone == sysStates.size())
//     	done = true;

//     // not done, need to check disjoint
//     // printf("Interpolating trajectories...\n");
//     // connect states to a 2D line
//     std::vector<std::vector<Segment>> sysLines(plan.size());
//     for (int a = 0; a < sysStates.size(); a++)
//     {
//         const ob::StateSpace *space(plan[a].getSpaceInformation()->
//             getStateSpace().get());
//         // std::cout << sysStates[a].size() << std::endl;
//         // std::cout << depth << std::endl;
//         for (int i = 1; i < sysStates[a].size(); i++)
//         {
//             // create line from (i-1)-th to i-th index
//             const ob::State *st_i_m_1 = sysStates[a][i-1];
//             std::vector<double> reals_i_m_1;
//             space->copyToReals(reals_i_m_1, st_i_m_1);
//             Point pt_i_m_1(reals_i_m_1[0], reals_i_m_1[1]);

//             const ob::State *st_i = sysStates[a][i];
//             std::vector<double> reals_i;
//             space->copyToReals(reals_i, st_i);
//             Point pt_i(reals_i[0], reals_i[1]);

//             Segment line(pt_i_m_1, pt_i);

//             sysLines[a].push_back(line);
//         }
//     }
//     // printf("Checking lines are disjoint... \n");
//     for (int a1 = 0; a1 < sysLines.size(); a1++)
//     {
//         for (int a2 = 0; a2 < sysLines.size(); a2++)
//         {
//             if (a1 != a2)
//             {
//                 // check sysLines[a1] vs sysLines[a2]
//                 for (const Segment s1: sysLines[a1])
//                 {
//                     for (const Segment s2: sysLines[a2])
//                     {
//                         // printf("Agent: %i Line: (%.1f, %.1f) to (%.1f, %.1f) \n", 
//                         // 	0, bg::get<0, 0>(s1), bg::get<0, 1>(s1),
//                         // 	bg::get<1, 0>(s1), bg::get<1, 1>(s1));
//                         // printf("Agent: %i Line: (%.1f, %.1f) to (%.1f, %.1f) \n", 
//                         // 	1, bg::get<0, 0>(s2), bg::get<0, 1>(s2),
//                         // 	bg::get<1, 0>(s2), bg::get<1, 1>(s2));
//                         intersection = boost::geometry::intersects(s1, s2);
//                         if (intersection == true)
//                         {
//                             // printf("intersection occurs \n");
//                             return;
//                         }
//                     }
//                 }
//             }
//         }
//     }
// }

// std::vector<int> generateExplanation(const std::vector<oc::PathControl> plan)
// {
//     printf("Generating Explanation...\n");
//     int maxLength = 0;
//     for (int i = 0; i < plan.size(); i++)
//     {
//         if (plan[i].getStateCount() > maxLength)
//         	maxLength = plan[i].getStateCount();
//     }
//     std::vector<int> costVec;
//     int cost = 1;
//     int beginIdx = 0;
//     int depth = 1;
//     bool done = false;

//     while(!done)
//     {
//         if (depth % 100 == 0)
//             printf("Depth: %i \n", depth);
//         bool intersection = false;
//         checkDisjoint(plan, beginIdx, depth, intersection, done);
//         if (intersection == true && !done)
//         {
//             for (int d = 1; d < depth; d++)
//             	costVec.push_back(cost);
//             cost++;
//             beginIdx = beginIdx + depth - 1;
//             depth = 1;
//         }
//         else if (intersection == false && !done)
//         {
//             depth ++;
//         }
//         else if (done)
//         {
//             while (costVec.size() < maxLength)
//             	costVec.push_back(cost);
//             done = true;
//         }
//     }
//     printf("Done. Explanation contains %i images.\n", costVec.back());
//     return costVec;
// }



// // generate date/time information to solutions or solution directories
// std::string GetCurrentTimeForFileName()
// {
//     auto time = std::time(nullptr);
//     std::stringstream ss;
//     ss << std::put_time(std::localtime(&time), "%Y_%m_%d_%T"); // ISO 8601 without timezone information.
//     auto s = ss.str();
//     std::replace(s.begin(), s.end(), ':', '_');
//     return s;
// }

// // parent function for including date/time information to files
// fs::path appendTimeToFileName(const fs::path& fileName)
// {
//     return fileName.stem().string() + "_" + GetCurrentTimeForFileName() + fileName.extension().string();
// }

// write solultion to the system
void exportBeliefPlan(const std::vector<oc::PathControl*> plan, const std::string problem_name)
{
    fs::path sol_dir = "solutions/" + problem_name;
    std::filesystem::remove_all(sol_dir);
    fs::create_directories(sol_dir);
    for (int i = 0; i < plan.size(); i++)
    {
        std::string fileName = "agent" + std::to_string(i) + ".txt";
        auto filePath = fs::current_path() / sol_dir / fs::path(fileName);
        std::ofstream file(filePath);
        plan[i]->printAsMatrix(file);
        file.close();

        // print covariances
        std::string covName = "agent" + std::to_string(i) + "_covs.txt";
        auto covFilePath = fs::current_path() / sol_dir / fs::path(covName);
        std::ofstream MyFile(covFilePath);
        std::vector<ob::State*> states = plan[i]->getStates();
        for (auto itr = states.begin(); itr != states.end(); itr++) {
            Eigen::MatrixXd Sigma = (*itr)->as<RealVectorBeliefSpace::StateType>()->getCovariance();
            int r = Sigma.rows();
            int c = Sigma.cols();
            for (int i = 0; i < r; ++i)
            {
                for (int j = 0; j < c; ++j)
                {
                    MyFile << Sigma(i,j) << " ";
                }
            }
            MyFile << std::endl;
        }
    }
}
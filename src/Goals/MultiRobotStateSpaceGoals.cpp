#include "Goals/MultiRobotStateSpaceGoals.h"


R2MultiRobotGoal::R2MultiRobotGoal(const oc::SpaceInformationPtr &si, 
    std::vector<double> goal, const double toll, const std::string dynModel) : 
        ompl::base::Goal(si), goal_{goal}, toll_{toll}, dyn_{dynModel} {}

bool R2MultiRobotGoal::isSatisfied(const ob::State *st) const
{
    auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
    std::vector<std::vector<double>> poses;
    int numVs = 0;
    if (dyn_ == "Two Dynamic Cars")
        numVs = 2;
    else if (dyn_ == "Three Dynamic Cars")
        numVs = 3;
        
    if (dyn_ == "Two Dynamic Cars" || dyn_ == "Three Dynamic Cars")
    {
        for (int a = 0; a < numVs; a++)
        {
            auto xyState = compState->as<
                ob::RealVectorStateSpace::StateType>(2*a + 0);
            std::vector<double> xy_pos{xyState->values[0], xyState->values[1]};
            poses.push_back(xy_pos);
        }
    }
    else
        OMPL_ERROR("Composed Goal not implemented for current composed dynamics.");
        
    if (poses.size() != numVs)
        OMPL_ERROR("Goal checking not working as expected.");

    // given a pos for each agent, calc if all in goal
    for (int a = 0; a < poses.size(); a++)
    {
        const double dx_sq = pow(goal_[poses.size() * a] - poses[a][0], 2);
        const double dy_sq = pow(goal_[poses.size() * a + 1] - poses[a][1], 2);
        // if a single agent is out of goal, system is not in goal
        if (sqrt(dx_sq + dy_sq) > toll_)
            return false;
    }

    // survived the pos check through all agents, found goal
    return true;
}

// Is a state in a goal region, also fill "distance" with result
bool R2MultiRobotGoal::isSatisfied(const ob::State *st, double *distance) const
{
    auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
    std::vector<std::vector<double>> poses;
    int numVs = 0;
    if (dyn_ == "Two Dynamic Cars")
        numVs = 2;
    else if (dyn_ == "Three Dynamic Cars")
        numVs = 3;

    if (dyn_ == "Two Dynamic Cars" || dyn_ == "Three Dynamic Cars")
    {
        for (int a = 0; a < numVs; a++)
        {
            auto xyState = compState->as<
                ob::RealVectorStateSpace::StateType>(2*a + 0);
            std::vector<double> xy_pos{xyState->values[0], xyState->values[1]};
            poses.push_back(xy_pos);
        }
    }
    else
        OMPL_ERROR("Composed Goal not implemented for current composed dynamics.");
        
    /* Nothing passed here should change */
    if (poses.size() != numVs)
        OMPL_ERROR("Goal checking not working as expected.");

    // given a pos for each agent, calc if all in goal
    double dist = 0.0;
    bool isSolved = true;
    for (int a = 0; a < poses.size(); a++)
    {
        const double dx_sq = pow(goal_[poses.size() * a] - poses[a][0], 2);
        const double dy_sq = pow(goal_[poses.size() * a + 1] - poses[a][1], 2);
        dist += sqrt(dx_sq + dy_sq);
        // if a single agent is out of goal, system is not in goal
        if (sqrt(dx_sq + dy_sq) > toll_)
        {
            isSolved = false;
        }
    }

    // const double d = getDistance2D(st);
    distance = &dist;

    // survived the pos check through all agents, found goal
    return isSolved;
}

std::vector<int> R2MultiRobotGoal::isInGoal(const ob::State *st)
{
    auto compState = st->as<ompl::base::CompoundStateSpace::StateType>();
    std::vector<std::vector<double>> poses;
    int numVs = 0;
    if (dyn_ == "Two Dynamic Cars")
        numVs = 2;
    else if (dyn_ == "Three Dynamic Cars")
        numVs = 3;

    if (dyn_ == "Two Dynamic Cars" || dyn_ == "Three Dynamic Cars")
    {
        for (int a = 0; a < numVs; a++)
        {
            auto xyState = compState->as<
                ob::RealVectorStateSpace::StateType>(2*a + 0);
            std::vector<double> xy_pos{xyState->values[0], xyState->values[1]};
            poses.push_back(xy_pos);
        }
    }
    else
        OMPL_ERROR("Composed Goal not implemented for current composed dynamics.");
        
    /* Nothing passed here should change */
    if (poses.size() != numVs)
        OMPL_ERROR("Goal checking not working as expected.");

    // given a pos for each agent, calc if all in goal
    std::vector<int> idxSolved;
    for (int a = 0; a < poses.size(); a++)
    {
        const double dx_sq = pow(goal_[poses.size() * a] - poses[a][0], 2);
        const double dy_sq = pow(goal_[poses.size() * a + 1] - poses[a][1], 2);
        double dist = sqrt(dx_sq + dy_sq);
        // if a single agent is out of goal, system is not in goal
        if (dist < toll_)
            idxSolved.push_back(a);
    }
return idxSolved;
}

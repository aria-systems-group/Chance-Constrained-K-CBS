/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* The main source code of constraint RRT
* 
* constraint RRT is RRT but extended such that the trajectories 
* satisfy a given set of constraints. Most of this code is copied
* from OMPL. It was simply extended to check constraint satisfaction.
*********************************************************************/
 
/* Author: Ioan Sucan */
 
#include "includes/constraintRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
 
ompl::control::constraintRRT::constraintRRT(const SpaceInformationPtr &si) : 
    base::Planner(si, "Constraint RRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
 
    Planner::declareParam<double>("goal_bias", this, 
        &constraintRRT::setGoalBias, &constraintRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, 
        &constraintRRT::setIntermediateStates, 
        &constraintRRT::getIntermediateStates, "0,1");
}
 
ompl::control::constraintRRT::~constraintRRT()
{
    freeMemory();
}
 
void ompl::control::constraintRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) 
        { return distanceFunction(a, b); });
}
 
void ompl::control::constraintRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}
 
void ompl::control::constraintRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

void ompl::control::constraintRRT::updateConstraints(std::vector<const Constraint*> c)
{
    // // get minimum step size
    // OMPL_INFORM("Attempting to cut the tree from size %d!", nn_->size());
    /* clear old data */
    clear();

    /* update constraints */
    constraints_ = c;

    // // remove existing branch that caused constraints
    // while (lastGoalMotion_)
    // {
    //     nn_->remove(lastGoalMotion_);
    //     lastGoalMotion_ = lastGoalMotion_->parent;
    // }

    // // remove branches of tree that do not satisfy the constraints
    // std::vector<Motion *> motions;
    // // vector of all motions that do not satisfy new constraints
    // std::vector<const base::State*> badStates;
    // std::vector<Motion *> newTree = {};
    // std::vector<Motion *> needRemoved = {};
    // nn_->list(motions);
    // for (const auto &motion : motions)
    // {
    //     // check if the motion needs to be deleted
    //     if (! satisfiesConstraints(motion))
    //         badStates.push_back(motion->state);
    // }
    // if (badStates.empty())
    //     OMPL_INFORM("Successfully pruned tree!", nn_->size());
    // else
    // {
    //     for (const auto &motion : motions)
    //     {
    //         Motion *cpy = motion;
    //         std::vector<Motion *> path;
    //         bool badBranch = false;
    //         while (cpy != nullptr)
    //         {
    //             for (const base::State *st: badStates)
    //             {
    //                 if (siC_->equalStates(st, cpy->state))
    //                     badBranch = true;
    //             }
    //             path.push_back(cpy);
    //             cpy = cpy->parent;
    //         }
    //         if (!badBranch)
    //             newTree.push_back(motion);
    //         else
    //             needRemoved.push_back(motion);
    //     }
    //     nn_->clear();
    //     nn_->add(newTree);
    //     OMPL_INFORM("Releasing memory of old states.");
    //     for (auto &motion: needRemoved)
    //     {
    //         if (motion->state)
    //             si_->freeState(motion->state);
    //         if (motion->control)
    //             siC_->freeControl(motion->control);
    //         delete motion;
    //     }
    //     std::vector<Motion *> check = {};
    //     nn_->list(check);
    //     OMPL_INFORM("%d Nodes removed. ", (needRemoved.size()));
    //     for (Motion *m: check)
    //     {
    //         // verify that node satisfied constraint
    //         if (! satisfiesConstraints(m))
    //         {
    //             OMPL_ERROR("Tree Pruning Failed. Invalid nodes still exist!");
    //             exit(1);
    //         }
    //         // verify that every motion still goes to root node
    //         Motion *cpy = m;
    //         while (cpy != nullptr)
    //             cpy = cpy->parent;
    //     }
    //     OMPL_INFORM("Successfully cut the tree to %d nodes!", nn_->size());
    // }
}

bool ompl::control::constraintRRT::satisfiesConstraints(const Motion *n) const
{
    // examines if a motion from parent to n satisfies constraints
    // get minimum step size
    const double stepSize = siC_->getPropagationStepSize();
    // the root node is always valid
    if (n->parent)
    {
        
        PathControl p(si_);
        p.append(n->parent->state);
        p.append(n->state, n->control, (n->steps * stepSize));
        p.interpolate();
        
        // get control time from parent to current
        double currTime = 0;
        Motion *cpy = n->parent;
        while (cpy)
        {
            currTime = currTime + (cpy->steps * stepSize);
            cpy = cpy->parent;
        }

        // for all interpolation points, create shape of robot and check for disjointedness with constraints
        for (int k = 0; k < p.getStateCount(); k++)
        {
            // Get xy state and theta state (in rad [0, 2*pi]) from state object
            auto compState = p.getState(k)->as<base::CompoundStateSpace::StateType>();
            auto xyState = compState->as<base::RealVectorStateSpace::StateType>(0);
            const double cx = xyState->values[0];
            const double cy = xyState->values[1];
            const double theta = compState->as<base::SO2StateSpace::StateType>(1)->value;

            // Get important params from car object
            const double carWidth = a_->getShape()[0];
            const double carHeight = a_->getShape()[1];

            // turn (x,y, theta), width, length to a polygon object
            // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
            // TOP RIGHT VERTEX:
            const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
            const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
            std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
            // TOP LEFT VERTEX:
            const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
            const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
            std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
            // BOTTOM LEFT VERTEX:
            const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
            const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
            std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
            // BOTTOM RIGHT VERTEX:
            const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
            const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
            std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);
            // convert to string for easy initializataion
            std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
            polygon currShape;
            boost::geometry::read_wkt(points,currShape);
            // need to check validity against all constraints
            /* 
            This is incorrect, need to be more precise in that specific time to specific geometry. 
            Checking the entire set of polygons within a range is over-conservative 
            */
            for (const Constraint *c: constraints_)
            {
                for (int t = 0; t < c->getTimes().size(); t++)
                {
                    if (c->getTimes()[t] == currTime)
                    {
                        if (! boost::geometry::disjoint(currShape, c->getPolygons()[t]))
                            return false; 
                    }
                }
            }
            currShape.clear();
            currTime = currTime + p.getControlDuration(k);
        }
        return true;
    }
    else
    {
        // at root node
        // Get xy state and theta state (in rad [0, 2*pi]) from state object
        auto compState = n->state->as<base::CompoundStateSpace::StateType>();
        auto xyState = compState->as<base::RealVectorStateSpace::StateType>(0);
        const double cx = xyState->values[0];
        const double cy = xyState->values[1];
        const double theta = compState->as<base::SO2StateSpace::StateType>(1)->value;

        // Get important params from car object
        const double carWidth = a_->getShape()[0];
        const double carHeight = a_->getShape()[1];

        // turn (x,y, theta), width, length to a polygon object
        // see https://stackoverflow.com/questions/41898990/find-corners-of-a-rotated-rectangle-given-its-center-point-and-rotation
        // TOP RIGHT VERTEX:
        const double TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
        const double TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
        std::string top_right = std::to_string(TR_x) + " " + std::to_string(TR_y);
        // TOP LEFT VERTEX:
        const double TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
        const double TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));
        std::string top_left = std::to_string(TL_x) + " " + std::to_string(TL_y);
        // BOTTOM LEFT VERTEX:
        const double BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
        const double BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
        std::string bottom_left = std::to_string(BL_x) + " " + std::to_string(BL_y);
        // BOTTOM RIGHT VERTEX:
        const double BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
        const double BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));
        std::string bottom_right = std::to_string(BR_x) + " " + std::to_string(BR_y);
        // convert to string for easy initializataion
        std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
        polygon currShape;
        boost::geometry::read_wkt(points,currShape);
        // for (const Constraint *c: constraints_)
        // {
        //     if ((c->getTimeRange()->first <= 0) && (0 <= c->getTimeRange()->second))
        //     {
        //         // need to check validity
        //         for (const polygon p: c->getPolygons())
        //             if (! boost::geometry::disjoint(currShape, p))
        //                 return false;
        //     }
        // }
        for (const Constraint *c: constraints_)
        {
            for (int t = 0; t < c->getTimes().size(); t++)
            {
                if (c->getTimes()[t] == 0)
                {
                    if (! boost::geometry::disjoint(currShape, c->getPolygons()[t]))
                        return false; 
                }
            }
        }
        return true;
    }
}
 
ompl::base::PlannerStatus ompl::control::constraintRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    if (a_ == nullptr)
    {
        OMPL_ERROR("No agent object.");
        exit(1);
    }
    // if (constraints_.size() > 0)
    // {
    //     // OMPL_WARN("Resolving %d Constraints.", constraints_.size());
    //     while (const base::State *st = pis_.nextStart())
    //     {
    //         auto *motion = new Motion(siC_);
    //         si_->copyState(motion->state, st);
    //         siC_->nullControl(motion->control);
    //         nn_->add(motion);
    //     }
    // }
    // else
    // {
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
    }
    // }
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
 
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();
 
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
    // for (const Constraint *c: constraints_)
    // {
    //     std::cout << c->getTimes().front() << ", " << c->getTimes().back() << std::endl;
    //     const std::vector <const polygon> ps = c->getPolygons();
    //     for (auto p: ps)
    //     {
    //         auto exterior_points = boost::geometry::exterior_ring(p);
    //         for (int i=0; i<exterior_points.size(); i++)
    //         {
    //             std::cout << boost::geometry::get<0>(exterior_points[i]) << ", " << boost::geometry::get<1>(exterior_points[i]) <<std::endl;
    //         }
    //     }
    // }

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
 
    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();
 
    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);
 
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
 
        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);
 
            if (cd >= siC_->getMinControlDuration())
            {
                OMPL_ERROR("Undefined Constraint Behaviour. Please impliment extensions when adding intermediate states.");
                exit(1);
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for (; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    auto *motion = new Motion();
                    motion->state = pstates[p];
                    // we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    nn_->add(motion);
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }
 
                // free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (auto &pstate : pstates)
                    si_->freeState(pstate);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* create a motion */
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;
                if (!constraints_.empty())
                {
                    if (satisfiesConstraints(motion))
                    {
                        nn_->add(motion);
                        double dist = 0.0;
                        bool solv = goal->isSatisfied(motion->state, &dist);
                        if (solv)
                        {
                            approxdif = dist;
                            solution = motion;
                            break;
                        }
                        if (dist < approxdif)
                        {
                            approxdif = dist;
                            approxsol = motion;
                        }
                    }
                    else
                        delete motion;
                }
                else
                {
                    // no constraints given, proceed as normal 
                    nn_->add(motion);
                    double dist = 0.0;
                    bool solv = goal->isSatisfied(motion->state, &dist);
                    if (solv)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }
                
            }
        }
    }
    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }
 
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;
 
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }
 
        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }
 
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);
 
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
 
    return {solved, approximate};
}
 
void ompl::control::constraintRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
 
    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);
 
    double delta = siC_->getPropagationStepSize();
 
    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
 
    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
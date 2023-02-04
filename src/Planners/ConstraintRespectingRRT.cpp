#include "Planners/ConstraintRespectingRRT.h"


oc::ConstraintRespectingRRT::ConstraintRespectingRRT(const SpaceInformationPtr &si): 
    ConstraintRespectingPlanner(si, "ConstraintRespectingRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
 
    ConstraintRespectingPlanner::declareParam<double>("goal_bias", this, &ConstraintRespectingRRT::setGoalBias, &ConstraintRespectingRRT::getGoalBias, "0.:.05:1.");
    ConstraintRespectingPlanner::declareParam<bool>("intermediate_states", this, &ConstraintRespectingRRT::setIntermediateStates, &ConstraintRespectingRRT::getIntermediateStates,
                                "0,1");
}
  
oc::ConstraintRespectingRRT::~ConstraintRespectingRRT()
{
    freeMemory();
}
  
void oc::ConstraintRespectingRRT::setup()
{
    ConstraintRespectingPlanner::setup();
    if (!nn_)
        nn_.reset(tools::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}
  
void oc::ConstraintRespectingRRT::clear()
{
    ConstraintRespectingPlanner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}
  
void oc::ConstraintRespectingRRT::freeMemory()
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

void oc::ConstraintRespectingRRT::updateConstraints(std::vector<ConstraintPtr> c)
{
	/* clear old data */
	clear();

	/* clear old solutions */
	getProblemDefinition()->clearSolutionPaths();

	/* update constraints */
	constraints_ = c;
}

ob::PlannerStatus oc::ConstraintRespectingRRT::solve(const ob::PlannerTerminationCondition &ptc)
{
	checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<ob::GoalSampleableRegion *>(goal);
    
    if (!replanning_)
    {
        while (const base::State *st = pis_.nextStart())
        {
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, st);
            siC_->nullControl(motion->control);
            nn_->add(motion);
        }
    }
    else
        replanning_ = false; // reset value;

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
        unsigned int cd = 0;
        // if (isCentralized_)
            // cd = MultiAgentControlSampler(rctrl, nmotion->control, nmotion->state, rmotion->state);
        // else
        cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
 
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
                    oc::PathControl p(si_);
                    p.append(motion->parent->state);
                    p.append(motion->state, motion->control, (motion->steps * siC_->getPropagationStepSize()));
                    p.interpolate();
                    if (constraintValidator_->satisfiesConstraints(p, constraints_))
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

void oc::ConstraintRespectingRRT::getPlannerData(ob::PlannerData &data) const
{
    ConstraintRespectingPlanner::getPlannerData(data);
  
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

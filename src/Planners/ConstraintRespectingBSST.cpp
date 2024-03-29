#include "Planners/ConstraintRespectingBSST.h"

ompl::control::ConstraintRespectingBSST::ConstraintRespectingBSST(const SpaceInformationPtr &si): 
	ConstraintRespectingPlanner(si, "ConstraintRespectingBSST")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
    prevSolution_.clear();
    prevSolutionControls_.clear();
    prevSolutionSteps_.clear();

    ConstraintRespectingPlanner::declareParam<double>("goal_bias", this, &ConstraintRespectingBSST::setGoalBias, &ConstraintRespectingBSST::getGoalBias, "0.:.05:1.");
    ConstraintRespectingPlanner::declareParam<double>("selection_radius", this, &ConstraintRespectingBSST::setSelectionRadius, &ConstraintRespectingBSST::getSelectionRadius, "0.:.1:"
                                                                                                                "100");
    ConstraintRespectingPlanner::declareParam<double>("pruning_radius", this, &ConstraintRespectingBSST::setPruningRadius, &ConstraintRespectingBSST::getPruningRadius, "0.:.1:100");
}

ompl::control::ConstraintRespectingBSST::~ConstraintRespectingBSST()
{
    freeMemory();
}

void ompl::control::ConstraintRespectingBSST::setup()
{
    ConstraintRespectingPlanner::setup();
    if (!nn_)
        nn_.reset(tools::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
    if (!witnesses_)
        witnesses_.reset(tools::getDefaultNearestNeighbors<Motion *>(this));
    witnesses_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                    {
                                        return distanceFunction(a, b);
                                    });

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
            if (dynamic_cast<base::MaximizeMinClearanceObjective *>(opt_.get()) ||
                dynamic_cast<base::MinimaxObjective *>(opt_.get()))
                OMPL_WARN("%s: Asymptotic near-optimality has only been proven with Lipschitz continuous cost "
                          "functions w.r.t. state and control. This optimization objective will result in undefined "
                          "behavior",
                          getName().c_str());
        }
        else
        {
            OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }

    prevSolutionCost_ = opt_->infiniteCost();

    max_eigenvalue_ = 10.0;
}

void ompl::control::ConstraintRespectingBSST::clear()
{
    ConstraintRespectingPlanner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (witnesses_)
        witnesses_->clear();
    if (opt_)
        prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::control::ConstraintRespectingBSST::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state_) {
                si_->freeState(motion->state_);
            }
            if (motion->control_) {
                siC_->freeControl(motion->control_);
            }
            delete motion;
        }
    }
    if (witnesses_)
    {
        std::vector<Motion *> witnesses;
        witnesses_->list(witnesses);
        for (auto &witness : witnesses)
        {
            delete witness;
        }
    }
    for (auto &i : prevSolution_)
    {
        if (i)
            si_->freeState(i);
    }
    prevSolution_.clear();
    for (auto &prevSolutionControl : prevSolutionControls_)
    {
        if (prevSolutionControl)
            siC_->freeControl(prevSolutionControl);
    }
    prevSolutionControls_.clear();
    prevSolutionSteps_.clear();
}

ompl::control::ConstraintRespectingBSST::Motion *ompl::control::ConstraintRespectingBSST::selectNode(ompl::control::ConstraintRespectingBSST::Motion *sample)
{
    std::vector<Motion *> ret;
    Motion *selected = nullptr;
    base::Cost bestCost = opt_->infiniteCost();
    nn_->nearestR(sample, selectionRadius_, ret);

    for (auto &i : ret)
    {
        if (!i->inactive_ && opt_->isCostBetterThan(i->accCost_, bestCost))
        {
            bestCost = i->accCost_;
            selected = i;
        }
    }
    if (selected == nullptr)
    {
        int k = 1;
        while (selected == nullptr)
        {
            nn_->nearestK(sample, k, ret);
            for (unsigned int i = 0; i < ret.size() && selected == nullptr; i++)
                if (!ret[i]->inactive_)
                    selected = ret[i];
            k += 5;
        }
    }
    return selected;
}

ompl::control::ConstraintRespectingBSST::Witness *ompl::control::ConstraintRespectingBSST::findClosestWitness(ompl::control::ConstraintRespectingBSST::Motion *node)
{
    if (witnesses_->size() > 0)
    {
        // std::cout << "154" << std::endl;
        auto *closest = static_cast<Witness *>(witnesses_->nearest(node));
        // si_->printState(closest->state_, std::cout);
        // si_->printState(node->state_, std::cout);
        // std::cout << pruningRadius_ << std::endl;
        // std::cout << distanceFunction(closest, node) << std::endl;
        if (distanceFunction(closest, node) > pruningRadius_)
        {
            // std::cout << "here" << std::endl;
            closest = new Witness(siC_);
            closest->linkRep(node);
            si_->copyState(closest->state_, node->state_);
            witnesses_->add(closest);
        }
        return closest;
    }
    else
    {
        // std::cout << "167" << std::endl;
        auto *closest = new Witness(siC_);
        closest->linkRep(node);
        si_->copyState(closest->state_, node->state_);
        witnesses_->add(closest);
        return closest;
    }
}

ompl::base::PlannerStatus ompl::control::ConstraintRespectingBSST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state_, st);
        siC_->nullControl(motion->control_);
        nn_->add(motion);
        motion->accCost_ = opt_->identityCost();
        findClosestWitness(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure\n", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state_;
    Control *rctrl = rmotion->control_;
    base::State *xstate = si_->allocState();

    unsigned iterations = 0;

    max_eigenvalue_ = 10.0; //TODO: make this general

    while (ptc == false && solution == nullptr)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);
        // if (si_->getStateSpace()->isCompound()) {
        //     rmotion->state_->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigmaX(rng_.uniform01() * max_eigenvalue_);
        //     rmotion->state_->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigmaY(rng_.uniform01() * max_eigenvalue_);
        // }
        // else {
        rmotion->state_->as<RealVectorBeliefSpace::StateType>()->sigma_(0, 0) = rng_.uniform01() * max_eigenvalue_;
        rmotion->state_->as<RealVectorBeliefSpace::StateType>()->sigma_(1, 1) = rng_.uniform01() * max_eigenvalue_;
        // }
        
        /* find closest state in the tree */
        Motion *nmotion = selectNode(rmotion);
        // std::cout << "nmotion: " << nmotion << std::endl;

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        // std::cout << "sample control" << std::endl;
        controlSampler_->sample(rctrl);
        // std::cout << "propagate" << std::endl;
        unsigned int cd = rng_.uniformInt(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
        unsigned int propCd = siC_->propagateWhileValid(nmotion->state_, rctrl, cd, rstate);
        // std::cout << "done propagating" << std::endl;
        if (propCd == cd)
        {
            base::Cost incCost = opt_->motionCost(nmotion->state_, rstate);
            // std::cout << incCost << std::endl;
            // std::cout << nmotion->accCost_ << std::endl;
            base::Cost cost = opt_->combineCosts(nmotion->accCost_, incCost);
            // std::cout << cost << std::endl;
            Witness *closestWitness = findClosestWitness(rmotion);

            // std::cout << "rmotion: " << rmotion << std::endl;
            // std::cout << "closestWitness->rep_: " << closestWitness->rep_ << std::endl;
            // std::cout << cost << std::endl;
            // std::cout << closestWitness->rep_->accCost_ << std::endl;
            // exit(-1);

            if (closestWitness->rep_ == rmotion || opt_->isCostBetterThan(cost, closestWitness->rep_->accCost_))
            {
                // std::cout << "in true" << std::endl;
                Motion *oldRep = closestWitness->rep_;
                /* create a motion */
                auto *motion = new Motion(siC_);
                motion->accCost_ = cost;
                motion->state_->as<RealVectorBeliefSpace::StateType>()->setCost(cost.value());
                si_->copyState(motion->state_, rmotion->state_);
                siC_->copyControl(motion->control_, rctrl);
                motion->steps_ = cd;
                motion->parent_ = nmotion;
                nmotion->numChildren_++;

                if (!constraints_.empty()) {
                    // get the path-segment from start ---> motion->state
                    
                    /* construct the solution path */
        			std::vector<Motion *> mpath;
        			Motion* mCpy = motion;
        			while (mCpy != nullptr)
        			{
        			    mpath.push_back(mCpy);
        			    mCpy = mCpy->parent_;
        			}
  
        			/* set the solution path */
        			oc::PathControl pathSegment(si_);
        			for (int i = mpath.size() - 1; i >= 0; --i)
        			    if (mpath[i]->parent_)
        			        pathSegment.append(mpath[i]->state_, mpath[i]->control_, mpath[i]->steps_ * siC_->getPropagationStepSize());
        			    else
        			        pathSegment.append(mpath[i]->state_);
                    pathSegment.interpolate();
                    // check the path-segment for constraint validation prior to adding motion
                    if (planValidator_->satisfiesConstraints(pathSegment, constraints_)) {
                        closestWitness->linkRep(motion);

                        nn_->add(motion);
                        // if (si_->getStateSpace()->isCompound()) {
                        //     if (motion->state_->as<ob::CompoundState>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(0,0) > max_eigenvalue_)
                        //     {
                        //         max_eigenvalue_ = motion->state_->as<ob::CompoundState>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(0,0);
                        //     }
                        //     else if (motion->state_->as<ob::CompoundState>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(1,1) > max_eigenvalue_)
                        //     {
                        //         max_eigenvalue_ = motion->state_->as<ob::CompoundState>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(1,1);
                        //     }
                        // }
                        // else {
                        if (motion->state_->as<RealVectorBeliefSpace::StateType>()->getCovariance()(0,0) > max_eigenvalue_)
                        {
                            max_eigenvalue_ = motion->state_->as<RealVectorBeliefSpace::StateType>()->getCovariance()(0,0);
                        }
                        else if (motion->state_->as<RealVectorBeliefSpace::StateType>()->getCovariance()(1,1) > max_eigenvalue_)
                        {
                            max_eigenvalue_ = motion->state_->as<RealVectorBeliefSpace::StateType>()->getCovariance()(1,1);
                        }
                        // }

                        double dist = 0.0;
                        bool solv = goal->isSatisfied(motion->state_, &dist);
                        if (solv && opt_->isCostBetterThan(motion->accCost_, prevSolutionCost_))
                        {
                            approxdif = dist;
                            solution = motion;

                            for (auto &i : prevSolution_)
                                if (i)
                                    si_->freeState(i);
                            prevSolution_.clear();
                            for (auto &prevSolutionControl : prevSolutionControls_)
                                if (prevSolutionControl)
                                    siC_->freeControl(prevSolutionControl);
                            prevSolutionControls_.clear();
                            prevSolutionSteps_.clear();

                            Motion *solTrav = solution;
                            while (solTrav->parent_ != nullptr)
                            {
                                prevSolution_.push_back(si_->cloneState(solTrav->state_));
                                prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                                prevSolutionSteps_.push_back(solTrav->steps_);
                                solTrav = solTrav->parent_;
                            }
                            prevSolution_.push_back(si_->cloneState(solTrav->state_));
                            prevSolutionCost_ = solution->accCost_;

                            OMPL_INFORM("Found solution with cost %.2f", solution->accCost_.value());
                            sufficientlyShort = opt_->isSatisfied(solution->accCost_);
                            if (sufficientlyShort)
                                break;
                        }
                        // removing the ability to return approximate solutions (HACK)
                        // To-Do: Is there a way to set this behavior inside pdef_?
                        // if (solution == nullptr && dist < approxdif)
                        // {
                        //     approxdif = dist;
                        //     approxsol = motion;

                        //     for (auto &i : prevSolution_)
                        //         if (i)
                        //             si_->freeState(i);
                        //     prevSolution_.clear();
                        //     for (auto &prevSolutionControl : prevSolutionControls_)
                        //         if (prevSolutionControl)
                        //             siC_->freeControl(prevSolutionControl);
                        //     prevSolutionControls_.clear();
                        //     prevSolutionSteps_.clear();

                        //     Motion *solTrav = approxsol;
                        //     while (solTrav->parent_ != nullptr)
                        //     {
                        //         prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        //         prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                        //         prevSolutionSteps_.push_back(solTrav->steps_);
                        //         solTrav = solTrav->parent_;
                        //     }
                        //     prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        // }

                        if (oldRep != rmotion)
                        {
                            while (oldRep->inactive_ && oldRep->numChildren_ == 0)
                            {
                                oldRep->inactive_ = true;
                                nn_->remove(oldRep);

                                if (oldRep->state_)
                                    si_->freeState(oldRep->state_);
                                if (oldRep->control_)
                                    siC_->freeControl(oldRep->control_);

                                oldRep->state_ = nullptr;
                                oldRep->control_ = nullptr;
                                oldRep->parent_->numChildren_--;
                                Motion *oldRepParent = oldRep->parent_;
                                delete oldRep;
                                oldRep = oldRepParent;
                            }
                        }
                    }
                }
                else {
                    // proceed as normal
                    closestWitness->linkRep(motion);

                    nn_->add(motion);

                    // if (DISTANCE_FUNC_ == 0){
                    //     if (motion->state_->as<RealVectorBeliefSpaceEuclidean::StateType>()->getCovariance()(0,0) > max_eigenvalue_)
                    //     {
                    //         max_eigenvalue_ = motion->state_->as<R2BeliefSpaceEuclidean::StateType>()->getCovariance()(0,0);
                    //     }
                    //     else if (motion->state_->as<R2BeliefSpaceEuclidean::StateType>()->getCovariance()(1,1) > max_eigenvalue_)
                    //     {
                    //         max_eigenvalue_ = motion->state_->as<R2BeliefSpaceEuclidean::StateType>()->getCovariance()(1,1);
                    //     }
                    // }
                    // else if (DISTANCE_FUNC_ == 1){
                    //     if (motion->state_->as<R2BeliefSpace::StateType>()->getCovariance()(0,0) > max_eigenvalue_)
                    //     {
                    //         max_eigenvalue_ = motion->state_->as<R2BeliefSpace::StateType>()->getCovariance()(0,0);
                    //     }
                    //     else if (motion->state_->as<R2BeliefSpace::StateType>()->getCovariance()(1,1) > max_eigenvalue_)
                    //     {
                    //         max_eigenvalue_ = motion->state_->as<R2BeliefSpace::StateType>()->getCovariance()(1,1);
                    //     }
                    // }
                    // if (si_->getStateSpace()->isCompound()) {
                    //     if (motion->state_->as<ob::CompoundState>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(0,0) > max_eigenvalue_)
                    //     {
                    //         max_eigenvalue_ = motion->state_->as<ob::CompoundState>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(0,0);
                    //     }
                    //     else if (motion->state_->as<ob::CompoundState>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(1,1) > max_eigenvalue_)
                    //     {
                    //         max_eigenvalue_ = motion->state_->as<ob::CompoundState>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(1,1);
                    //     }
                    // }
                    // else {
                    if (motion->state_->as<RealVectorBeliefSpace::StateType>()->getCovariance()(0,0) > max_eigenvalue_)
                    {
                        max_eigenvalue_ = motion->state_->as<RealVectorBeliefSpace::StateType>()->getCovariance()(0,0);
                    }
                    else if (motion->state_->as<RealVectorBeliefSpace::StateType>()->getCovariance()(1,1) > max_eigenvalue_)
                    {
                        max_eigenvalue_ = motion->state_->as<RealVectorBeliefSpace::StateType>()->getCovariance()(1,1);
                    }
                    // }

                    double dist = 0.0;
                    bool solv = goal->isSatisfied(motion->state_, &dist);
                    if (solv && opt_->isCostBetterThan(motion->accCost_, prevSolutionCost_))
                    {
                        approxdif = dist;
                        solution = motion;

                        for (auto &i : prevSolution_)
                            if (i)
                                si_->freeState(i);
                        prevSolution_.clear();
                        for (auto &prevSolutionControl : prevSolutionControls_)
                            if (prevSolutionControl)
                                siC_->freeControl(prevSolutionControl);
                        prevSolutionControls_.clear();
                        prevSolutionSteps_.clear();

                        Motion *solTrav = solution;
                        while (solTrav->parent_ != nullptr)
                        {
                            prevSolution_.push_back(si_->cloneState(solTrav->state_));
                            prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                            prevSolutionSteps_.push_back(solTrav->steps_);
                            solTrav = solTrav->parent_;
                        }
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        prevSolutionCost_ = solution->accCost_;

                        OMPL_INFORM("Found solution with cost %.2f", solution->accCost_.value());
                        sufficientlyShort = opt_->isSatisfied(solution->accCost_);
                        if (sufficientlyShort)
                            break;
                    }
                    // removing the ability to return approximate solutions (HACK)
                    // To-Do: Is there a way to set this behavior inside pdef_?
                    // if (solution == nullptr && dist < approxdif)
                    // {
                    //     approxdif = dist;
                    //     approxsol = motion;

                    //     for (auto &i : prevSolution_)
                    //         if (i)
                    //             si_->freeState(i);
                    //     prevSolution_.clear();
                    //     for (auto &prevSolutionControl : prevSolutionControls_)
                    //         if (prevSolutionControl)
                    //             siC_->freeControl(prevSolutionControl);
                    //     prevSolutionControls_.clear();
                    //     prevSolutionSteps_.clear();

                    //     Motion *solTrav = approxsol;
                    //     while (solTrav->parent_ != nullptr)
                    //     {
                    //         prevSolution_.push_back(si_->cloneState(solTrav->state_));
                    //         prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                    //         prevSolutionSteps_.push_back(solTrav->steps_);
                    //         solTrav = solTrav->parent_;
                    //     }
                    //     prevSolution_.push_back(si_->cloneState(solTrav->state_));
                    // }
                    if (oldRep != rmotion)
                    {
                        while (oldRep->inactive_ && oldRep->numChildren_ == 0)
                        {
                            oldRep->inactive_ = true;
                            nn_->remove(oldRep);
                            if (oldRep->state_)
                                si_->freeState(oldRep->state_);
                            if (oldRep->control_)
                                siC_->freeControl(oldRep->control_);
                            oldRep->state_ = nullptr;
                            oldRep->control_ = nullptr;
                            oldRep->parent_->numChildren_--;
                            Motion *oldRepParent = oldRep->parent_;
                            delete oldRep;
                            oldRep = oldRepParent;
                        }
                    }
                }
            }
        }
        iterations++;
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
        /* set the solution path */
        auto path(std::make_shared<oc::PathControl>(si_));
        for (int i = prevSolution_.size() - 1; i >= 1; --i)
            path->append(prevSolution_[i], prevSolutionControls_[i - 1],
                         prevSolutionSteps_[i - 1] * siC_->getPropagationStepSize());
        path->append(prevSolution_[0]);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    si_->freeState(xstate);
    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    if (rmotion->control_)
        siC_->freeControl(rmotion->control_);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states in %u iterations", getName().c_str(), nn_->size(), iterations);

    return {solved, approximate};
}

void ompl::control::ConstraintRespectingBSST::getPlannerData(base::PlannerData &data) const
{
    ConstraintRespectingPlanner::getPlannerData(data);

    std::vector<Motion *> motions;
    std::vector<Motion *> allMotions;
    if (nn_)
        nn_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->numChildren_ == 0)
        {
            allMotions.push_back(motion);
        }
    }
    for (unsigned i = 0; i < allMotions.size(); i++)
    {
        if (allMotions[i]->parent_ != nullptr)
        {
            allMotions.push_back(allMotions[i]->parent_);
        }
    }

    double delta = siC_->getPropagationStepSize();

    if (prevSolution_.size() != 0)
        data.addGoalVertex(base::PlannerDataVertex(prevSolution_[0]));

    for (auto m : allMotions)
    {
        if (m->parent_)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent_->state_), base::PlannerDataVertex(m->state_),
                             control::PlannerDataEdgeControl(m->control_, m->steps_ * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent_->state_), base::PlannerDataVertex(m->state_));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state_));
    }
}

void ompl::control::ConstraintRespectingBSST::getPlannerDataAndCosts(base::PlannerData &data, std::vector<double> &costs) const
{
    ConstraintRespectingBSST::getPlannerData(data);

    std::vector<Motion *> motions;
    std::vector<Motion *> allMotions;
    if (nn_)
        nn_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->numChildren_ == 0)
        {
            allMotions.push_back(motion);
        }
    }
    for (unsigned i = 0; i < allMotions.size(); i++)
    {
        costs.push_back(allMotions[i]->accCost_.value());
        if (allMotions[i]->parent_ != nullptr)
        {
            allMotions.push_back(allMotions[i]->parent_);
        }
    }

    double delta = siC_->getPropagationStepSize();

    if (prevSolution_.size() != 0)
        data.addGoalVertex(base::PlannerDataVertex(prevSolution_[0]));

    for (auto m : allMotions)
    {
        if (m->parent_)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent_->state_), base::PlannerDataVertex(m->state_),
                             control::PlannerDataEdgeControl(m->control_, m->steps_ * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent_->state_), base::PlannerDataVertex(m->state_));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state_));
    }
}

#pragma once
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/R2BeliefSpaceEuclidean.h"
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/control/planners/PlannerIncludes.h>

namespace oc = ompl::control;


namespace ompl
{
    namespace control
    {
        class BSST : public base::Planner
        {
        public:
            /** \brief Constructor */
            BSST(const SpaceInformationPtr &si);

            ~BSST() override;

            void setup() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void getPlannerData(base::PlannerData &data) const override;

            void getPlannerDataAndCosts(base::PlannerData &data, std::vector<double > &costs) const;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /**
                \brief Set the radius for selecting nodes relative to random sample.
                This radius is used to mimic behavior of RRT* in that it promotes
                extending from nodes with good path cost from the root of the tree.
                Making this radius larger will provide higher quality paths, but has two
                major drawbacks; exploration will occur much more slowly and exploration
                around the boundary of the state space may become impossible. */
            void setSelectionRadius(double selectionRadius)
            {
                selectionRadius_ = selectionRadius;
            }

            /** \brief Get the selection radius the planner is using */
            double getSelectionRadius() const
            {
                return selectionRadius_;
            }

            /**
                \brief Set the radius for pruning nodes.
                This is the radius used to surround nodes in the witness set.
                Within this radius around a state in the witness set, only one
                active tree node can exist. This limits the size of the tree and
                forces computation to focus on low path costs nodes. If this value
                is too large, narrow passages will be impossible to traverse. In addition,
                children nodes may be removed if they are not at least this distance away
                from their parent nodes.*/
            void setPruningRadius(double pruningRadius)
            {
                pruningRadius_ = pruningRadius;
            }

            /** \brief Get the pruning radius the planner is using */
            double getPruningRadius() const
            {
                return pruningRadius_;
            }

            /** TODO: BRIEF FOR THIS */
            void setSamplingBias(double samplingBias)
            {
                samplingBias_ = samplingBias;
            }

            /** \brief Get the sampling bias the planner is using */
            double getSamplingBias() const
            {
                return samplingBias_;
            }

            void setDistanceFunction(int distfunc)
            {
                DISTANCE_FUNC_ = distfunc;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                witnesses_ = std::make_shared<NN<Motion *>>();
                setup();
            }

        protected:
            /** \brief Representation of a motion
                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state_(si->allocState()), control_(si->allocControl())
                {
                }

                virtual ~Motion() = default;

                virtual base::State *getState() const
                {
                    return state_;
                }
                virtual Motion *getParent() const
                {
                    return parent_;
                }

                base::Cost accCost_{0};

                /** \brief The state contained by the motion */
                base::State *state_{nullptr};

                /** \brief The control contained by the motion */
                Control *control_{nullptr};

                /** \brief The number of steps_ the control is applied for */
                unsigned int steps_{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent_{nullptr};

                /** \brief Number of children */
                unsigned numChildren_{0};

                /** \brief If inactive, this node is not considered for selection.*/
                bool inactive_{false};
            };

            class Witness : public Motion
            {
            public:
                Witness() = default;

                Witness(const SpaceInformation *si) : Motion(si)
                {
                }
                base::State *getState() const override
                {
                    return rep_->state_;
                }
                Motion *getParent() const override
                {
                    return rep_->parent_;
                }

                void linkRep(Motion *lRep)
                {
                    rep_ = lRep;
                }

                /** \brief The node in the tree that is within the pruning radius.*/
                Motion *rep_{nullptr};
            };

            /** \brief Finds the best node in the tree withing the selection radius around a random sample.*/
            Motion *selectNode(Motion *sample);

            /** \brief Find the closest witness node to a newly generated potential node.*/
            Witness *findClosestWitness(Motion *node);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state_, b->state_);
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            ControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief A nearest-neighbors datastructure containing the tree of witness motions */
            std::shared_ptr<NearestNeighbors<Motion *>> witnesses_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief The radius for determining the node selected for extension. */
            double selectionRadius_{0.2};

            /** \brief The radius for determining the size of the pruning region. */
            double pruningRadius_{0.1};

            double samplingBias_{0.50};

            int DISTANCE_FUNC_{1}; //1 is 

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The best solution we found so far. */
            std::vector<base::State *> prevSolution_;
            std::vector<Control *> prevSolutionControls_;
            std::vector<unsigned> prevSolutionSteps_;

            /** \brief The best solution cost we found so far. */
            base::Cost prevSolutionCost_;

            /** \brief The optimization objective. */
            base::OptimizationObjectivePtr opt_;

            double max_eigenvalue_;
        };
    }
}

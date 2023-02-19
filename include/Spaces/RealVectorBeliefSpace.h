#pragma once
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/math/constants/constants.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using namespace ompl::base;


class RealVectorBeliefSpace : public ompl::base::RealVectorStateSpace
{
    public:
        RealVectorBeliefSpace(double dim): RealVectorStateSpace(dim), dimension_(dim)
        {
            setName("RealVectorBeliefSpace" + getName());
            type_ = STATE_SPACE_REAL_VECTOR;
            // sigma_init_ = sigma_init;
        }

        /** \brief A belief in R(n): ([x, y, ...], covariance(n x n)) */
        class StateType: public RealVectorStateSpace::StateType
        {
        public:
            // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            StateType(): RealVectorStateSpace::StateType() {}

            // const Eigen::MatrixXd getSigma(void) const
            // {
            //     return sigma_;
            // }

            // const Eigen::MatrixXd getLambda(void) const
            // {
            //     return lambda_;
            // }

            const Eigen::MatrixXd getCovariance(void) const
            {
                return sigma_ + lambda_;
            } 

            // void setMatrixData(const Eigen::Vector2d &x)
            // {
            //     setX(x[0]);
            //     setY(x[1]);
            // }

            // void setSigmaX(double val){
            //     sigma_(0,0) = val;
            // }

            // void setSigmaY(double val){
            //     sigma_(1,1) = val;
            // }

            // void setSigma(Eigen::MatrixXd cov){
            //     sigma_ = cov;
            // }

            // void setSigma(double val){
            //     sigma_ = val*Eigen::MatrixXd::Identity(2,2);
            // }

            // void setLambda(Eigen::Matrix2d cov){
            //     lambda_ = cov;
            // }

            // Eigen::Vector2d getMatrixData(void) const
            // {
            //     Eigen::Vector2d stateVec(getX(), getY());
            //     return stateVec;
            // }

            // void setCost(double cost){
            //     cost_ = cost;
            // }

            // double getCost(void) const{ 
            //     return cost_;
            // }

            /** \brief Checks if the input state has stabilized to this state (node reachability check) */
            // bool isReached(ompl::base::State *state, bool relaxedConstraint=false) const;

            // static double meanNormWeight_, covNormWeight_, reachDist_;

            // int DISTANCE_FUNCTION_TYPE_;
            
        // private:
            Eigen::MatrixXd sigma_;
            Eigen::MatrixXd lambda_;
        };

        virtual ~RealVectorBeliefSpace(void) {}

        // virtual State* allocState(void) const override;
        ompl::base::State* allocState(void) const override;

        void copyState(State *destination,const State *source) const override;

        void freeState(State *state) const override;

        //virtual void registerProjections(void);
        double distance(const State* state1, const State *state2) const override;

        // gets the relative vector between "from" and "to"
        // equivalent to result = vectorA-vectorB
        // void getRelativeState(const State *from, const State *to, State *state);

        // void printBeliefState(const State *state);

    protected:
        const unsigned int dimension_;
        // double sigma_init_;

};

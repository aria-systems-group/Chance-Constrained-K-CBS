#pragma once
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/math/constants/constants.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Dense>


using namespace ompl::base;
class R2BeliefSpace : public ompl::base::RealVectorStateSpace
{

    public:

        /** \brief A belief in R(2): (x, y, covariance) */
        class StateType : public RealVectorStateSpace::StateType
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            StateType(double sigma_init) : RealVectorStateSpace::StateType()
            {
              
              sigma_ = sigma_init*Eigen::MatrixXd::Identity(2,2);
              this->setLambda(Eigen::Matrix2d::Zero());

            }

            /** \brief Get the X component of the state */
            double getX(void) const
            {
                return this->values[0];
            }

            /** \brief Get the Y component of the state */
            double getY(void) const
            {
                return this->values[1];
            }

            const Eigen::Vector2d getXY(void) const
            {
                const Eigen::Vector2d stateVec(getX(), getY());
                return stateVec;
            }

            Eigen::Matrix2d getSigma(void) const
            {    
                return sigma_;
            }

            Eigen::Matrix2d getLambda(void) const
            {
                return lambda_;
            }

            Eigen::Matrix2d getCovariance(void) const
            {
                return sigma_ + lambda_;
            }

            /** \brief Set the X~ component of the state */
            void setX(double x)
            {
                this->values[0] = x;
            }

            /** \brief Set the Y component of the state */
            void setY(double y)
            {
                this->values[1] = y;
            }

            /** \brief Set the X and Y components of the state */
            void setXY(double x, double y)
            {
                setX(x);
                setY(y);
            }    

            void setMatrixData(const Eigen::Vector2d &x)
            {
                setX(x[0]);
                setY(x[1]);
            }

            void setSigmaX(double val){
                sigma_(0,0) = val;
            }

            void setSigmaY(double val){
                sigma_(1,1) = val;
            }

            void setSigma(Eigen::Matrix2d cov){
                sigma_ = cov;
            }

            void setSigma(double val){
                sigma_ = val*Eigen::MatrixXd::Identity(2,2);
            }

            void setLambda(Eigen::Matrix2d cov){
                lambda_ = cov;
            }

            Eigen::Vector2d getMatrixData(void) const
            {
                Eigen::Vector2d stateVec(getX(), getY());
                return stateVec;
            }

            void setCost(double cost){
                cost_ = cost;
            }

            double getCost(void) const{ 
                return cost_;
            }

            /** \brief Checks if the input state has stabilized to this state (node reachability check) */
            bool isReached(ompl::base::State *state, bool relaxedConstraint=false) const;

            static double meanNormWeight_, covNormWeight_, reachDist_;

            int DISTANCE_FUNCTION_TYPE_;
            
        private:
              Eigen::Matrix2d sigma_;
              Eigen::Matrix2d lambda_;
              int dimensions_ = 2;
              double cost_;

        };

        R2BeliefSpace(double sigma_init) : RealVectorStateSpace(2)
        {
            setName("R2_BELIEF" + getName());
            type_ = STATE_SPACE_REAL_VECTOR;
            sigma_init_ = sigma_init;
        }

        virtual ~R2BeliefSpace(void)
        {
        }

        virtual State* allocState(void) const override;

        virtual void copyState(State *destination,const State *source) const override;

        virtual void freeState(State *state) const override;

        //virtual void registerProjections(void);
        virtual double distance(const State* state1, const State *state2) const override;

        // gets the relative vector between "from" and "to"
        // equivalent to result = vectorA-vectorB
        void getRelativeState(const State *from, const State *to, State *state);

        void printBeliefState(const State *state);

        protected:
            double sigma_init_;

};

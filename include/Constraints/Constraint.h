#pragma once
#include <ompl/util/ClassForward.h>


// abstract class for constraint
OMPL_CLASS_FORWARD(Constraint);
class Constraint
{
public:
	Constraint(int agentIdx, std::vector<double> timeRange):
		times_(timeRange), agentIdx_(agentIdx) {}
	~Constraint()
	{
		times_.clear();
	}
	const std::vector<double> getTimes() const {return times_;};
	const int getAgent() const {return agentIdx_;};

	/** \brief Cast this instance to a desired type. */
    template <class T>
    T *as()
    {
        /** \brief Make sure the type we are casting to is indeed a Constraint */
        BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Constraint *>));
        return static_cast<T *>(this);
    }

    /** \brief Cast this instance to a desired type. */
    template <class T>
    const T *as() const
    {
        /** \brief Make sure the type we are casting to is indeed a Constraint */
        BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Constraint *>));
        return static_cast<const T *>(this);
    }
protected:
	std::vector<double> times_;
	const int agentIdx_;
};

#pragma once
#include <vector>
#include <ompl/util/ClassForward.h>
#include <boost/concept_check.hpp>


// abstract class for constraint
OMPL_CLASS_FORWARD(Constraint);
class Constraint
{
public:
	Constraint(int constrained_agent, int constraining_agent, std::vector<double> timeRange):
		times_(timeRange), constrained_agent_(constrained_agent), constraining_agent_(constraining_agent) {}
	~Constraint()
	{
		times_.clear();
	}

	const std::vector<double> getTimes() const {return times_;};
	const int getConstrainedAgent() const {return constrained_agent_;};
	const int getConstrainingAgent() const {return constraining_agent_;};

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
	const int constrained_agent_;
	const int constraining_agent_;
};

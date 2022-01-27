/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file holds all of the structures for defining, creating, 
* and storing Constraints. 
*********************************************************************/

/* Author: Justin Kottinger */

#ifndef CONFLICT_
#define CONFLICT_

#include <boost/geometry.hpp>
// #include <unordered_set>
#include <iostream>

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

/* A conflict occurs when two agents' shapes collide. 
Their agent indices, thier shapes, and time are stored */
struct Conflict
{
	const int agent1; 
	const int agent2;
	polygon p1;
	polygon p2;
	const double time;

	bool operator==(const Conflict &other) const;

	void print() const;

	void printP1() const;

	void printP2() const;

	polygon::ring_type getP1() const;

	polygon::ring_type getP2() const;
};
#endif

#ifndef CONSTRAINT_
#define CONSTRAINT_
/* A conflict occurs an infinite number of times per
collision. However, this range is minimally discretized 
within OMPL through control durrations. Thus, each 
collision creates a finite number of conflicts and a time range. 
The constraint is the set of conflicting polygons for an agent
within the given time range */
class Constraint
{

public:
	Constraint(const std::vector <const polygon> polygons, 
		const std::vector<double> timeRange, 
		const int agentIdx) : polys_(polygons), times_(timeRange), agentIdx_(agentIdx) {}
	const std::vector <const polygon> getPolygons() const {return polys_;};
	const std::vector<double> getTimes() const {return times_;};
	const int getAgent() const {return agentIdx_;};
private:
	const std::vector <const polygon> polys_;
	const std::vector<double> times_;
	const int agentIdx_;
};
#endif




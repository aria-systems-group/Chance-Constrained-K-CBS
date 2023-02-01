#include "Constraint.h"


bool Conflict::operator==(const Conflict &other) const
{ 
    return (agent1 == other.agent1 && 
    		agent2 == other.agent2 &&
    		boost::geometry::equals(p1, other.p1) &&
    		boost::geometry::equals(p2, other.p2) &&
    		time == other.time);
};

void Conflict::print() const
{
	std::cout << time << ": (" << agent1 << ", " << agent2 << ")" << std::endl;
};

void Conflict::printP1() const
{
    auto exterior_points = boost::geometry::exterior_ring(p1);
    for (int i=0; i<exterior_points.size(); i++)
    {
        std::cout << boost::geometry::get<0>(exterior_points[i]) << ", " << boost::geometry::get<1>(exterior_points[i]) <<std::endl;
    }
};

void Conflict::printP2() const
{
    auto exterior_points = boost::geometry::exterior_ring(p2);
    for (int i=0; i<exterior_points.size(); i++)
    {
        std::cout << boost::geometry::get<0>(exterior_points[i]) << ", " << boost::geometry::get<1>(exterior_points[i]) <<std::endl;
    }
};

polygon::ring_type Conflict::getP1() const
{
    return boost::geometry::exterior_ring(p1);
};

polygon::ring_type Conflict::getP2() const
{
    return boost::geometry::exterior_ring(p1);
};
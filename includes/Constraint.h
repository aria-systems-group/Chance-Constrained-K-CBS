/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* This file holds all of the structures for defining, creating, 
* and storing Constraints. 
*********************************************************************/

/* Author: Justin Kottinger */

#include <boost/geometry.hpp>
#include <unordered_set>

/* A conflict occurs when two agents' shapes collide. 
Their agent indices, thier shapes, and time are stored */
struct Conflict
{
	int agent1; 
	int agent2;
	polygon p1;
	polygon p2;
	double time;

	bool operator==(const Conflict &other) const
    { 
        return (agent1 == other.agent1 && 
        		agent2 == other.agent2 &&
        		boost::geometry::equals(p1, other.p1) &&
        		boost::geometry::equals(p2, other.p2) &&
        		time == other.time);
    }

    void printP1() const
    {
        auto exterior_points = boost::geometry::exterior_ring(p1);
        for (int i=0; i<exterior_points.size(); i++)
        {
            std::cout << boost::geometry::get<0>(exterior_points[i]) << ", " << boost::geometry::get<1>(exterior_points[i]) <<std::endl;
        }
    }
    void printP2() const
    {
        auto exterior_points = boost::geometry::exterior_ring(p2);
        for (int i=0; i<exterior_points.size(); i++)
        {
            std::cout << boost::geometry::get<0>(exterior_points[i]) << ", " << boost::geometry::get<1>(exterior_points[i]) <<std::endl;
        }
    }
    polygon::ring_type getP1() const
    {
        return boost::geometry::exterior_ring(p1);
    }

    polygon::ring_type getP2() const
    {
        return boost::geometry::exterior_ring(p1);
    }
};

// custom hash function for Conflict structure
namespace std
{
	template <>
	struct hash<polygon>
	{
		size_t operator()(const polygon& p) const
		{
			auto exterior_points = boost::geometry::exterior_ring(p);
			// it is assumed that polygon object is a rectangle,
			// contains 5 points and idx 0 == idx 4
			double point_1_x = boost::geometry::get<0>(exterior_points[0]);
			double point_1_y = boost::geometry::get<1>(exterior_points[0]);
			double point_2_x = boost::geometry::get<0>(exterior_points[1]);
			double point_2_y = boost::geometry::get<1>(exterior_points[1]);
			double point_3_x = boost::geometry::get<0>(exterior_points[2]);
			double point_3_y = boost::geometry::get<1>(exterior_points[2]);
			double point_4_x = boost::geometry::get<0>(exterior_points[3]);
			double point_4_y = boost::geometry::get<1>(exterior_points[3]);

			return ((hash<double>()(point_1_x) ^ 
            		(hash<double>()(point_1_y) ^ 
            		(hash<double>()(point_2_x) ^
            		(hash<double>()(point_2_y) ^
            		(hash<double>()(point_3_x) ^
            		(hash<double>()(point_3_y) ^
            		(hash<double>()(point_4_x) ^
            		(hash<double>()(point_4_y) << 1)))))))) >> 1);
		}
	};
    template <>
    struct hash<Conflict>
    {
        size_t operator()(const Conflict& k) const
        {
            // Compute individual hash values for all data members and combine them using XOR and bit shifting
            return ((hash<int>()(k.agent1) ^ 
            		(hash<int>()(k.agent2) ^ 
            		(hash<polygon>()(k.p1) ^
            		(hash<polygon>()(k.p2) ^
            		(hash<double>()(k.time) << 1))))) >> 1);
        }
    };
}

/* A conflict occurs an infinite number of times per
collision. However, this range is minimally discretized 
within OMPL through control durrations. Thus, each 
collision creates a finite number of conflicts and a time range. 
The constraint is the minimum bounding box of the conflicts, 
and the time range [t_min, t_max]. */
// see https://stackoverflow.com/questions/62741147/combining-two-polygons-into-one-polygon-in-boost-geometry-exterior-points-only
// for creating minimum bounding box.
class Constraint
{
public:
	Constraint(std::unordered_set <Conflict> c, int agent)
	{
		c_ = c;
		agentIdx_ = agent;
	}
private:
	std::unordered_set <Conflict> c_;
	int agentIdx_;
};





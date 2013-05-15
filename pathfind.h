/*****************************************************************************
 *       ME132b Lab 1 - Pioneer Wall Follower Pathfind Constants, etc
 *           Matthew Dughi, Tiffany Huang, Gregory Izatt
 *****************************************************************************/

#ifndef H_PATHFIND
#define H_PATHFIND

bool figure_out_movement(double * speed, double * turnrate,
    vector<double> range_data, vector<double> bearing_data, unsigned int n,
    double * pose, SimpleOccupancyGrid& oc, bool use_vector_field);
    
#define MIN_TURN_RATE 0.3
#define MAX_TURN_RATE 0.5
#define SPEED_EPS 0.01
#define ANGLE_EPS 0.01
#define MIN_WALL_DIST 0.1
#define MAX_SPEED 0.5
    
// This object stores the (x,y,theta) pose of a robot
struct Pose
{
    double x;
    double y;
    double theta;
    Pose() : x(0), y(0), theta(0)  {}
    Pose(const double x, const double y, const double theta) : x(x), y(y), theta(theta) {}
};

// This object stores the (x,y) coordinates of a point
struct Point
{
    double x;
    double y;
    Point() : x(0), y(0)  {}
    Point(const double x, const double y) : x(x), y(y)  {}
    double distance_to(const Point p2) const
    {
        return distance_to(p2.x, p2.y);
    }
    double distance_to(const double x2, const double y2) const
    {
        double dx = x - x2;
        double dy = y - y2;
        return sqrt(dx * dx + dy * dy);
    }
    double distance_to(const Pose p) const
    {
        return distance_to(p.x, p.y);
    }
    // This function returns the angle between two points.
    double angle_to(const double x2, const double y2) const
    {
        return atan((y - y2)/(x - x2));
    }
    // This function returns the angle between two points. When theta2 is given,
    // it interprets the point given as the location of the robot and gives the
    // relative angle toward *this.
    double angle_to(const double x2, const double y2, const double theta2) const
    {
        double dtheta = angle_to(x2, y2) - theta2;
        dtheta -= PI * (x2 > x);  // Correct on left half of plane
        dtheta += ((dtheta < -PI) - (dtheta > PI)) * 2 * PI; // Bound on [-PI, PI]
        return dtheta;
    }
    double angle_to(const Pose p) const
    {
        return angle_to(p.x, p.y, p.theta);
    }
};

#endif

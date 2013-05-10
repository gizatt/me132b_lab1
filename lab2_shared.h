// This file contains a struct that holds laser scan data

// Include guard
#ifndef __LAB2_SHARED__
#define __LAB2_SHARED__

// General include files
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>

#define PI 3.14159265358979323846
#define MIN_TURN_RATE 0.3
#define SPEED_EPS 0.01
#define ANGLE_EPS 0.01
#define MIN_WALL_DIST 0.1
#define MAX_SPEED 0.3

// Include namespaces for convenience in code writing
using namespace std;

// This object stores the (x,y,theta) pose of a robot
struct Pose
{
    double x;
    double y;
    double theta;
    Pose() : x(0), y(0), theta(0)  {}
    Pose(const double x, const double y, const double theta) : x(x), y(y), theta(theta) {}
};
// This function prints out a Pose as x, y, theta
ostream& operator<<(ostream& os, const Pose& p)
{
    os << p.x << ", " << p.y << ", " << p.theta;
    return os;
}
// This function reads in a Pose given as x, y, theta
istream& operator>>(istream& s, Pose& p)
{
    double x = 0, y = 0, theta = 0;   // initialize components of vector to read in.
    char ch = 0;                    // characters read in from string

    if (!s)     {return s;}     // stream invalid, just return fail state

    // Read in each component and return if invalid syntax occurs
    s >> x >> ch;
    if (ch != ',') {s.clear(ios_base::failbit); return s;}  // no ',' between num
    s >> y >> ch;
    if (ch != ',') {s.clear(ios_base::failbit); return s;}  // no ',' between num
    s >> theta >> ch;

    // Everything valid, create Color
    p = Pose(x, y, theta);

    return s;
}

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
// This function prints out a Point as [x, y]
ostream& operator<<(ostream& os, const Point& p)
{
    os << "[" << p.x << ", " << p.y << "]";
    return os;
}
// This function reads in a Point given as 2 doubles separated by whitespace
// It is assumed that there is a new line separator at the end
istream& operator>>(istream& s, Point& p)
{
    // initialize components of Point to read in
    double x = 0, y = 0;
    if (!s)     {return s;}     // stream invalid, just return fail state
    s >> x >> y;                // Get data from stream
    p = Point(x, y);            // Everything valid, create DoublePoint
    return s;                   // Return stream
}

// This object stores the (x,y) coordinates of two paired points
struct LaserData
{
    double range;
    double bearing;
    LaserData() : range(0), bearing(0)  {}
    LaserData(const double r, const double b) : range(r), bearing(b)    {}
};
// This function prints out a LaserData as range, bearing
ostream& operator<<(ostream& os, const LaserData& ld)
{
    os << ld.range << ", " << ld.bearing;
    return os;
}
// This function reads in a LaserData given as range, bearing
istream& operator>>(istream& s, LaserData& dp)
{
    double range = 0, bearing = 0;  // initialize components of vector to read in.
    char ch = 0;                    // characters read in from string

    if (!s)     {return s;}     // stream invalid, just return fail state

    // Read in each component and return if invalid syntax occurs
    s >> range >> ch;
    if (ch != ',') {s.clear(ios_base::failbit); return s;}  // no ',' between num
    s >> bearing >> ch;

    // Everything valid, create Color
    dp = LaserData(range, bearing);

    return s;
}

// This function implements a go to function based on inputs of where the robot
// destination is and the current pose. It returns the direction and speed to go
// by reference.
int go_to_point(double goal_x, double goal_y,
                double robot_x, double robot_y, double robot_theta,
                double* r_dot, double* theta_dot)
{
    Point goal = Point(goal_x, goal_y); // Turn into point for convenience
    // Get distance and direction to goal
    double dr = goal.distance_to(robot_x, robot_y);
    double dtheta = goal.angle_to(robot_x, robot_y, robot_theta);
    // Don't move if more than 22.5 degrees off, otherwise scale speed with log
    // of angle but cap at dr to prevent overshoot
    *r_dot = (abs(dtheta) < PI/8) * dr * -log(abs(dtheta)) / 10.0;
    *r_dot = (dr < *r_dot) ? dr : *r_dot;
    *r_dot = (MAX_SPEED < *r_dot) ? MAX_SPEED : *r_dot;
    // Turn angle is simply direction offset
    *theta_dot = dtheta / 2;    // Slow rotation to prevent overshoot
    
    // Check that the turn rate is greater than the minimum if turning in place
    if (abs(*theta_dot) < MIN_TURN_RATE && abs(*r_dot) < SPEED_EPS &&
        abs(*theta_dot) > ANGLE_EPS)
    {
        // Set to min turn rate if below
        *theta_dot = MIN_TURN_RATE * ((*theta_dot > 0.0) * 2 - 1);
    }
    return 0;   // Dummy return value
}

// This function returns whether the robot is less than dist_thres meters away
// from an object.
bool next_to_wall(const vector<LaserData> &laser_data, double dist_thres)
{
    // Search laser data for range less that threshold
    for (vector<LaserData>::const_iterator it = laser_data.begin(); it != laser_data.end(); it++)
    {
        // If an object closer than threshold value, wall nearby
        if ((*it).range < dist_thres)
        {
            return true;
        }
    }
    return false;
}

// Read in points from a given filename
vector<Point> read_points(string filename = "goal_points.txt")
{
    ifstream in_file(filename.c_str());
    if (!in_file)
    {
        cerr << "Error: Couldn't open " << filename.c_str() << endl;
        exit(-1);
    }
    // Read in all the points from the feature list
    vector<Point> p_list;
    std::stringstream s;
    s << in_file.rdbuf();
    while (!(s.rdstate() & ios_base::eofbit))
    {
        Point dp;
        s >> dp;
        p_list.push_back(dp);
    }
    p_list.pop_back(); // Account for extra newline in data file
    in_file.close();
    return p_list;
}

#endif // __LAB2_SHARED__

#ifndef CATKIN_DRUHE_ZADANIE_GEOMETRY_MSGS_EXTENSIONS_H
#define CATKIN_DRUHE_ZADANIE_GEOMETRY_MSGS_EXTENSIONS_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
using namespace geometry_msgs;

namespace geometry_msgs::msg
{
    auto make_point(double x = 0.0, double y = 0.0, double z = 0.0)
    {
        Point p;
        p.x = x; p.y = y; p.z = z;
        return p;
    }
    auto make_quaternion(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0)
    {
        Quaternion q;
        q.x = x; q.y = y; q.z = z; q.w = w;
        return q;
    }

    auto make_pose(Point p, Quaternion q)
    {
        Pose pose;
        pose.position = p;
        pose.orientation = q;
        return pose;
    }
}

#endif //CATKIN_DRUHE_ZADANIE_GEOMETRY_MSGS_EXTENSIONS_H
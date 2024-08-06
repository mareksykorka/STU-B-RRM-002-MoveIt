#ifndef CATKIN_DRUHE_ZADANIE_PLAN_INTERFACE_H
#define CATKIN_DRUHE_ZADANIE_PLAN_INTERFACE_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
namespace rvt = rviz_visual_tools;

class PlanInterface
{
    public:
        PlanInterface(moveit::planning_interface::MoveGroupInterface* mgi,
                      const moveit::core::JointModelGroup* jmg,
                      moveit_visual_tools::MoveItVisualTools* vt);
        moveit::planning_interface::MoveGroupInterface::Plan planTarget(std::string targetName);
        moveit::planning_interface::MoveGroupInterface::Plan planPTP(geometry_msgs::Pose pose);
        moveit_msgs::RobotTrajectory planCartesian(std::vector<geometry_msgs::Pose> waypoints);
    private:
        moveit::planning_interface::MoveGroupInterface* move_group_interface_;
        const moveit::core::JointModelGroup* joint_model_group_;
        moveit_visual_tools::MoveItVisualTools* visual_tools_;
};

#endif //CATKIN_DRUHE_ZADANIE_PLAN_INTERFACE_H

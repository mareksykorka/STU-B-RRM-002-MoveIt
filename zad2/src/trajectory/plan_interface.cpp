#include "zad2/trajectory/plan_interface.h"

PlanInterface::PlanInterface(moveit::planning_interface::MoveGroupInterface* mgi,
                             const moveit::core::JointModelGroup* jmg,
                             moveit_visual_tools::MoveItVisualTools* vt)
{
    this->move_group_interface_ = mgi;
    this->joint_model_group_ = jmg;
    this->visual_tools_ = vt;
}

moveit::planning_interface::MoveGroupInterface::Plan PlanInterface::planTarget(std::string targetName)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_interface_->setNamedTarget(targetName);
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Target-plan", "Planing of the movement to '%s' %s", targetName.c_str(), success ? "" : "FAILED");
    visual_tools_->deleteAllMarkers();
    visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group_);
    visual_tools_->trigger();
    return plan;
}

moveit::planning_interface::MoveGroupInterface::Plan PlanInterface::planPTP(geometry_msgs::Pose pose)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_interface_->setPoseTarget(pose);
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("PTP-plan", "Planing of the movement to\np:[%.2f %.2f %.2f]\nq:[%.2f %.2f %.2f %.2f]\nstatus:%s",
                                pose.position.x, pose.position.y, pose.position.z,
                                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                                success ? "" : "FAILED");
    visual_tools_->deleteAllMarkers();
    visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group_);
    visual_tools_->trigger();
    return plan;
}

moveit_msgs::RobotTrajectory PlanInterface::planCartesian(std::vector<geometry_msgs::Pose> waypoints)
{
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_interface_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    ROS_INFO_NAMED("Cartesian-plan", "Planing of the Cartesian movement");
    visual_tools_->deleteAllMarkers();
    visual_tools_->publishTrajectoryLine(trajectory, joint_model_group_);
    visual_tools_->publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools_->trigger();
    return trajectory;
}
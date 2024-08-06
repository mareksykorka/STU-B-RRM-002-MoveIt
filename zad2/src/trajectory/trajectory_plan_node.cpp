#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "plan_interface.h"
#include "geometry_msgs_extensions.h"
using namespace geometry_msgs::msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_plan_node");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    #pragma region Premenne pre planovanie trajektorii a pohyb
        static const std::string PLANNING_GROUP = "abb_arm";
        moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit_msgs::RobotTrajectory trajectory;
    #pragma endregion

    #pragma region Premenne pre vizualizaciu
        moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
        visual_tools.loadRemoteControl();
    #pragma endregion

    PlanInterface moveInterface(&move_group_interface, joint_model_group, &visual_tools);

    #pragma region JOINT-STATE pohyb HOME
        visual_tools.prompt("Press 'next' for PLANNING.");
        plan = moveInterface.planTarget("home");
        visual_tools.prompt("Press 'next' for EXECUTION.");
        move_group_interface.execute(plan);
    #pragma endregion

    #pragma region JOINT-STATE pohyb POSE1
        visual_tools.prompt("Press 'next' for PLANNING.");
        plan = moveInterface.planPTP(make_pose(make_point(1.63244,0.676192, 1.0324), make_quaternion(-0.000771819,0.825926,0.000722185,0.563777)));
        visual_tools.prompt("Press 'next' for EXECUTION.");
        move_group_interface.execute(plan);
    #pragma endregion

    #pragma region CARTESIAN pohyb
        visual_tools.prompt("Press 'next' for PLANNING.");
        std::vector<geometry_msgs::Pose> waypoints1;
        waypoints1.push_back(make_pose(make_point(1.63244,0.676192, 1.0324), make_quaternion(-0.000771819,0.825926,0.000722185,0.563777)));
        waypoints1.push_back(make_pose(make_point(1.66138,0.182614, 1.10642), make_quaternion(-0.000777007,0.825964,0.00088843,0.563721)));
        waypoints1.push_back(make_pose(make_point(1.59476,-0.475037, 0.900661), make_quaternion(-0.000740694,0.825837,0.000781059,0.563908)));
        trajectory = moveInterface.planCartesian(waypoints1);
        visual_tools.prompt("Press 'next' for EXECUTION.");
        move_group_interface.execute(trajectory);
    #pragma endregion

    #pragma region JOINT-STATE pohyb HOME
        visual_tools.prompt("Press 'next' for PLANNING.");
        plan = moveInterface.planTarget("home");
        visual_tools.prompt("Press 'next' for EXECUTION.");
        move_group_interface.execute(plan);
    #pragma endregion

    #pragma region JOINT-STATE pohyb POSE2
        visual_tools.prompt("Press 'next' for PLANNING.");
        plan = moveInterface.planPTP(make_pose(make_point(1.58022,-0.3400141, 0.816495), make_quaternion(0.33516,0.61395,-0.0149517,0.714498)));
        visual_tools.prompt("Press 'next' for EXECUTION.");
        move_group_interface.execute(plan);
    #pragma endregion

    #pragma region CARTESIAN pohyb
        visual_tools.prompt("Press 'next' for PLANNING.");
        std::vector<geometry_msgs::Pose> waypoints2;
        waypoints2.push_back(make_pose(make_point(1.58022,-0.3400141, 0.816495), make_quaternion(0.33516,0.61395,-0.0149517,0.714498)));
        waypoints2.push_back(make_pose(make_point(1.58,-0.229689, 0.629234), make_quaternion(0.333476,0.717239,0.0370962,0.610725)));
        trajectory = moveInterface.planCartesian(waypoints2);
        visual_tools.prompt("Press 'next' for EXECUTION.");
        move_group_interface.execute(trajectory);
    #pragma endregion

    #pragma region JOINT-STATE pohyb HOME
        visual_tools.prompt("Press 'next' for PLANNING.");
        plan = moveInterface.planTarget("home");
        visual_tools.prompt("Press 'next' for EXECUTION.");
        move_group_interface.execute(plan);
    #pragma endregion

    visual_tools.prompt("Press 'next' for EXIT.");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    ros::shutdown();
    return 0;
}
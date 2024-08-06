#include <ros/ros.h>
#include "geometry_msgs_extensions.h"
using namespace geometry_msgs::msg;

//Marker Visualization
#include <visualization_msgs/Marker.h>

//Collision Geometry
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit_msgs/AttachedCollisionObject.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model_publisher_node");
    ros::NodeHandle n;

    ros::Publisher publisher_stl = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    ros::Publisher publisher_col = n.advertise<moveit_msgs::CollisionObject>("/collision_object", 1);

    #pragma region Marker mesh message
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "meshes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = make_pose(make_point(2.0,0.75, 0.0), make_quaternion(0.0,0.0,0.0,1.0));

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.7;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.mesh_resource = "package://zad2_models/truck.stl";
    #pragma endregion

    #pragma region Collision mesh message
    shapes::Mesh*  c_mesh = shapes::createMeshFromResource("package://zad2_models/truck.stl");
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(c_mesh, mesh_msg);
    shape_msgs::Mesh truck_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    moveit_msgs::CollisionObject collision;
    collision.id = "truck_model";
    collision.header.frame_id = "world";
    collision.header.stamp = ros::Time::now();
    collision.operation = moveit_msgs::CollisionObject::ADD;
    collision.mesh_poses.resize(1);
    collision.meshes.push_back(truck_mesh);
    collision.mesh_poses[0] = make_pose(make_point(2.0,0.75, 0.0), make_quaternion(0.0,0.0,0.0,1.0));
    #pragma endregion

    ros::Rate rate(1);
    while(ros::ok())
    {
        marker.header.stamp = ros::Time::now();
        collision.header.stamp = ros::Time::now();
        publisher_stl.publish(marker);
        publisher_col.publish(collision);
        rate.sleep();
    }
    return 0;
}
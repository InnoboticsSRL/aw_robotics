/*
* Copyright Ⓒ Automationware Srl 2022 
* The robot will move using joint position commands
* Check workspace for possible collisions
* Author: selvija@automationware.it
* Mainteiner: selvija@automationware.it
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";

    ros::Duration(3).sleep();

    ROS_INFO("Starting cartesian_coords_test programm.");
    ROS_INFO("Make sure robot cannot collide with object in workspace!");

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group_interface.setEndEffectorLink("awtube3_link6");
    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();

    geometry_msgs::Pose next;

    // convert to Pose from PoseStamped message
    next.position = current_pose.pose.position;
    next.orientation = current_pose.pose.orientation;

    // Waypoints to draw a rectangle with the tip of the end effector
    // WARNING: Make sure robot cannot collide with object in workspace!
    std::vector<geometry_msgs::Pose> waypoints;
    next.position.y += 0.2;
    waypoints.push_back(next);

    next.position.x += 0.2;
    waypoints.push_back(next);

    next.position.y -= 0.4;
    waypoints.push_back(next);

    next.position.x -= 0.2;
    waypoints.push_back(next);

    next.position.y += 0.2;
    waypoints.push_back(next);

    // cartesian path parameters
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    moveit::core::MoveItErrorCode return_value = move_group_interface.execute( trajectory ); // move the cartesian plan

		if( return_value == moveit::planning_interface::MoveItErrorCode::SUCCESS )
		{
		  ROS_INFO( "Move_group.execute successful! ");
		}

    ROS_INFO( "test programm COMPLETED.");

    return 0;
}   
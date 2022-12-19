/*
* Copyright Ⓒ Automationware Srl 2022 
* Script moves robot using joint position commands
* Make sure robot cannot collide with objects in workspace!
* Author: selvija@automationware.it
* Maintainer: selvija@automationware.it
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>

void publishVisualizations(
    const geometry_msgs::Pose &current_pose, 
    moveit_visual_tools::MoveItVisualTools visual_tools, 
    const moveit::planning_interface::MoveGroupInterface::Plan &plan,
    moveit::planning_interface::MoveGroupInterface &move_group
    )
{
    visual_tools.publishAxisLabeled(
        current_pose, 
        "Point(" 
        + std::to_string(current_pose.position.x) + ", " 
        + std::to_string(current_pose.position.y) + ", "
        + std::to_string(current_pose.position.z) + 
        " )"
    );
    visual_tools.publishTrajectoryLine(
        plan.trajectory_, 
        move_group.getCurrentState()->getJointModelGroup("arm")->getLinkModel("awtube3_link6"), 
        move_group.getCurrentState()->getJointModelGroup("arm"));
    visual_tools.trigger();
}

bool plan_movement( 
    const geometry_msgs::Pose &pose, 
    moveit::planning_interface::MoveGroupInterface &move_group, 
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    moveit_visual_tools::MoveItVisualTools visual_tools,
    double &scale_vel,
    double &scale_accel )
{
    move_group.setPoseTarget(pose);
    move_group.setMaxVelocityScalingFactor(scale_vel);
    move_group.setMaxAccelerationScalingFactor(scale_accel);
    bool success = move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    publishVisualizations(pose, visual_tools, plan, move_group);
    return success;
}

void move_to_pose(
    const geometry_msgs::Pose &pose, 
    moveit::planning_interface::MoveGroupInterface &move_group, 
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    moveit_visual_tools::MoveItVisualTools visual_tools,
    double &scale_vel,
    double &scale_accel)
{
    if( plan_movement( pose, move_group, plan, visual_tools, scale_vel, scale_accel ) ) {
        ROS_INFO( "POSE GOAL: Successfuly generated Plan! Move to desired JointValues..." );
        move_group.move();
    }
    else {
        ROS_FATAL( "POSE GOAL: Not possible to find an executable trajectory path" );
    }
}

int main( int argc, char **argv )
{
    // ros node initialization
    ros::init( argc, argv, "pose_coords_node" );
    ros::NodeHandle node_handle;

    // start async spinner
    ros::AsyncSpinner spinner( 1 );
    spinner.start();

    ROS_INFO("Starting pose_coords_node test programm.");
    ROS_INFO("Make sure robot cannot collide with object in workspace!");

    // moveit initialization
    const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group( PLANNING_GROUP );
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("awtube3_baselink");
    visual_tools.deleteAllMarkers();
    
    // WARNING: CHOOSE SAFE VALUES
    // safety velocity and acceleration
    double scale_vel = 0.2;
    double scale_accel = 0.2;
    const int ncycles = 2;

    // arbitrarily chosen
    move_group.setPlanningTime( 15.0 );
    move_group.setGoalTolerance( 0.0005 );
    move_group.setGoalOrientationTolerance( 0.0005 );
    move_group.setPlannerId("PTP");

    for (int n=0;n<ncycles;n++)
    {
        ROS_INFO("  ITERATION: %d", n+1);

        visual_tools.resetMarkerCounts();

        move_group.setEndEffectorLink("awtube3_link6");
        geometry_msgs::PoseStamped current_pose_stamped = move_group.getCurrentPose();
        geometry_msgs::Pose current;
        current.position = current_pose_stamped.pose.position;
        current.orientation = current_pose_stamped.pose.orientation;

        current.position.x += 0.3;
        move_to_pose(current, move_group, plan, visual_tools, scale_vel, scale_accel);

        current.position.y += 0.3;
        current.position.z -= 0.1;
        move_to_pose(current, move_group, plan, visual_tools, scale_vel, scale_accel);

        current.position.x -= 0.3;
        move_to_pose(current, move_group, plan, visual_tools, scale_vel, scale_accel);

        current.position.y -= 0.3;
        current.position.z += 0.1;
        move_to_pose(current, move_group, plan, visual_tools, scale_vel, scale_accel);

    }

    ROS_INFO( "test programm COMPLETED.");

    return 0;

 }
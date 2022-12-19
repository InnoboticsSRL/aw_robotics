/*
* Copyright Ⓒ Automationware Srl 2022 
* The robot will move using joint position commands
* Make sure robot cannot collide with objects in workspace!
* Author: selvija@automationware.it
* Mainteiner: selvija@automationware.it
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>

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

int main(int argc, char** argv)
{ 
    // node setup
    ros::init(argc, argv, "move_group_tutorial");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    

    // visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("awtube3_baselink");
    visual_tools.deleteAllMarkers();

    ROS_INFO("Starting cartesian_coords_test programm.");
    ROS_INFO("Make sure robot cannot collide with object in workspace!");

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    

    move_group.setEndEffectorLink("awtube3_link6");
    geometry_msgs::PoseStamped current_pose_stamped = move_group.getCurrentPose();

    geometry_msgs::Pose next;

    // convert to Pose from PoseStamped message
    next.position = current_pose_stamped.pose.position;
    next.orientation = current_pose_stamped.pose.orientation;


    // arbitrarily chosen
    const int nCycles = 4;
    double scale_vel = 0.05;
    double scale_accel = 0.05;
    move_group.setPlanningTime( 15.0 );
    move_group.setGoalTolerance( 0.0005 );
    move_group.setGoalOrientationTolerance( 0.0005 );
    move_group.setPlannerId("LIN");

    // Waypoints to draw a rectangle with the tip of the end effector
    // WARNING: Make sure robot cannot collide with object in workspace!
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(next);

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

    for( int n = 0; n < nCycles ; ++n)
    {
      // visualizations
      visual_tools.deleteAllMarkers();
      // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::MEDIUM);
      for (const auto &it: waypoints)
      {
        // visual_tools.publishAxisLabeled(it, "(" + std::to_string(it.position.x) + " ," 
        //   + std::to_string(it.position.y) +" )", rvt::MEDIUM);
        move_to_pose(it, move_group, plan, visual_tools, scale_vel, scale_accel);
      }
      // visual_tools.trigger();

      // cartesian path parameters
      // moveit_msgs::RobotTrajectory trajectory;
      // const double jump_threshold = 0.0;
      // const double eef_step = 0.01;
      // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

      // execution
      // moveit::core::MoveItErrorCode return_value = move_group.execute( trajectory );



      // if( return_value == moveit::planning_interface::MoveItErrorCode::SUCCESS )
      // {
      //   ROS_INFO( "Move_group.execute successful! ");
      // }

      
    }
    ROS_INFO( "test programm COMPLETED.");

    return 0;
}   
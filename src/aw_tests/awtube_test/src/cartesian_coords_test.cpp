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

int main(int argc, char** argv)
{ 
    // node setup
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";

    // visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("awtube3_baselink");
    visual_tools.deleteAllMarkers();

    ROS_INFO("Starting cartesian_coords_test programm.");
    ROS_INFO("Make sure robot cannot collide with object in workspace!");

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    move_group_interface.setEndEffectorLink("awtube3_link6");
    geometry_msgs::PoseStamped current_pose_stamped = move_group_interface.getCurrentPose();

    geometry_msgs::Pose next;

    // convert to Pose from PoseStamped message
    next.position = current_pose_stamped.pose.position;
    next.orientation = current_pose_stamped.pose.orientation;

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

    const int nCycles = 4;

    for( int n = 0; n < nCycles ; ++n)
    {
      // visualizations
      visual_tools.deleteAllMarkers();
      visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::MEDIUM);
      for (const auto &it: waypoints)
        visual_tools.publishAxisLabeled(it, "(" + std::to_string(it.position.x) + " ," + std::to_string(it.position.y) +" )", rvt::MEDIUM);
      visual_tools.trigger();

      // cartesian path parameters
      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

      // execution
      moveit::core::MoveItErrorCode return_value = move_group_interface.execute( trajectory );

      if( return_value == moveit::planning_interface::MoveItErrorCode::SUCCESS )
      {
        ROS_INFO( "Move_group.execute successful! ");
      }
    }
    ROS_INFO( "test programm COMPLETED.");

    return 0;
}   
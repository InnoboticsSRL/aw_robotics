/*
* Copyright Ⓒ Automationware Srl 2022 
* Script moves robot using joint position commands
* Make sure robot cannot collide with objectS in workspace!
* Author: selvija@automationware.it
* Maintainer: selvija@automationware.it
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

bool plan_movement( 
    const geometry_msgs::Pose &pose, 
    moveit::planning_interface::MoveGroupInterface &move_group, 
    moveit::planning_interface::MoveGroupInterface::Plan &plan, 
    double &scale_vel,
    double &scale_accel )
{
    move_group.setPoseTarget(pose);
    move_group.setMaxVelocityScalingFactor(scale_vel);
    move_group.setMaxAccelerationScalingFactor(scale_accel);
    return (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

void move_to_pose(
    const geometry_msgs::Pose &pose, 
    moveit::planning_interface::MoveGroupInterface &move_group, 
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    double &scale_vel,
    double &scale_accel)
{
    if( plan_movement( pose, move_group, plan, scale_vel, scale_accel ) ) {
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
    ros::init( argc, argv, "coord_joint_node" );
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

    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup( PLANNING_GROUP );
    
    // WARNING: CHOOSE SAFE VALUES
    // safety velocity and acceleration
    double scale_vel = 0.4;
    double scale_accel = 0.4;
    const int ncycles = 2;

    // arbitrarily chosen
    move_group.setPlanningTime( 15.0 );
    move_group.setGoalTolerance( 0.0005 );
    move_group.setGoalOrientationTolerance( 0.0005 );

    for (int n=0;n<ncycles;n++)
    {
        ROS_INFO("  ITERATION: %d", n+1);

        move_group.setEndEffectorLink("awtube3_link6");
        geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
        
        geometry_msgs::Pose current;

        current.position = current_pose.pose.position;
        current.orientation = current_pose.pose.orientation;

        current.position.x += 0.2;

        // PRESS ENTER TO START
        // WARNING: Make sure robot cannot collide with object in workspace!
        std::cin.get();
        std::cout << "Press Enter for next movement:" << std::endl;
        std::cin.get();

        move_to_pose(current, move_group, plan, scale_vel, scale_accel);

        current.position.y += 0.2;
        current.orientation.w +=0.1;

        std::cout << "Press Enter for next movement:" << std::endl;    
        std::cin.get();    

        move_to_pose(current, move_group, plan, scale_vel, scale_accel);

        current.position.x -= 0.2;

        std::cout << "Press Enter for next movement:" << std::endl;
        std::cin.get();

        move_to_pose(current, move_group, plan, scale_vel, scale_accel);

        current.position.y -= 0.2;
        current.orientation.w -=0.1;

        std::cout << "Press Enter for next movement:" << std::endl;    
        std::cin.get();    

        move_to_pose(current, move_group, plan, scale_vel, scale_accel);

    }

    ROS_INFO( "test programm COMPLETED.");

    return 0;

 }
/*
* Copyright Ⓒ Automationware Srl 2022 
* The robot will move using joint position commands
* Author: selvija@automationware.it
* Mainteiner: selvija@automationware.it
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

bool plan_movement( 
    const geometry_msgs::Pose &pose, 
    moveit::planning_interface::MoveGroupInterface &move_group, 
    moveit::planning_interface::MoveGroupInterface::Plan &plan, 
    double &scale_vel )
{
    move_group.setPoseTarget(pose);
    move_group.setMaxVelocityScalingFactor(scale_vel);
    return (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}
//-----------------------------------------------------------------------------------------------------------------------------------

void move_to_pose(
    const geometry_msgs::Pose &pose, 
    moveit::planning_interface::MoveGroupInterface &move_group, 
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    double &scal_vel)
{

    if( plan_movement( pose, move_group, plan, scal_vel ) ) {
        ROS_INFO( "JOINT GOAL: Successfuly generated JointPlan! Move to desired JointValues..." );
        move_group.move();
    }
    else {
        ROS_FATAL( "JOINT GOAL: Not possible to find an executable trajectory path" );
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

    ROS_INFO("Starting coord_joint_node test programm.");
    ROS_INFO("Make sure robot cannot collide with object in workspace!");

    // moveit initialization
    const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group( PLANNING_GROUP );
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // moveit::planning_interface::MoveGroupInterface::Plan initial_plan;

    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup( PLANNING_GROUP );

    // arbitrarily chosen
    move_group.setPlanningTime( 15.0 );
    move_group.setGoalTolerance( 0.0005 );
    move_group.setGoalOrientationTolerance( 0.0005 );

    // arbitrarily chosen
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = 0.2;
    target_pose1.position.z = 0.5;
    geometry_msgs::Pose target_pose2;
    target_pose1.orientation.w = -1.0;
    target_pose1.position.x = -0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    geometry_msgs::Pose target_pose3;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = -0.5;
    

    // velocity multiplier
    double scale_homing_vel = 0.4;
    const int ncycles = 4;

    for (int n=0;n<ncycles;n++)
    {
        ROS_INFO("  ITERATION: %d", n+1);

        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        move_to_pose(target_pose1, move_group, plan, scale_homing_vel);

        // move_to_pose(target_pose2, move_group, plan, scale_homing_vel);

        // move_to_pose(target_pose3, move_group, plan, scale_homing_vel);

    }

    ROS_INFO( "test programm COMPLETED.");

    return 0;

 }
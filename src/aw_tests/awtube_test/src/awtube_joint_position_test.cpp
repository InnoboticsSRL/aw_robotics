/*
* Copyright Ⓒ Automationware Srl 2022 
* The robot will move using joint position commands
* Author: selvija@automationware.it
* Mainteiner: selvija@automationware.it
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

bool setTargetJointValues( const std::vector<double> &des_joint_values, moveit::planning_interface::MoveGroupInterface &move_group, double &scale_vel )
{
    std::vector<double> joints;
    joints = move_group.getCurrentJointValues();
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    move_group.setMaxVelocityScalingFactor(scale_vel);
    return move_group.setJointValueTarget( des_joint_values ) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}
//-----------------------------------------------------------------------------------------------------------------------------------

void move_joints(const std::vector<double> &d_j_values, moveit::planning_interface::MoveGroupInterface &move_group, double &scal_vel)
{

    if( setTargetJointValues( d_j_values, move_group, scal_vel ) ) {
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
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    moveit::planning_interface::MoveGroupInterface::Plan initial_plan;

    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup( PLANNING_GROUP );

    // arbitrarily chosen
    move_group.setPlanningTime( 15.0 );
    move_group.setGoalTolerance( 0.0005 );
    move_group.setGoalOrientationTolerance( 0.0005 );

    std::vector<double> home_pos = {0, 0, -1.567, 0, 1.567 , 0};
    // arbitrarily chosen
    std::vector<double> first_pos = {2.39, 0.441, -2.023, -2.331, 2.238 , -1.378};
    std::vector<double> second_pos = {1.341, -1.365, -2.674, 1.554, 0.27 , -1.959};
    std::vector<double> third_pos = {1.229, 0.529, -1.834, -0.481, 0.949 , -2.92};

    // velocity multiplier
    double scale_homing_vel = 0.4;
    const int ncycles = 4;

    for (int n=0;n<ncycles;n++)
    {
      ROS_INFO("  ITERATION: %d", n+1);

      scale_homing_vel = 0.9;
      move_joints(home_pos, move_group, scale_homing_vel);

      scale_homing_vel = 0.9;
      move_joints(first_pos, move_group, scale_homing_vel);

      scale_homing_vel = 0.9;
      move_joints(second_pos, move_group, scale_homing_vel);

      scale_homing_vel = 0.9;
      move_joints(third_pos, move_group, scale_homing_vel);

      scale_homing_vel = 0.9;
      move_joints(home_pos, move_group, scale_homing_vel);


    }

    ROS_INFO( "test programm COMPLETED.");

    return 0;

 }
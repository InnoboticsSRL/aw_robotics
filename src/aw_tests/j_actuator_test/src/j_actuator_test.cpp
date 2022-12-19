/*
* Copyright Ⓒ Automationware Srl 2022 
* Script moves robot using joint position commands
* Make sure robot joint cannot collide with object in workspace!
* Author: selvija@automationware.it
* Maintainer: selvija@automationware.it
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

bool setTargetJointValues(const std::vector<double> &des_joint_values, moveit::planning_interface::MoveGroupInterface &move_group, double &vel_scale, double &accel_scale)
{
    std::vector<double> joints;
    joints = move_group.getCurrentJointValues();
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(accel_scale);
    move_group.setJointValueTarget(des_joint_values) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    
}

void move_joints(const std::vector<double> &des_joint_values, moveit::planning_interface::MoveGroupInterface &move_group, double &vel_scale, double &accel_scale)
{
    if(setTargetJointValues(des_joint_values, move_group, vel_scale, accel_scale))
    {
        ROS_INFO("JOINT GOAL: Successfuly generated JointPlan! Move to desired JointValues...");
        move_group.move();
    }
    else
    {
        ROS_FATAL( "JOINT GOAL: Not possible to find an executable trajectory path" );
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "j_actuator_test");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::cout << "[coord_joint_node]: Starting coord_joint_node test programm." << std::endl;
    std::cout << "[coord_joint_node]: Make sure robot cannot collide with object in workspace!" << std::endl;

    const std::string PLANNING_GROUP = "awjoint";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup( PLANNING_GROUP );

    const int nCycles = 4;
    double vec_scaling = 0.5;
    double accel_scaling = 0.5;
    move_group.setPlanningTime( 15.0 );
    move_group.setGoalTolerance( 0.0005 );
    move_group.setGoalOrientationTolerance( 0.0005 );
    move_group.setPlannerId("PTP");
    
    std::vector<double> neg_extreme = {-3.14/2};
    std::vector<double> pos_extreme = {3.14/2};

    for(int i = 0; i < nCycles ; ++i)
    {
        move_joints(neg_extreme, move_group, vec_scaling, accel_scaling);

        move_joints(pos_extreme, move_group, vec_scaling, accel_scaling);
    }

    return 0;

}

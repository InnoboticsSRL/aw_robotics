/*
* Copyright Ⓒ Automationware Srl 2022
* The joint will move using joint position commands
* Author: selvija@automationware.it
* Mainteiner: selvija@automationware.it
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

bool setTargetJointValues(const std::vector<double> &des_joint_values, moveit::planning_interface::MoveGroupInterface &move_group, double &vel_scale)
{
    std::vector<double> joints;
    joints = move_group.getCurrentJointValues();
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setJointValueTarget(des_joint_values) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    
}

void move_joints(const std::vector<double> &des_joint_values, moveit::planning_interface::MoveGroupInterface &move_group, double &vel_scale)
{
    if(setTargetJointValues(des_joint_values, move_group, vel_scale))
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

    const int nCycles = 20;
    double vec_scaling = 0.9;

    move_group.setPlanningTime( 15.0 );//sec arbitrarily chosen
    // move_group.setGoalTolerance( 0.0005 );//m arbitrarily chosen
    // move_group.setGoalOrientationTolerance( 0.0005 );//rad arbitrarily chosenI
    
    std::vector<double> neg_extreme = {-3.14/2};
    std::vector<double> pos_extreme = {3.14/2};

    for(int i = 0; i < nCycles ; ++i)
    {
        move_joints(neg_extreme, move_group, vec_scaling);

        move_joints(pos_extreme, move_group, vec_scaling);
    }

    return 0;

}

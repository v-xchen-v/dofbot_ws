#include <ros/ros.h>

// moveit
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


const double tau = 2*M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{

}

void closeGripper(trajectory_msgs::JointTrajectory& posture)
{

}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& group)
{
    ROS_INFO("Creating collisions");
    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.resize(3);

    // // adds two tables to hold the object to pick and place and a object to pick and place
    // // add the first table
    // collision_objects[0].id = "table1";
    // collision_objects[0].header.frame_id = "base_link";

    // // define the primitive dimension, position of the table1
    // collision_objects[0].primitives.resize(1);
    // collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    // collision_objects[0].primitives[0].dimensions.resize(3);
    // collision_objects[0].primitives[0].dimensions[0] = 0.2;
    // collision_objects[0].primitives[0].dimensions[1] = 0.4;
    // collision_objects[0].primitives[0].dimensions[2] = 0.4;
    // // pose of table1
    // collision_objects[0].primitive_poses.resize(1);
    // collision_objects[0].primitive_poses[0].position.x = 1;
    // collision_objects[0].primitive_poses[0].position.y = 0;
    // collision_objects[0].primitive_poses[0].position.z = 0.2;
    // collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    // // add table1 to the scene
    // collision_objects[0].operation = collision_objects[0].ADD;


    // planning_scene_interface.applyCollisionObjects(collision_objects);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = group.getPlanningFrame();

    // Define primitive dimension, position of the table 1
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2/4;
    collision_objects[0].primitives[0].dimensions[1] = 0.4/4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4/4;
    // pose of table 1
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.20;
    collision_objects[0].primitive_poses[0].position.y = 0.00;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 1 to the scene
    collision_objects[0].operation = collision_objects[0].ADD;


    // Add the second table
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = group.getPlanningFrame();

    // Define primitive dimension, position of the table 2
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.2/4;
    collision_objects[1].primitives[0].dimensions[1] = 0.4/4;
    collision_objects[1].primitives[0].dimensions[2] = 0.4/4;
    // pose of table 2
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.00;
    collision_objects[1].primitive_poses[0].position.y = 0.20;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    collision_objects[1].primitive_poses[0].orientation.z = 1.7;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the scene
    collision_objects[1].operation = collision_objects[1].ADD;

    // Add the object to be picked
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = group.getPlanningFrame();

    // Define primitive dimension, position of the object
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02/4;
    collision_objects[2].primitives[0].dimensions[1] = 0.02/4;
    collision_objects[2].primitives[0].dimensions[2] = 0.2/4;
    // pose of object
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.20;
    collision_objects[2].primitive_poses[0].position.y = 0.00;
    collision_objects[2].primitive_poses[0].position.z = 0.30;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the object
    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO("Added a table to scene");

}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // grasp pose
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;

    // pick test 1
    orientation.setRPY(-tau/4, -tau/4, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.20;
    grasps[0].grasp_pose.pose.position.y = -0.0085;
    grasps[0].grasp_pose.pose.position.z = 0.25;

    //pre=grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // the grasp will move in y direction
    grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;


    //post-grasp approachh

    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    // the grasp will move in y direction
    grasps[0].post_grasp_retreat.direction.vector.y = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);

    closeGripper(grasps[0].grasp_posture);

    move_group.setSupportSurfaceName("table1");

    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm_group");

    // put the object in the scene
    // ros::WallDuration(1.0).sleep();
    addCollisionObject(planning_scene_interface, group);

    // wait for initialization
    ros::WallDuration(1.0).sleep();

    // pick the object
    pick(group);

    // ros::WallDuration(1.0).sleep();

    // // place the object
    // place(group);

    ros::waitForShutdown();
    return 0;
}

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)

{

	ros::init(argc, argv, "seven_dof_arm_planner");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//Initialize group and scene interfaces
	moveit::planning_interface::MoveGroupInterface group("arm");
	moveit::planning_interface::PlanningSceneInterface current_scene;
	//Waiting for scene initialization
	sleep(2);

	//Define CollisionObject objects to spawn into the scene
	moveit_msgs::CollisionObject grasping_object;
	grasping_object.id = "grasping_object";

	//---Add grasping object to the scene
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.03;
	primitive.dimensions[1] = 0.03;
	primitive.dimensions[2] = 0.08;
	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.x =  0.03;
	pose.position.y =  0.0;
	pose.position.z =  0.65;

	grasping_object.primitives.push_back(primitive);
	grasping_object.primitive_poses.push_back(pose);
	grasping_object.operation = grasping_object.ADD;
	grasping_object.header.frame_id = "base_link";
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(grasping_object);
	//---


	// Once all of the objects (in this case just one) have been added to the
	// vector, we tell the planning scene to add our new box
	current_scene.addCollisionObjects(collision_objects);
	
	sleep(4);
	//---attach object to the robot
	ROS_INFO("Attaching object grasping_object to robot's body");
	moveit_msgs::AttachedCollisionObject attacched_object;
	attacched_object.link_name = "grasping_frame";
	attacched_object.object = grasping_object;
	current_scene.applyAttachedCollisionObject( attacched_object );
	sleep(4);

	ROS_INFO("Detaching object grasping_object to robot's body");
	grasping_object.operation = grasping_object.REMOVE;
	attacched_object.link_name = "grasping_frame";
	attacched_object.object = grasping_object;
	current_scene.applyAttachedCollisionObject( attacched_object );

	sleep(1);

	ros::shutdown();

}

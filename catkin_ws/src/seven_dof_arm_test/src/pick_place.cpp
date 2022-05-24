#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "seven_dof_arm_planner");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);

	spinner.start();

	moveit::planning_interface::MoveGroupInterface group("arm");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


	string line;

	//Waiting for scene initialization
	sleep(2);


	//--- objects into the scene
	moveit::planning_interface::PlanningSceneInterface current_scene;
	geometry_msgs::Pose pose;

	//---Add grasping object to the scene
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.03;
	primitive.dimensions[1] = 0.03;
	primitive.dimensions[2] = 0.08;

	moveit_msgs::CollisionObject grasping_object;
	grasping_object.id = "grasping_object";
	pose.orientation.w = 1.0;
	pose.position.y =  0.0;
	pose.position.x =  0.41;
	pose.position.z =  0.35;

	grasping_object.primitives.push_back(primitive);
	grasping_object.primitive_poses.push_back(pose);
	grasping_object.operation = grasping_object.ADD;
	grasping_object.header.frame_id = "base_link";

	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.3;
	primitive.dimensions[1] = 0.5;
	primitive.dimensions[2] = 0.32;
	moveit_msgs::CollisionObject grasping_table;
	grasping_table.id = "grasping_table";
	pose.position.y =  0.0;
	pose.position.x =  0.46;
	pose.position.z =  0.15;
	grasping_table.primitives.push_back(primitive);
	grasping_table.primitive_poses.push_back(pose);
	grasping_table.operation = grasping_object.ADD;
	grasping_table.header.frame_id = "base_link";
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(grasping_object);
	collision_objects.push_back(grasping_table);
	//---


	// Once all of the objects (in this case just one) have been added to the
	// vector, we tell the planning scene to add our new box
	current_scene.addCollisionObjects(collision_objects);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	const robot_state::JointModelGroup *joint_model_group =
  group.getCurrentState()->getJointModelGroup("arm");

	
	//---approaching
	geometry_msgs::Pose target_pose;
	target_pose.orientation.x = 0;
	target_pose.orientation.y = 0;
	target_pose.orientation.z = 0;
	target_pose.orientation.w = 1;
	target_pose.position.y = 0.0;
	target_pose.position.x = 0.28;
	target_pose.position.z = 0.35;
	group.setPoseTarget(target_pose);
	group.move();
	sleep(2);

	cout << "First motion done!" << endl;
	
	//---grasping
	target_pose.position.y = 0.0;
	target_pose.position.x = 0.34;
	target_pose.position.z = 0.35;
	group.setPoseTarget(target_pose);
	group.move();


	cout << "attacching!" << endl;
	
	//---attach object to the robot
	//grasping_object
	moveit_msgs::AttachedCollisionObject att_coll_object;
	att_coll_object.object.id = "grasping_object";
	att_coll_object.link_name = "gripper_finger_link1";
	att_coll_object.object.operation = att_coll_object.object.ADD;
	planning_scene_interface.applyAttachedCollisionObject(att_coll_object);
	//attacched_object.link_name = "grasping_frame";
	//attacched_object.object = grasping_object;
	//current_scene.applyAttachedCollisionObject( attacched_object );

	sleep(2);

	cout << "second motion!" << endl;
	//---move far away from the grasping position
	target_pose.position.y = 0.0;
	target_pose.position.x = 0.34;
	target_pose.position.z = 0.4;
	group.setPoseTarget(target_pose);
	group.move();
	sleep(2);


	cout << "Picking motion!" << endl;

	//---picking
	target_pose.orientation.x = -1;
	target_pose.orientation.y = 0;
	target_pose.orientation.z = 0;
	target_pose.orientation.w = 0;
	target_pose.position.y = -0.1;
	target_pose.position.x = 0.34;
	target_pose.position.z = 0.4;
	group.setPoseTarget(target_pose);
	group.move();
	//---
	
	
	cout << "Picked!" << endl;
	
	target_pose.position.y = -0.1;
	target_pose.position.x = 0.34;
	target_pose.position.z = 0.35;
	group.setPoseTarget(target_pose);
	group.move();
	//---remove object from robot's body

	att_coll_object.object.operation = att_coll_object.object.REMOVE;
	att_coll_object.link_name = "gripper_finger_link1";
	att_coll_object.object.id = "grasping_object";
	planning_scene_interface.applyAttachedCollisionObject(att_coll_object);

	target_pose.position.y = -0.1;
	target_pose.position.x = 0.32;
	target_pose.position.z = 0.35;
	group.setPoseTarget(target_pose);
	group.move();

	ros::shutdown();

}


#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

#define PATH_LENGTH 160
#define PATH_LENGTH_VICON 150

class Communicator{
	private:
		nav_msgs::Path base_path_current;
		nav_msgs::Path vicon_path_current;

		std::vector<geometry_msgs::PoseStamped> current_base_poses;
		std::vector<geometry_msgs::PoseStamped> current_vicon_poses;

		bool initialized_base_path  = false;
		bool initialized_vicon_path = false;

		int data_num_index_path     = 10;
		int data_num_index_setpoint = 10;

		double base_position_x  = 0;
		double base_position_y  = 0;
		double base_position_z  = 0;

		double vicon_position_x = 0;
		double vicon_position_y = 0;
		double vicon_position_z = 0;

		double vicon_position_x_init = 0;
		double vicon_position_y_init = 0;
		double vicon_position_z_init = 0;

		bool first_initialized_position = false;
		bool first_initialized_vicon = false;

		ros::NodeHandle nh;
		ros::Subscriber position_sub;
		ros::Subscriber vicon_sub;
		ros::Publisher path_pub;
		ros::Publisher path_vicon_pub;

	public:
		Communicator(){
			position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/sgpvo/pose", 10, &Communicator::position_callback, this);
			vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/CHK_M100/CHK_M100",30, &Communicator::vicon_callback, this);

			path_pub       = nh.advertise<nav_msgs::Path>("/rviz/cam_path", 10);
			path_vicon_pub = nh.advertise<nav_msgs::Path>("/rviz/vicon_path", 30);

			current_base_poses = std::vector<geometry_msgs::PoseStamped>(PATH_LENGTH);
			current_vicon_poses = std::vector<geometry_msgs::PoseStamped>(PATH_LENGTH_VICON);
		}

		void position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
			if( !first_initialized_position )
				first_initialized_position = true;

			//>> Base Link
			std::string link_name = "base";

			geometry_msgs::PoseStamped msg_temp;

			base_position_x = msg->pose.position.x;
			base_position_y = msg->pose.position.y;
			base_position_z = msg->pose.position.z;

			msg_temp.pose.position.x = base_position_z + vicon_position_x_init;
			msg_temp.pose.position.y = -base_position_x + vicon_position_y_init;
			msg_temp.pose.position.z = -base_position_y + vicon_position_z_init;

			//std::cout << msg_temp.pose.position.x << ", " << msg_temp.pose.position.y << ", " << msg_temp.pose.position.z << std::endl;

			//  msg->pose.position.x = base_position_x;
			//  msg->pose.position.y = base_position_z;
			//  msg->pose.position.z = base_position_y;

			if(!initialized_base_path) {
				for(int i=data_num_index_path; i < PATH_LENGTH; i++) {
					if(i<=data_num_index_path){
						current_base_poses.at(i).pose.position = msg_temp.pose.position;
					}
					else{
						current_base_poses.at(i).pose.position = current_base_poses.at(i-1).pose.position;
					}
				}
				if(++data_num_index_path == PATH_LENGTH) initialized_base_path = true;
			}
			else {
				for(int i=0; i<PATH_LENGTH-1; i++) {
					current_base_poses.at(i).pose.position = current_base_poses.at(i+1).pose.position;
				}
				current_base_poses.at(PATH_LENGTH-1).pose.position = msg_temp.pose.position;
			}
			base_path_current.poses = current_base_poses;
			base_path_current.header.frame_id = "/world";
			path_pub.publish(base_path_current);
		}


		void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
			if( first_initialized_position & !first_initialized_vicon ){
				vicon_position_x_init = msg->transform.translation.x;
				vicon_position_y_init = msg->transform.translation.y;
				vicon_position_z_init = msg->transform.translation.z;
				first_initialized_vicon = true;

				std::cout << "Cam position is initialized with Vicon coordinates: [" << vicon_position_x_init << ", " << vicon_position_y_init << ", "  << vicon_position_z_init << ']' << std::endl;
			}

			//>> Base Link
			std::string link_name = "base";

			geometry_msgs::PoseStamped msg_temp;

			vicon_position_x = msg->transform.translation.x;
			vicon_position_y = msg->transform.translation.y;
			vicon_position_z = msg->transform.translation.z;

			msg_temp.pose.position.x =  vicon_position_x;
			msg_temp.pose.position.y =  vicon_position_y;
			msg_temp.pose.position.z =  vicon_position_z;

			if(!initialized_vicon_path)	{
				for(int i = data_num_index_path; i < PATH_LENGTH_VICON; i++) {
					if(i <= data_num_index_path) {
						current_vicon_poses.at(i).pose.position = msg_temp.pose.position;
					}
					else {
						current_vicon_poses.at(i).pose.position = current_vicon_poses.at(i-1).pose.position;
					}
				}
				if(++data_num_index_path == PATH_LENGTH_VICON){
					//			vicon_position_x_init = vicon_position_x;
					//			vicon_position_y_init = vicon_position_y;
					//			vicon_position_z_init = vicon_position_z;

					initialized_vicon_path = true;
				}
			}
			else {
				for(int i=0; i<PATH_LENGTH_VICON-1; i++) {
					current_vicon_poses.at(i).pose.position = current_vicon_poses.at(i+1).pose.position;
				}
				current_vicon_poses.at(PATH_LENGTH_VICON-1).pose.position = msg_temp.pose.position;
			}
			vicon_path_current.poses = current_vicon_poses;
			vicon_path_current.header.frame_id = "/world";
			path_vicon_pub.publish(vicon_path_current);
		}
};



int main(int argc, char **argv) {

	ros::init(argc, argv, "trajectory_drawer");
	Communicator comm;
//	ros::Rate rate(100);

	while(ros::ok()) {
		ros::spinOnce();
//		rate.sleep();
	}

	return 0;
}

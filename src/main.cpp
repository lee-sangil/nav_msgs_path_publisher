#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

#define PATH_LENGTH 100
#define PATH_LENGTH_VICON 200

bool updated_base_path  = false;
bool updated_vicon_path = false;

geometry_msgs::PoseStamped      pose;
geometry_msgs::TransformStamped poseVicon;

nav_msgs::Path base_path_current;
nav_msgs::Path vicon_path_current;

std::vector<geometry_msgs::PoseStamped> current_base_poses(PATH_LENGTH);
std::vector<geometry_msgs::PoseStamped> current_vicon_poses(PATH_LENGTH_VICON);

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


void position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  std::string link_name = "base";

  geometry_msgs::PoseStamped *msg_temp;

  base_position_x = msg->pose.position.x;
  base_position_y = msg->pose.position.y;
  base_position_z = msg->pose.position.z;

  msg_temp->pose.position.x = base_position_z + vicon_position_x_init;
  msg_temp->pose.position.y = -base_position_x + vicon_position_y_init;
  msg_temp->pose.position.z = -base_position_y + vicon_position_z_init;

//  msg->pose.position.x = base_position_x;
//  msg->pose.position.y = base_position_z;
//  msg->pose.position.z = base_position_y;

  if(!initialized_base_path) {
    for(int i=data_num_index_path; i < PATH_LENGTH; i++) {
      if(i<=data_num_index_path){
        current_base_poses.at(i).pose.position = msg_temp->pose.position;
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
    current_base_poses.at(PATH_LENGTH-1).pose.position = msg_temp->pose.position;
  }
  base_path_current.poses = current_base_poses;
  base_path_current.header.frame_id = "/world";

  updated_base_path = true;
}


void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  //>> Base Link
  std::string link_name = "base";

  geometry_msgs::PoseStamped *msg_temp;

  vicon_position_x = msg->transform.translation.x;
  vicon_position_y = msg->transform.translation.y;
  vicon_position_z = msg->transform.translation.z;

  msg_temp->pose.position.x =  vicon_position_x;
  msg_temp->pose.position.y =  vicon_position_y;
  msg_temp->pose.position.z =  vicon_position_z;

  if(!initialized_vicon_path)	{
    for(int i = data_num_index_path; i < PATH_LENGTH_VICON; i++) {
      if(i <= data_num_index_path) {
        current_vicon_poses.at(i).pose.position = msg_temp->pose.position;
      }
      else {
        current_vicon_poses.at(i).pose.position = current_vicon_poses.at(i-1).pose.position;
      }
    }
    if(++data_num_index_path == PATH_LENGTH_VICON){
      vicon_position_x_init = vicon_position_x;
      vicon_position_y_init = vicon_position_y;
      vicon_position_z_init = vicon_position_z;

      initialized_vicon_path = true;
    }
  }
  else {
    for(int i=0; i<PATH_LENGTH_VICON-1; i++) {
      current_vicon_poses.at(i).pose.position = current_vicon_poses.at(i+1).pose.position;
    }
    current_vicon_poses.at(PATH_LENGTH_VICON-1).pose.position = msg_temp->pose.position;
  }
  vicon_path_current.poses = current_vicon_poses;
  vicon_path_current.header.frame_id = "/world";


  updated_vicon_path = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_drawer");
  ros::NodeHandle nh;

  //br = new tf::TransformBroadcaster;

  ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/sgpvo/pose", 10, position_callback);
  ros::Subscriber vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/CHK_M100/CHK_M100",30,vicon_callback);


  ros::Publisher path_pub       = nh.advertise<nav_msgs::Path>("/rviz/cam_path", 10);
  ros::Publisher path_vicon_pub = nh.advertise<nav_msgs::Path>("/rviz/vicon_path", 30);

  ros::Rate rate(100);
  //	initialize_base_attitude();
  while(ros::ok()) {
    ros::spinOnce();
    if(updated_base_path){
      path_pub.publish(base_path_current);
      updated_base_path = false;
    }
    if(updated_vicon_path){
      path_vicon_pub.publish(vicon_path_current);
      updated_vicon_path = false;
    }
    rate.sleep();
  }
  //delete br;
  return 0;
}

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <traj_utils/Bspline.h>
#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <sensor_msgs/PointCloud2.h>
#include <unistd.h>
#include "reliable_bridge.hpp"

using namespace std;

std::vector<int> id_list_;
std::vector<string> ip_list_;
ros::Subscriber other_odoms_sub_, one_traj_sub_, joystick_sub_, goal_sub_, object_odoms_sub_, takeoff_land_sub_, occupancy_inflate_sub_;
ros::Publisher other_odoms_pub_, one_traj_pub_, joystick_pub_, goal_pub_, object_odoms_pub_, takeoff_land_pub_, occupancy_inflate_pub_;
ros::Subscriber goal_exploration_sub_,star_cvx_sub_,frontier_sub_;
ros::Publisher goal_exploration_pub_,star_cvx_pub_,frontier_pub_;
int self_id_;
int self_id_in_bridge_;
int drone_num_;
int ground_station_num_;
double odom_broadcast_freq_;
double occupancy_broadcast_freq_;
bool is_groundstation_;
bool pub_occupancy_;

unique_ptr<ReliableBridge> bridge;

inline int remap_ground_station_id(int id)
{
  return id+drone_num_;
}

template <typename T>
int send_to_all_drone_except_me(string topic, T &msg)
{
  int err_code = 0; 
  for (int i = 0; i < drone_num_; ++i)// Only send to all drones.
  {
    if (i == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(i,topic,msg);
    if(err_code< 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!",typeid(T).name());
    }
  }
  return err_code;
}

template <typename T>
int send_to_all_groundstation_except_me(string topic, T &msg)
{
  int err_code = 0;
  for (int i = 0; i < ground_station_num_; ++i)// Only send to all groundstations.
  {
    int ind = remap_ground_station_id(i);
    if (ind == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(ind,topic,msg);
    if(err_code < 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!",typeid(T).name());
    }
  }
  return err_code;
}

void register_callback_to_all_groundstation(string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < ground_station_num_; ++i)
  {
    int ind = remap_ground_station_id(i);
    if (ind == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    bridge->register_callback(ind,topic_name,callback);
  }
}

void register_callback_to_all_drones(string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < drone_num_; ++i)
  {
    if (i == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    bridge->register_callback(i,topic_name,callback);
  }
}
 

// Here is callback from local topic.
void odom_sub_cb(const nav_msgs::OdometryPtr &msg)
{

  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  msg->child_frame_id = string("drone_") + std::to_string(self_id_);

  other_odoms_pub_.publish(msg); // send to myself
  send_to_all_drone_except_me("/odom",*msg);// Only send to drones.
  send_to_all_groundstation_except_me("/odom",*msg);// Only send to ground stations.
}




void occupancy_inflate_sub_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
    static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * occupancy_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;
  // msg->child_frame_id = string("obj_") + std::to_string(self_id_);

  // object_odoms_pub_.publish(msg); // send to myself
  // send_to_all_drone_except_me("/object_odom",*msg);// Only send to drones.
  send_to_all_groundstation_except_me("/occupancy_inf",*msg);// Only send to ground stations.
}

void one_traj_sub_cb(const traj_utils::BsplinePtr &msg)
{
  one_traj_pub_.publish(msg);  // Send to myself.
  if (bridge->send_msg_to_all("/traj_from_planner",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (ONE_TRAJ)!!!");
  }
}

void joystick_sub_cb(const sensor_msgs::JoyPtr &msg)
{
  joystick_pub_.publish(msg); // Send to myself.
  send_to_all_drone_except_me("/joystick",*msg);
}

void takeoff_land_sub_cb(const quadrotor_msgs::TakeoffLandPtr &msg)
{
  // takeoff_land_pub_.publish(msg); // Send to myself.
  send_to_all_drone_except_me("/takeoff_land",*msg);
}

void goal_sub_cb(const quadrotor_msgs::GoalSetPtr &msg)
{
  if (msg->drone_id==self_id_in_bridge_)
  {
    goal_pub_.publish(msg);  // Send to myself.
    return;
  }
  
  if(bridge->send_msg_to_one(msg->drone_id,"/goal",*msg)< 0)
  {
    ROS_ERROR("[Bridge] SEND ERROR (GOAL)!!!");
  }
}

void goal_exploration_sub_cb(const quadrotor_msgs::GoalSetPtr &msg)
{
  if (bridge->send_msg_to_all("/goal_exploration",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (goal_exploration)!!!");
  }

}

void star_cvx_sub_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
  if (bridge->send_msg_to_all("/star_cvx",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (star_cvx)!!!");
  }

}

void frontier_sub_cb(const sensor_msgs::PointCloud2Ptr &msg)
{

  if (bridge->send_msg_to_all("/frontier",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (frontier)!!!");
  }

}


// Here is callback when the brodge received the data from others.
void odom_bridge_cb(int ID, ros::SerializedMessage& m)
{
  nav_msgs::Odometry odom_msg_;
  ros::serialization::deserializeMessage(m,odom_msg_);
  other_odoms_pub_.publish(odom_msg_);
}

void object_odom_bridge_cb(int ID, ros::SerializedMessage& m)
{
  nav_msgs::Odometry object_odom_msg_;
  ros::serialization::deserializeMessage(m,object_odom_msg_);
  object_odoms_pub_.publish(object_odom_msg_);
}

void goal_bridge_cb(int ID, ros::SerializedMessage& m)
{
  quadrotor_msgs::GoalSet goal_msg_;
  ros::serialization::deserializeMessage(m,goal_msg_);
  goal_pub_.publish(goal_msg_);
}

void traj_bridge_cb(int ID, ros::SerializedMessage& m)
{
  traj_utils::Bspline Bspline_msg_;
  ros::serialization::deserializeMessage(m,Bspline_msg_);
  one_traj_pub_.publish(Bspline_msg_);
}

void occupancy_inflate_bridge_cb(int ID, ros::SerializedMessage& m)
{
  sensor_msgs::PointCloud2 Occupancy_msg_;
  ros::serialization::deserializeMessage(m,Occupancy_msg_);
  occupancy_inflate_pub_.publish(Occupancy_msg_);
}

void joystick_bridge_cb(int ID, ros::SerializedMessage& m)
{
  sensor_msgs::Joy joystick_msg_;
  ros::serialization::deserializeMessage(m,joystick_msg_);
  joystick_pub_.publish(joystick_msg_);
}

void takeoff_land_bridge_cb(int ID, ros::SerializedMessage& m)
{
  quadrotor_msgs::TakeoffLand takeoff_land_msg_;
  ros::serialization::deserializeMessage(m,takeoff_land_msg_);
  takeoff_land_pub_.publish(takeoff_land_msg_);
}

void goal_exploration_bridge_cb(int ID, ros::SerializedMessage& m)
{
  quadrotor_msgs::GoalSet goal_msg_;
  ros::serialization::deserializeMessage(m,goal_msg_);
  goal_exploration_pub_.publish(goal_msg_);
}

void star_cvx_bridge_cb(int ID, ros::SerializedMessage& m)
{
  sensor_msgs::PointCloud2 point_msg_;
  ros::serialization::deserializeMessage(m,point_msg_);
  star_cvx_pub_.publish(point_msg_);
}

void frontier_bridge_cb(int ID, ros::SerializedMessage& m)
{
  sensor_msgs::PointCloud2 point_msg_;
  ros::serialization::deserializeMessage(m,point_msg_);
  frontier_pub_.publish(point_msg_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  // nh.param("broadcast_ip", udp_ip_, string("127.0.0.255"));
  nh.param("self_id", self_id_, -1);
  nh.param("is_ground_station", is_groundstation_, false);
  nh.param("drone_num", drone_num_, 0);
  nh.param("ground_station_num", ground_station_num_, 0);
  nh.param("odom_max_freq", odom_broadcast_freq_, 1000.0);
  nh.param("occupancy_broadcast_freq", occupancy_broadcast_freq_, 10.0);
  nh.param("pub_occupancy", pub_occupancy_, true);

  id_list_.resize(drone_num_ + ground_station_num_);
  ip_list_.resize(drone_num_ + ground_station_num_);
  for (int i = 0; i < drone_num_ + ground_station_num_; ++i)
  {
    nh.param((i < drone_num_ ? "drone_ip_" + to_string(i) : "ground_station_ip_" + to_string(i-drone_num_)), ip_list_[i], string("127.0.0.1"));
    id_list_[i]=i;
  }  
  self_id_in_bridge_ = self_id_;
  if (is_groundstation_)
  {
    self_id_in_bridge_ = remap_ground_station_id(self_id_);
  }
  //the ground statation ID = self ID + drone_num_

  if (self_id_in_bridge_ < 0 || self_id_in_bridge_ > 99)
  {
    ROS_WARN("[swarm bridge] Wrong self_id!");
    exit(EXIT_FAILURE);
  }

  //initalize the bridge  
  bridge.reset(new ReliableBridge(self_id_in_bridge_,ip_list_,id_list_,100000));    
  

  other_odoms_sub_ = nh.subscribe("my_odom", 10, odom_sub_cb, ros::TransportHints().tcpNoDelay());
  other_odoms_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom", 10);
  //register callback
  register_callback_to_all_drones("/odom",odom_bridge_cb);

  // one_traj_sub_ = nh.subscribe("/broadcast_traj_from_planner", 100, one_traj_sub_cb, ros::TransportHints().tcpNoDelay());
  // one_traj_pub_ = nh.advertise<traj_utils::Bspline>("/broadcast_traj_to_planner", 100);
  // bridge->register_callback_for_all("/traj_from_planner",traj_bridge_cb);


  // joystick_sub_ = nh.subscribe("/joystick_from_users", 100, joystick_sub_cb, ros::TransportHints().tcpNoDelay()); //from user or onboard nodes
  // joystick_pub_ = nh.advertise<sensor_msgs::Joy>("/joystick_from_bridge", 100);//from bridge
  // register_callback_to_all_groundstation("/joystick",joystick_bridge_cb);//initialize bridge

  // goal_sub_ = nh.subscribe("/goal_user2brig", 100, goal_sub_cb, ros::TransportHints().tcpNoDelay());
  // goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_brig2plner", 100);
  // register_callback_to_all_groundstation("/goal",goal_bridge_cb);


  takeoff_land_sub_ = nh.subscribe("/takeoff_land_from_users", 100, takeoff_land_sub_cb, ros::TransportHints().tcpNoDelay());
  takeoff_land_pub_ = nh.advertise<quadrotor_msgs::TakeoffLand>("/takeoff_land_to_drones", 100);
  register_callback_to_all_groundstation("/takeoff_land",takeoff_land_bridge_cb);

  if (pub_occupancy_){
    occupancy_inflate_sub_ = nh.subscribe("my_occupancy_inflate", 100, occupancy_inflate_sub_cb, ros::TransportHints().tcpNoDelay());
    occupancy_inflate_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/occupancy_inflate_from_bridge", 100);
    register_callback_to_all_drones("/occupancy_inf",occupancy_inflate_bridge_cb);
  }


  

  // goal_exploration_sub_ = nh.subscribe("/goal_with_id", 100, goal_exploration_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  // goal_exploration_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id_to_planner", 100);
  // bridge->register_callback_for_all("/goal_exploration",goal_exploration_bridge_cb);

  // star_cvx_sub_ = nh.subscribe("/free_map/star_cvx", 100, star_cvx_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  // star_cvx_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/free_map/star_cvx_to_planner", 100);
  // bridge->register_callback_for_all("/star_cvx",star_cvx_bridge_cb);

  // frontier_sub_ = nh.subscribe("/frontier_pc", 100, frontier_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  // frontier_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/frontier_pc_to_planner", 100);
  // bridge->register_callback_for_all("/frontier",frontier_bridge_cb);

  ros::spin();

  bridge->StopThread();

  return 0;
}

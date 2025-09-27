#ifndef TCP_DRONE_STATION_HPP
#define TCP_DRONE_STATION_HPP

#include <ros/ros.h>

// 基础消息
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <visualization_msgs/MarkerArray.h>

// MAVROS 专用
#include <mavros_msgs/State.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandLong.h>

using namespace std;

int self_id_;
int self_id_in_bridge_;
int drone_num_;
int ground_station_num_;
bool is_groundstation_;

vector<int> id_list_;
vector<string> ip_list_;

unique_ptr<ReliableBridge> bridge;

// ---------------- 工具函数 ----------------
inline int remap_ground_station_id(int id) {
  return id + drone_num_;
}

template <typename T>
int send_to_all_drone_except_me(const string &topic, const T &msg) {
  int err_code = 0;
  for (int i = 0; i < drone_num_; ++i) {
    if (i == self_id_in_bridge_) continue;
    err_code += bridge->send_msg_to_one(i, topic, msg);
  }
  return err_code;
}

template <typename T>
int send_to_all_groundstation_except_me(const string &topic, const T &msg) {
  int err_code = 0;
  for (int i = 0; i < ground_station_num_; ++i) {
    int ind = remap_ground_station_id(i);
    if (ind == self_id_in_bridge_) continue;
    err_code += bridge->send_msg_to_one(ind, topic, msg);
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

#endif 
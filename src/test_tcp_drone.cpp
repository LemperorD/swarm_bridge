#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include "reliable_bridge.hpp"
// #include <robot_msgs/SO2Command.h>

using namespace std;

// ---------------- 全局变量 ----------------
int self_id_;
int self_id_in_bridge_;
int drone_num_;
int ground_station_num_;

double odom_broadcast_freq_;
double occupancy_broadcast_freq_;
double so2_cmd_broadcast_freq_;
double cmd_vel_broadcast_freq_;

bool is_groundstation_;
bool pub_occupancy_;
bool pub_so2_cmd_;
bool pub_cmd_vel_;

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

// ---------------- 回调函数 ----------------
ros::Publisher odom_pub_, odom_pub_share_;

// 本地订阅 → 发送给其他节点
void odom_sub_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * odom_broadcast_freq_ < 1.0) return;
  t_last = t_now;

  auto odom = *msg;
  odom.child_frame_id = "drone_1";
  odom.header.frame_id = "map";

  send_to_all_drone_except_me("/odom_tcp", odom);
  send_to_all_groundstation_except_me("/odom_tcp", odom);
}

// Bridge 接收 → 发布到本地
void odom_bridge_cb(int ID, ros::SerializedMessage &m) {
  nav_msgs::Odometry odom;
  ros::serialization::deserializeMessage(m, odom);
  odom_pub_.publish(odom);
  odom_pub_share_.publish(odom);
}

// ---------------- main ----------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  // 参数读取
  nh.param("self_id", self_id_, -1);
  nh.param("is_ground_station", is_groundstation_, false);
  nh.param("drone_num", drone_num_, 1);
  nh.param("ground_station_num", ground_station_num_, 0);

  nh.param("odom_max_freq", odom_broadcast_freq_, 100.0);
  nh.param("occupancy_broadcast_freq", occupancy_broadcast_freq_, 10.0);
//   nh.param("so2_cmd_broadcast_freq", so2_cmd_broadcast_freq_, 50.0);
  nh.param("cmd_vel_broadcast_freq", cmd_vel_broadcast_freq_, 50.0);

  nh.param("pub_occupancy", pub_occupancy_, true);
  nh.param("pub_so2_cmd", pub_so2_cmd_, true);
  nh.param("pub_cmd_vel", pub_cmd_vel_, true);

  // 配置 ID & IP
  id_list_.resize(drone_num_ + ground_station_num_);
  ip_list_.resize(drone_num_ + ground_station_num_);
  for (int i = 0; i < drone_num_ + ground_station_num_; ++i) {
    nh.param((i < drone_num_ ? "drone_ip_" + to_string(i)
                             : "ground_station_ip_" + to_string(i - drone_num_)),
             ip_list_[i], string("127.0.0.1"));
    id_list_[i] = i;
  }

  self_id_in_bridge_ = self_id_;
  if (is_groundstation_) self_id_in_bridge_ = remap_ground_station_id(self_id_);

  bridge.reset(new ReliableBridge(self_id_in_bridge_, ip_list_, id_list_, 100000));

  // ---------------- Topic 绑定 ----------------
  // Odom
  auto odom_sub = nh.subscribe("odom", 10, odom_sub_cb, ros::TransportHints().tcpNoDelay());
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom1", 10);
  odom_pub_share_ = nh.advertise<nav_msgs::Odometry>("/share_odom1", 10);
  bridge->register_callback(0, "/odom", odom_bridge_cb);

  ros::spin();
  bridge->StopThread();
  return 0;
}

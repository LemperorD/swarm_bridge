#include "reliable_bridge.hpp"
#include "tcp_drone_station.hpp"

int self_id_;
int self_id_in_bridge_;
int drone_num_;
int ground_station_num_;
bool is_groundstation_;

vector<int> id_list_;
vector<string> ip_list_;

unique_ptr<ReliableBridge> bridge;

void takeoff_command_bridge_cb(int ID, ros::SerializedMessage &m); // 地面到飞机：起飞指令
void land_command_bridge_cb(int ID, ros::SerializedMessage &m);    // 地面到飞机：降落或返航指令
void waypoint_push_bridge_cb(int ID, ros::SerializedMessage &m); // 地面到飞机：航点下发

void pose_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg); // 飞机到地面：位姿
void vel_sub_cb(const geometry_msgs::Twist::ConstPtr &msg); // 飞机到地面：速度
void battery_sub_cb(const mavros_msgs::BatteryStatus::ConstPtr &msg); // 飞机到地面：电池状态
void state_sub_cb(const mavros_msgs::BatteryStatus::ConstPtr &msg); // 飞机到地面：飞控状态
void waypoint_list_sub_cb(const mavros_msgs::BatteryStatus::ConstPtr &msg); // 飞机到地面：当前航点列表
void video_sub_cb(const mavros_msgs::BatteryStatus::ConstPtr &msg); // 飞机到地面：视频流

int main(int argc, char **argv) {
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  // 参数读取
  nh.param("self_id", self_id_, -1);
  nh.param("is_ground_station", is_groundstation_, false);
  nh.param("drone_num", drone_num_, 1);
  nh.param("ground_station_num", ground_station_num_, 0);

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

  ros::spin();
  bridge->StopThread();
  return 0;
}
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

ros::Publisher takeoff_command_pub_; // 地面到飞机：起飞指令
ros::Publisher land_command_pub_;    // 地面到飞机：降落或返航
ros::ServiceClient waypoint_client; // 地面到飞机：航点下发

mavros_msgs::WaypointPush waypoint_push_srv;

ros::Subscriber pose_sub_; // 飞机到地面：位姿
ros::Subscriber vel_sub_; // 飞机到地面：速度
ros::Subscriber battery_sub_; // 飞机到地面：电池状态
ros::Subscriber state_sub_; // 飞机到地面：飞控状态
ros::Subscriber waypoint_list_sub_; // 飞机到地面：当前航点列表
ros::Subscriber video_sub_; // 飞机到地面：视频流

void takeoff_command_bridge_cb(int ID, ros::SerializedMessage &m); // 地面到飞机：起飞指令
void land_command_bridge_cb(int ID, ros::SerializedMessage &m);    // 地面到飞机：降落或返航指令
void waypoint_list_bridge_cb(int ID, ros::SerializedMessage &m); // 地面到飞机：航点下发

void pose_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg); // 飞机到地面：位姿
void vel_sub_cb(const geometry_msgs::Twist::ConstPtr &msg); // 飞机到地面：速度
void battery_sub_cb(const mavros_msgs::BatteryStatus::ConstPtr &msg); // 飞机到地面：电池状态
void state_sub_cb(const mavros_msgs::State::ConstPtr &msg); // 飞机到地面：飞控状态
void waypoint_list_sub_cb(const mavros_msgs::WaypointList::ConstPtr &msg); // 飞机到地面：当前航点列表
void video_sub_cb(const sensor_msgs::Image::ConstPtr &msg); // 飞机到地面：视频流

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

  // 注册回调函数
  pose_sub_ = nh.subscribe("mavros/local_position/pose", 10, pose_sub_cb, ros::TransportHints().tcpNoDelay());
  vel_sub_ = nh.subscribe("mavros/local_position/velocity", 10, vel_sub_cb, ros::TransportHints().tcpNoDelay());
  battery_sub_ = nh.subscribe("mavros/battery", 10, battery_sub_cb, ros::TransportHints().tcpNoDelay());
  state_sub_ = nh.subscribe("mavros/state", 10, state_sub_cb, ros::TransportHints().tcpNoDelay());
  waypoint_list_sub_ = nh.subscribe("mavros/mission/waypoints", 10, waypoint_list_sub_cb, ros::TransportHints().tcpNoDelay());
  video_sub_ = nh.subscribe("camera/image", 10, video_sub_cb, ros::TransportHints().tcpNoDelay()); // TBD

  waypoint_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

  // if (bridge->register_callback(self_id_in_bridge_, "/takeoff_command_tcp", takeoff_command_bridge_cb))
  // {

  // }

  // if (bridge->register_callback(self_id_in_bridge_, "/land_command_tcp", land_command_bridge_cb))
  // {

  // }

  if(bridge->register_callback(self_id_in_bridge_, "/waypoint_list_tcp", waypoint_list_bridge_cb))
  {
    waypoint_client.call(waypoint_push_srv);
    if (waypoint_push_srv.response.success)
      ROS_INFO("Waypoint push success");
    else
      ROS_INFO("Waypoint push failed");
  }

  bridge->register_callback(self_id_in_bridge_, "/takeoff_command_tcp", takeoff_command_bridge_cb);
  bridge->register_callback(self_id_in_bridge_, "/land_command_tcp", land_command_bridge_cb);

  takeoff_command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("trigger1", 10);
  land_command_pub_ = nh.advertise<std_msgs::String>("trigger2", 10);

  ros::spin();
  bridge->StopThread();
  return 0;
}

void takeoff_command_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::PoseStamped cmd;
  ros::serialization::deserializeMessage(m, cmd);
  ROS_INFO("Received takeoff command");
  takeoff_command_pub_.publish(cmd);
}

void land_command_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::PoseStamped cmd;
  ros::serialization::deserializeMessage(m, cmd);
  ROS_INFO("Received land command");
  land_command_pub_.publish(cmd);
}

void waypoint_list_bridge_cb(int ID, ros::SerializedMessage &m) {
  mavros_msgs::WaypointList wp;
  ros::serialization::deserializeMessage(m, wp);
  ROS_INFO("Received waypoint list");
  waypoint_push_srv.request.waypoints = wp.waypoints;
}

void pose_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  std::string topic = "/pose_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void vel_sub_cb(const geometry_msgs::Twist::ConstPtr &msg) {
  std::string topic = "/vel_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void battery_sub_cb(const mavros_msgs::BatteryStatus::ConstPtr &msg) {
  std::string topic = "/battery_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void state_sub_cb(const mavros_msgs::State::ConstPtr &msg) {
  std::string topic = "/state_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void waypoint_list_sub_cb(const mavros_msgs::WaypointList::ConstPtr &msg) {
  std::string topic = "/wplist_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void video_sub_cb(const sensor_msgs::Image::ConstPtr &msg) {
  std::string topic = "/video_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}
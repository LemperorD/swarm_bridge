#include "reliable_bridge.hpp"
#include "tcp_drone_station.hpp"

// int self_id_;
// int self_id_in_bridge_;
// int drone_num_;
// int ground_station_num_;
// bool is_groundstation_;

// vector<int> id_list_;
// vector<string> ip_list_;

// unique_ptr<ReliableBridge> bridge;

ros::Subscriber takeoff_command_sub_; // 地面到飞机：起飞指令
ros::Subscriber land_command_sub_;    // 地面到飞机：降落或返航
ros::Subscriber waypoint_list_sub_; // 地面到飞机：航点下发

// ros::Publisher pose_pub_; // 飞机到地面：位姿
// ros::Publisher vel_pub_; // 飞机到地面：速度
// ros::Publisher battery_pub_; // 飞机到地面：电池状态
// ros::Publisher state_pub_; // 飞机到地面：飞控状态
// ros::Publisher waypoint_list_pub_; // 飞机到地面：当前航点列表
// ros::Publisher video_pub_; // 飞机到地面：视频流

ros::Publisher* pose_pubs = nullptr;
ros::Publisher* vel_pubs = nullptr;
ros::Publisher* battery_pubs = nullptr;
ros::Publisher* state_pubs = nullptr;
ros::Publisher* waypoint_list_pubs = nullptr;
ros::Publisher* video_pubs = nullptr;

std::vector<ros::Subscriber> waypoint_list_subs_; // 地面到飞机：航点下发

void takeoff_command_sub_cb(const std_msgs::String::ConstPtr &msg); // 地面到飞机：起飞指令
void land_command_sub_cb(const std_msgs::String::ConstPtr &msg);    // 地面到飞机：降落或返航指令
void waypoint_list_sub_cb(const std_msgs::String::ConstPtr &msg); // 地面到飞机：航点下发

void pose_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：位姿
void vel_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：速度
void battery_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：电池状态
void state_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：飞控状态
void waypoint_list_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：当前航点列表
void video_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：视频流

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
    
    pose_pubs = new ros::Publisher[drone_num_];
    vel_pubs = new ros::Publisher[drone_num_];
    battery_pubs = new ros::Publisher[drone_num_];
    state_pubs = new ros::Publisher[drone_num_];
    waypoint_list_pubs = new ros::Publisher[drone_num_];
    video_pubs = new ros::Publisher[drone_num_];
    for (int i = 0; i < drone_num_; ++i) {
      pose_pubs[i] = nh.advertise<geometry_msgs::PoseStamped>("pose_" + std::to_string(i), 10);
      vel_pubs[i] = nh.advertise<geometry_msgs::Twist>("vel_" + std::to_string(i), 10);
      battery_pubs[i] = nh.advertise<mavros_msgs::BatteryStatus>("battery_" + std::to_string(i), 10);
      state_pubs[i] = nh.advertise<mavros_msgs::State>("state_" + std::to_string(i), 10);
      waypoint_list_pubs[i] = nh.advertise<mavros_msgs::WaypointList>("wplist_" + std::to_string(i), 10);
      video_pubs[i] = nh.advertise<sensor_msgs::Image>("video_" + std::to_string(i), 10);
    }

    for (int i = 0; i < drone_num_; ++i) {
      std::string topic_name = "/swarm_bridge/drone_" + std::to_string(i) + "/mission_upload";
      waypoint_list_subs_.push_back(nh.subscribe(topic_name, 10, 
        boost::bind(&waypoint_list_sub_cb, boost::placeholders::_1, i), 
        ros::TransportHints().tcpNoDelay()));
      ROS_INFO("Subscribed to drone %d: %s", i, topic_name.c_str());
    }

    for (int i = 0; i < drone_num_; ++i) {
      if (bridge->register_callback(i, "/pose_tcp_" + std::to_string(i), pose_bridge_cb))
      {
        ROS_INFO("Register pose callback for drone %d", i);
      }
      if (bridge->register_callback(i, "/vel_tcp_" + std::to_string(i), vel_bridge_cb))
      {
        ROS_INFO("Register vel callback for drone %d", i);
      }
      if (bridge->register_callback(i, "/battery_tcp_" + std::to_string(i), battery_bridge_cb))
      {
        ROS_INFO("Register battery callback for drone %d", i);
      }
      if (bridge->register_callback(i, "/state_tcp_" + std::to_string(i), state_bridge_cb))
      {
        ROS_INFO("Register state callback for drone %d", i);
      }
      if (bridge->register_callback(i, "/wplist_tcp_" + std::to_string(i), waypoint_list_bridge_cb))
      {
        ROS_INFO("Register waypoint list callback for drone %d", i);
      }
      if (bridge->register_callback(i, "/video_tcp_" + std::to_string(i), video_bridge_cb))
      {
        ROS_INFO("Register video callback for drone %d", i);
      }
    }

    takeoff_command_sub_ = nh.subscribe("takeoff_command", 10, takeoff_command_sub_cb, ros::TransportHints().tcpNoDelay());
    land_command_sub_ = nh.subscribe("land_command", 10, land_command_sub_cb, ros::TransportHints().tcpNoDelay());
    waypoint_list_sub_ = nh.subscribe("waypoint_push", 10, waypoint_list_sub_cb, ros::TransportHints().tcpNoDelay());

  ros::spin();
  bridge->StopThread();
  return 0;
}

void takeoff_command_sub_cb(const std_msgs::String::ConstPtr &msg) {
  send_to_all_drone_except_me("/takeoff_command_tcp", *msg);
}

void land_command_sub_cb(const std_msgs::String::ConstPtr &msg) {
  send_to_all_drone_except_me("/land_command_tcp", *msg);
}

void waypoint_list_sub_cb(const std_msgs::String::ConstPtr &msg, int drone_id) {
  bridge->send_msg_to_one(drone_id, "/wplist_"+std::to_string(drone_id), *msg);
}

void pose_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::PoseStamped pose_msg_;
  ros::serialization::deserializeMessage(m, pose_msg_);
  pose_pubs[ID].publish(pose_msg_);
}

void vel_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::Twist vel_msg_;
  ros::serialization::deserializeMessage(m, vel_msg_);
  vel_pubs[ID].publish(vel_msg_);
}

void battery_bridge_cb(int ID, ros::SerializedMessage &m) {
  mavros_msgs::BatteryStatus battery_msg_;
  ros::serialization::deserializeMessage(m, battery_msg_);
  battery_pubs[ID].publish(battery_msg_);
}

void state_bridge_cb(int ID, ros::SerializedMessage &m) {
  mavros_msgs::State state_msg_;
  ros::serialization::deserializeMessage(m, state_msg_);
  state_pubs[ID].publish(state_msg_);
}

void waypoint_list_bridge_cb(int ID, ros::SerializedMessage &m) {
  mavros_msgs::WaypointList wplist_msg_;
  ros::serialization::deserializeMessage(m, wplist_msg_);
  waypoint_list_pubs[ID].publish(wplist_msg_);
}

void video_bridge_cb(int ID, ros::SerializedMessage &m) {
  sensor_msgs::Image video_msg_;
  ros::serialization::deserializeMessage(m, video_msg_);
  video_pubs[ID].publish(video_msg_);
}
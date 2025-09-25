#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// 平移值
const double shift_x1 = 0; // 在这里设置 x 方向的平移值
const double shift_y1 = 0; // 在这里设置 y 方向的平移值
const double shift_x2 = -1.2; 
const double shift_y2 = -0.69; 
const double shift_x3 = -1.2; 
const double shift_y3 = 0.69; 
const double shift_x4 = -1.2; // 添加无人机4的x平移值
const double shift_y4 = -0.3; // 添加无人机4的y平移值
const double shift_x5 = -1.2; // 添加无人机5的x平移值
const double shift_y5 = 0.3; // 添加无人机5的y平移值

// 发布者
ros::Publisher pub_drone_1;
ros::Publisher pub_drone_2;
ros::Publisher pub_drone_3;
ros::Publisher pub_drone_4;
ros::Publisher pub_drone_5;
ros::Publisher pub_drone_1_share;
ros::Publisher pub_drone_2_share;
ros::Publisher pub_drone_3_share;
ros::Publisher pub_drone_4_share;
ros::Publisher pub_drone_5_share;

// 回调函数，用于处理第一个无人机的里程计数据
void odomCallbackDrone1(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry modified_msg = *msg; // 复制原始消息
    modified_msg.pose.pose.position.x += shift_x1; // 修改 x 坐标
    modified_msg.pose.pose.position.y += shift_y1; // 修改 y 坐标
    pub_drone_1.publish(modified_msg); // 发布修改后的消息
    pub_drone_1_share.publish(modified_msg);

}

// 回调函数，用于处理第二个无人机的里程计数据
void odomCallbackDrone2(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry modified_msg = *msg; // 复制原始消息
    modified_msg.pose.pose.position.x += shift_x2; // 修改 x 坐标
    modified_msg.pose.pose.position.y += shift_y2; // 修改 y 坐标
    pub_drone_2.publish(modified_msg); // 发布修改后的消息
    pub_drone_2_share.publish(modified_msg);
}

// 回调函数，用于处理第三个无人机的里程计数据
void odomCallbackDrone3(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry modified_msg = *msg; // 复制原始消息
    modified_msg.pose.pose.position.x += shift_x3; // 修改 x 坐标
    modified_msg.pose.pose.position.y += shift_y3; // 修改 y 坐标
    pub_drone_3.publish(modified_msg); // 发布修改后的消息
    pub_drone_3_share.publish(modified_msg);
}

// 回调函数，用于处理第四个无人机的里程计数据
void odomCallbackDrone4(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry modified_msg = *msg; // 复制原始消息
    modified_msg.pose.pose.position.x += shift_x4; // 修改 x 坐标
    modified_msg.pose.pose.position.y += shift_y4; // 修改 y 坐标
    pub_drone_4.publish(modified_msg); // 发布修改后的消息
    pub_drone_4_share.publish(modified_msg);
}

// 回调函数，用于处理第五个无人机的里程计数据
void odomCallbackDrone5(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry modified_msg = *msg; // 复制原始消息
    modified_msg.pose.pose.position.x += shift_x5; // 修改 x 坐标
    modified_msg.pose.pose.position.y += shift_y5; // 修改 y 坐标
    pub_drone_5.publish(modified_msg); // 发布修改后的消息
    pub_drone_5_share.publish(modified_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_odom_modifier");
    ros::NodeHandle nh;

    // 订阅五个无人机的里程计数据
    ros::Subscriber sub_drone_1 = nh.subscribe("/drone_odom_raw1", 10, odomCallbackDrone1);
    ros::Subscriber sub_drone_2 = nh.subscribe("/drone_odom_raw2", 10, odomCallbackDrone2);
    ros::Subscriber sub_drone_3 = nh.subscribe("/drone_odom_raw3", 10, odomCallbackDrone3);
    ros::Subscriber sub_drone_4 = nh.subscribe("/drone_odom_raw4", 10, odomCallbackDrone4);
    ros::Subscriber sub_drone_5 = nh.subscribe("/drone_odom_raw5", 10, odomCallbackDrone5);

    // 初始化发布者
    pub_drone_1 = nh.advertise<nav_msgs::Odometry>("/drone_odom_mod1", 10);
    pub_drone_2 = nh.advertise<nav_msgs::Odometry>("/drone_odom_mod2", 10);
    pub_drone_3 = nh.advertise<nav_msgs::Odometry>("/drone_odom_mod3", 10);
    pub_drone_4 = nh.advertise<nav_msgs::Odometry>("/drone_odom_mod4", 10);
    pub_drone_5 = nh.advertise<nav_msgs::Odometry>("/drone_odom_mod5", 10);
    pub_drone_1_share = nh.advertise<nav_msgs::Odometry>("/share_odom1", 10);
    pub_drone_2_share = nh.advertise<nav_msgs::Odometry>("/share_odom2", 10);
    pub_drone_3_share = nh.advertise<nav_msgs::Odometry>("/share_odom3", 10);
    pub_drone_4_share = nh.advertise<nav_msgs::Odometry>("/share_odom4", 10);
    pub_drone_5_share = nh.advertise<nav_msgs::Odometry>("/share_odom5", 10);

    // ROS 循环
    ros::spin();

    return 0;
}
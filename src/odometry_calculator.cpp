#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

class DroneOdometryCalculator
{
public:
    DroneOdometryCalculator()
    {
        // 初始化节点句柄和发布者
        ros::NodeHandle nh;
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/drone_0/visual_slam/odom", 10);
        
        // 订阅三个无人机的Odometry话题
        drone_1_sub_ = nh.subscribe("/single_odom1", 10, &DroneOdometryCalculator::drone2Callback, this);
        drone_2_sub_ = nh.subscribe("/single_odom2", 10, &DroneOdometryCalculator::drone3Callback, this);
        drone_3_sub_ = nh.subscribe("/single_odom3", 10, &DroneOdometryCalculator::drone4Callback, this);
        
        // 初始化定时器
        timer_ = nh.createTimer(ros::Duration(0.05), &DroneOdometryCalculator::processOdometry, this);
    }

private:
    // 存储无人机的Odometry数据
    struct OdometryData
    {
        geometry_msgs::Point position;
        geometry_msgs::Quaternion orientation;
        bool is_received; // 标志位，指示是否已接收数据
    };

    // 无人机数据
    OdometryData drone_1_data_;
    OdometryData drone_2_data_;
    OdometryData drone_3_data_;

    ros::Publisher odom_pub_;
    ros::Subscriber drone_1_sub_;
    ros::Subscriber drone_2_sub_;
    ros::Subscriber drone_3_sub_;
    ros::Timer timer_;

    // 无人机2的回调函数
    void drone2Callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        drone_1_data_.position = msg->pose.pose.position;
        drone_1_data_.orientation = msg->pose.pose.orientation;
        drone_1_data_.is_received = true;
    }

    // 无人机3的回调函数
    void drone3Callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        drone_2_data_.position = msg->pose.pose.position;
        drone_2_data_.orientation = msg->pose.pose.orientation;
        drone_2_data_.is_received = true;
    }

    // 无人机4的回调函数
    void drone4Callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        drone_3_data_.position = msg->pose.pose.position;
        drone_3_data_.orientation = msg->pose.pose.orientation;
        drone_3_data_.is_received = true;
    }

    // 定时器触发的处理函数
    void processOdometry(const ros::TimerEvent&)
    {
        if (drone_1_data_.is_received && drone_2_data_.is_received && drone_3_data_.is_received)
        {
            // 计算平均位置
            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            odom.pose.pose.position.x = (drone_1_data_.position.x + drone_2_data_.position.x + drone_3_data_.position.x) / 3.0;
            odom.pose.pose.position.y = (drone_1_data_.position.y + drone_2_data_.position.y + drone_3_data_.position.y) / 3.0;
            odom.pose.pose.position.z = (drone_1_data_.position.z + drone_2_data_.position.z + drone_3_data_.position.z) / 3.0;

            // 计算平均方向
            odom.pose.pose.orientation.x = (drone_1_data_.orientation.x + drone_2_data_.orientation.x + drone_3_data_.orientation.x) / 3.0;
            odom.pose.pose.orientation.y = (drone_1_data_.orientation.y + drone_2_data_.orientation.y + drone_3_data_.orientation.y) / 3.0;
            odom.pose.pose.orientation.z = (drone_1_data_.orientation.z + drone_2_data_.orientation.z + drone_3_data_.orientation.z) / 3.0;
            odom.pose.pose.orientation.w = (drone_1_data_.orientation.w + drone_2_data_.orientation.w + drone_3_data_.orientation.w) / 3.0;

            // 发布计算出的Odometry
            odom_pub_.publish(odom);
            // ROS_INFO("Publishing average odometry for drone_0: (%.2f, %.2f, %.2f)",
                    //   odom.pose.pose.position.x,
                    //   odom.pose.pose.position.y,
                    //   odom.pose.pose.position.z);

            // 重置无人机数据
            resetOdometryData();
        }
    }

    // 重置无人机数据
    void resetOdometryData()
    {
        drone_1_data_.is_received = false;
        drone_2_data_.is_received = false;
        drone_3_data_.is_received = false;

        // 清空位置和方向
        drone_1_data_.position.x = 0.0;
        drone_1_data_.position.y = 0.0;
        drone_1_data_.position.z = 0.0;

        drone_2_data_.position.x = 0.0;
        drone_2_data_.position.y = 0.0;
        drone_2_data_.position.z = 0.0;

        drone_3_data_.position.x = 0.0;
        drone_3_data_.position.y = 0.0;
        drone_3_data_.position.z = 0.0;

        drone_1_data_.orientation.x = 0.0;
        drone_1_data_.orientation.y = 0.0;
        drone_1_data_.orientation.z = 0.0;
        drone_1_data_.orientation.w = 1.0; // 四元数的w分量通常设为1.0

        drone_2_data_.orientation.x = 0.0;
        drone_2_data_.orientation.y = 0.0;
        drone_2_data_.orientation.z = 0.0;
        drone_2_data_.orientation.w = 1.0;

        drone_3_data_.orientation.x = 0.0;
        drone_3_data_.orientation.y = 0.0;
        drone_3_data_.orientation.z = 0.0;
        drone_3_data_.orientation.w = 1.0;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_odometry_calculator"); // 初始化节点
    DroneOdometryCalculator calculator; // 创建计算器实例
    ros::spin(); // 进入循环，等待回调
    return 0;
}

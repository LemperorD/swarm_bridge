#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
static ros::Publisher pub;
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // 创建一个新的点云
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 创建一个新的点云，用于存储扩展后的点云
    pcl::PointCloud<pcl::PointXYZ> expanded_cloud;

    // 遍历点云中的每个点
    for (const auto& point : cloud.points)
    {
        // 在 Z 轴上扩展为 -1 到 1 的点
        for (float z_offset = -1.0; z_offset <= 1.0; z_offset += 0.1)
        {
            pcl::PointXYZ new_point(point.x, point.y, point.z + z_offset);
            expanded_cloud.points.push_back(new_point);
        }
    }

    // 更新点云的宽度和高度
    expanded_cloud.width = expanded_cloud.points.size();
    expanded_cloud.height = 1;  // 无法表示为一个2D图像
    expanded_cloud.is_dense = false;

    // 将扩展后的点云转换回ROS消息
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(expanded_cloud, output);
    output.header = msg->header;  // 保留原始消息的header

    // 发布扩展后的点云
    // ROS_WARN("Expanded point cloud published");

    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_expander");
    ros::NodeHandle nh;
    pub = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("expanded_point_cloud", 1);
    // 订阅输入点云话题
    ros::Subscriber sub = nh.subscribe("occupancy_inflate", 1, pointCloudCallback);

    ros::spin();
    return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher pub_scan_tf;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // Create a transform listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try
    {
        // Lookup the transform from base_laser_link to base_link
        geometry_msgs::TransformStamped transformStamped;
        tfBuffer.canTransform("laser", "base_link", ros::Time(0), ros::Duration(0.1));
        transformStamped = tfBuffer.lookupTransform("laser","base_link", ros::Time(0));

        // Apply the transform to each laser point in the scan
        sensor_msgs::LaserScan transformed_scan = *scan_msg;

        for (size_t i = 0; i < transformed_scan.ranges.size(); ++i)
        {
            // Convert each laser point to a geometry_msgs::PointStamped
            geometry_msgs::PointStamped laser_point;
            laser_point.header.frame_id = "laser";
            laser_point.header.stamp = scan_msg->header.stamp;
            laser_point.point.x = transformed_scan.ranges[i] * std::cos(transformed_scan.angle_min + transformed_scan.angle_increment * i);
            laser_point.point.y = transformed_scan.ranges[i] * std::sin(transformed_scan.angle_min + transformed_scan.angle_increment * i);
            laser_point.point.z = 0.0;

            // Transform the laser point to the base_link frame
            geometry_msgs::PointStamped transformed_point;
            tf2::doTransform(laser_point, transformed_point, transformStamped);

            // Update the transformed scan ranges
            transformed_scan.ranges[i] = std::sqrt(std::pow(transformed_point.point.x, 2) + std::pow(transformed_point.point.y, 2));
        }

        // Process and use the transformed scan data as needed
        pub_scan_tf.publish(transformed_scan);

    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Failed to transform laser scan: %s", ex.what());
        return;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_transformer");
    ros::NodeHandle nh;

    // Subscribe to the /scan topic
    ros::Subscriber scan_sub    = nh.subscribe("/scan", 10, scanCallback);
    pub_scan_tf  = nh.advertise<sensor_msgs::LaserScan>("/scan_tf", 1);
    // Spin the node
    ros::spin();

    return 0;
}

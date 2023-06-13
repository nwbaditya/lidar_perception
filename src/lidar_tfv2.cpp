#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

ros::Publisher rotated_scan_pub;


void lidarRotationCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    tf::TransformListener tf_listener;

    try {
        // Create a laser scan message for the rotated data
        sensor_msgs::LaserScan rotated_scan = *msg;

        // Get the transform from the original frame to the target frame
        tf::StampedTransform transform;
        
        // Wait for the transform between the frames
        if(tf_listener.waitForTransform("base_link", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0))){
            tf_listener.lookupTransform("base_link", msg->header.frame_id, msg->header.stamp, transform);

            // Perform the rotation on each laser beam
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                tf::Vector3 point_in, point_out;
                point_in.setX(msg->ranges[i] * cos(msg->angle_min + i * msg->angle_increment));
                point_in.setY(msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment));
                point_in.setZ(0.0);

                point_out = transform * point_in;

                rotated_scan.ranges[i] = sqrt(pow(point_out.x(), 2) + pow(point_out.y(), 2));
            }
        }        

        // Publish the rotated laser scan data to a new topic
        rotated_scan_pub.publish(rotated_scan);

    } catch (tf::TransformException& ex) {
        ROS_ERROR("Transform exception: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "lidar_tf_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create a publisher for the rotated laser scan data
    rotated_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_tf", 10);

    // Subscribe to the original LIDAR data topic
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, lidarRotationCallback);

    ros::spin();

    return 0;
}

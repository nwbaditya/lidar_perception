#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <lidar_perception/Object.h>
#include <lidar_perception/ObjectsStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <cmath>


const float theta = 0;

ros::Publisher pub_lidar_segmented;
ros::Publisher pub_objects_pose;
ros::Publisher pub_objects_marker;

void laser_cb(const sensor_msgs::LaserScan &scan_msg)
{
    // // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // // Container for clustered object coordinate
    std::vector<Eigen::Vector2f> coordinate_vector;

    // // Objects Array Pose Publisher
    lidar_perception::ObjectsStamped objects_msg;
    objects_msg.header.stamp = ros::Time::now();
    objects_msg.header.frame_id = "laser";

    visualization_msgs::MarkerArray marker_array;
    // // Convert to PCL data type
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud2 pntCloud;
    projector_.projectLaser(scan_msg, pntCloud, 2);

    pcl_conversions::toPCL(pntCloud, *cloud);

    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2( *cloud, point_cloud);
    pcl::copyPointCloud(point_cloud, *point_cloudPtr);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(point_cloudPtr);

    pcl::PointXYZ queryPoint;
    queryPoint.x = 0;
    queryPoint.y = 0;
    queryPoint.z = 0;

    float radius = 0.4;

    std::vector<int> pointIndices;
    std::vector<float> pointDistances;
    kdtree.radiusSearch(queryPoint, radius, pointIndices, pointDistances);

    for(int index: pointIndices){
      point_cloudPtr->points[index].x = std::numeric_limits<float>::quiet_NaN();
      point_cloudPtr->points[index].y = std::numeric_limits<float>::quiet_NaN();
      point_cloudPtr->points[index].z = std::numeric_limits<float>::quiet_NaN();
    }

    point_cloudPtr->points.erase(std::remove_if(
      point_cloudPtr->points.begin(),
      point_cloudPtr->points.end(),
      [](const pcl::PointXYZ& p){
        return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
      }),
      point_cloudPtr->points.end());

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(point_cloudPtr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(99999);
    ec.setSearchMethod(tree);
    ec.setInputCloud(point_cloudPtr);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

    int j= 0;
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
          {
            cluster_cloud->push_back(point_cloudPtr->points[*pit]);
            pcl::PointXYZRGB point;

            point.x = point_cloudPtr->points[*pit].x;
            point.y = point_cloudPtr->points[*pit].y;
            point.z = point_cloudPtr->points[*pit].z;

            if (j == 0) //Red	#FF0000	(255,0,0)
            {
                point.r = 0;
                point.g = 0;
                point.b = 255;
            }
            else if (j == 1) //Lime	#00FF00	(0,255,0)
            {
                point.r = 0;
                point.g = 255;
                point.b = 0;
            }
            else if (j == 2) // Blue	#0000FF	(0,0,255)
            {
                point.r = 255;
                point.g = 0;
                point.b = 0;
            }
            else if (j == 3) // Yellow	#FFFF00	(255,255,0)
            {
                point.r = 255;
                point.g = 255;
                point.b = 0;
            }
            else if (j == 4) //Cyan	#00FFFF	(0,255,255)
            {
                point.r = 0;
                point.g = 255;
                point.b = 255;
            }
            else if (j == 5) // Magenta	#FF00FF	(255,0,255)
            {
                point.r = 255;
                point.g = 0;
                point.b = 255;
            }
            else if (j == 6) // Olive	#808000	(128,128,0)
            {
                point.r = 128;
                point.g = 128;
                point.b = 0;
            }
            else if (j == 7) // Teal	#008080	(0,128,128)
            {
                point.r = 0;
                point.g = 128;
                point.b = 128;
            }
            else if (j == 8) // Purple	#800080	(128,0,128)
            {
                point.r = 128;
                point.g = 0;
                point.b = 128;
            }
            else
            {
              if (j % 2 == 0)
              {
                  point.r = 255 * j / (cluster_indices.size());
                  point.g = 128;
                  point.b = 50;
              }
              else
              {
                  point.r = 0;
                  point.g = 255 * j / (cluster_indices.size());
                  point.b = 128;
              }
            }

            point_cloud_segmented->push_back(point);
          }

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster_cloud, centroid);

        float x = centroid[0] * cos(theta) - centroid[1] * sin(theta);
        float y = centroid[0] * sin(theta) + centroid[1] * cos(theta);

        std::cout << centroid[0] << " " << x << std::endl;
        lidar_perception::Object object;
        object.position.x = x;
        object.position.y = y;

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "laser";
        marker.id = j;
        marker.type = marker.CYLINDER;
        marker.action = marker.ADD;
        marker.pose.position = object.position;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_array.markers.push_back(marker);
        objects_msg.objects.push_back(object);
        j++;
      }
    
    for(const Eigen::Vector2f& coordinate: coordinate_vector){
        std::cout << "Coordinate : " << "(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
    }
    // std::cout<< coordinate_vector << std::endl;
    // std::cerr<< "segmented:  " << (int)point_cloud_segmented->size() << "\n";
    // std::cerr<< "origin:     " << (int)point_cloudPtr->size() << "\n";

    std::cout << "markers size : " << marker_array.markers.size() << std::endl;

    // Convert to ROS data type
    point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;

    if(point_cloud_segmented->size()) pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);
    else pcl::toPCLPointCloud2(*point_cloudPtr, cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);
    // Publish the data
    pub_objects_pose.publish(objects_msg);
    pub_lidar_segmented.publish(output);
    pub_objects_marker.publish(marker_array);

}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "perception");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/scan",1000, laser_cb);
    // // Create a ROS publisher for the output point cloud
    pub_lidar_segmented = nh.advertise<sensor_msgs::PointCloud2>("/lidar_segmented", 1);
    pub_objects_pose = nh.advertise<lidar_perception::ObjectsStamped>("/objects_stamped", 1);
    pub_objects_marker = nh.advertise<visualization_msgs::MarkerArray>("/objects_marker", 1);

    // Spin
    ros::spin();
}
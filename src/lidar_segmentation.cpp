#include <ros/ros.h>
#include <tf/transform_listener.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <lidar_perception/Object.h>
#include <lidar_perception/ObjectsStamped.h>
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


//Param
float theta = 0;
double detection_threshold;
double lidar_ignore_radius;
double cluster_tolerance;
int min_cluster;
int max_cluster;

ros::Publisher pub_lidar_segmented;
ros::Publisher pub_objects_pose;

double getEuclideanDistance(double x, double y){
  double distance = sqrt(pow(x, 2) + pow(y,2));
  return distance;
}

void laser_cb(const sensor_msgs::LaserScan& scan_msg)
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

    // // Convert to PCL data type
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tflistener;
    sensor_msgs::PointCloud2 pntCloud;
    projector_.projectLaser(scan_msg, pntCloud, -1);
    // projector_.transformLaserScanToPointCloud("base_link", *scan_msg, pntCloud, tflistener);

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

    std::cout << detection_threshold << std::endl;
    float radius = lidar_ignore_radius;

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
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster);
    ec.setMaxClusterSize(max_cluster);
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

        float x = centroid[0] * cos(theta) + centroid[1] * sin(theta);
        float y = -1 * centroid[0] * sin(theta) + centroid[1] * cos(theta);

        double euclidean_distance = getEuclideanDistance(x, y);
        if(euclidean_distance > detection_threshold){
          continue;
        }

        lidar_perception::Object object;
        object.position.x = x;
        object.position.y = y;

        objects_msg.objects.push_back(object);
        j++;
      }
    
    for(const Eigen::Vector2f& coordinate: coordinate_vector){
        std::cout << "Coordinate : " << "(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
    }
    // Convert to ROS data type
    point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;

    if(point_cloud_segmented->size()) pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);
    // else pcl::toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
    else return;

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);
    // Publish the data
    pub_objects_pose.publish(objects_msg);
    pub_lidar_segmented.publish(output);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "perception");
    // ros::param::get("detection_threshold", detection_threshold);
    ros::NodeHandle nh;
    nh.getParam("/lidar_perception/detection_threshold", detection_threshold);
    nh.getParam("/lidar_perception/lidar_ignore_radius", lidar_ignore_radius);
    nh.getParam("/lidar_perception/cluster_tolerance", cluster_tolerance);
    nh.getParam("/lidar_perception/min_cluster", min_cluster);
    nh.getParam("/lidar_perception/max_cluster", max_cluster);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/scan", 1, laser_cb);
    // // Create a ROS publisher for the output point cloud
    pub_lidar_segmented = nh.advertise<sensor_msgs::PointCloud2>("/lidar_segmented", 1);
    pub_objects_pose = nh.advertise<lidar_perception::ObjectsStamped>("/lidar_perception/objects_stamped", 1);
    // Spin
    ros::spin();
}
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <ctime>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/common/common.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    //ROS_INFO("subscribed!");

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL (*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    /////////////////////////////////

    visualization_msgs::Marker marker_msg;

    marker_msg.type = marker_msg.LINE_LIST;

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*temp_cloud, minPt, maxPt);
    float min_x = minPt.x;
    float max_x = maxPt.x;
    float min_y = minPt.y;
    float max_y = maxPt.y;
    float min_z = minPt.z;
    float max_z = maxPt.z;

    marker_msg.color.g = 1.0f;
    marker_msg.color.a = 1.0;

    geometry_msgs::Point p;

    p.x = min_x;
    p.y = min_y;
    p.z = min_z;
    marker_msg.points.push_back(p);
    p.x = max_x;
    marker_msg.points.push_back(p); //1

    p.x = min_x;
    p.y = min_y;
    p.z = min_z;
    marker_msg.points.push_back(p);
    p.y = max_y;
    marker_msg.points.push_back(p); //2

    p.x = min_x;
    p.y = min_y;
    p.z = min_z;
    marker_msg.points.push_back(p);
    p.z = max_z;
    marker_msg.points.push_back(p); //3

    p.x = min_x;
    p.y = max_y;
    p.z = max_z;
    marker_msg.points.push_back(p); //4
    p.x = max_x;
    marker_msg.points.push_back(p);

    p.x = max_x;
    p.y = min_y;
    p.z = max_z;
    marker_msg.points.push_back(p); //5
    p.y = max_y;
    marker_msg.points.push_back(p);
   
    p.x = max_x;
    p.y = max_y;
    p.z = min_z;
    marker_msg.points.push_back(p); //6
    p.z = max_z;
    marker_msg.points.push_back(p);
    
    p.x = max_x;
    p.y = min_y;
    p.z = min_z;
    marker_msg.points.push_back(p);
    p.y = max_y;
    marker_msg.points.push_back(p); //7

    p.x = max_x;
    p.y = min_y;
    p.z = min_z;
    marker_msg.points.push_back(p);
    p.z = max_z;
    marker_msg.points.push_back(p); //8

    p.x = min_x;
    p.y = max_y;
    p.z = min_z;
    marker_msg.points.push_back(p); //9
    p.x = max_x;
    marker_msg.points.push_back(p);

    p.x = min_x;
    p.y = max_y;
    p.z = min_z;
    marker_msg.points.push_back(p); //10
    p.z = max_z;
    marker_msg.points.push_back(p);
    
    p.x = min_x;
    p.y = min_y;
    p.z = max_z;
    marker_msg.points.push_back(p);
    p.x = max_x;
    marker_msg.points.push_back(p); //11

    p.x = min_x;
    p.y = min_y;
    p.z = max_z;
    marker_msg.points.push_back(p);
    p.y = max_y;
    marker_msg.points.push_back(p); //12

    marker_msg.scale.x = 0.1;

    marker_msg.header.frame_id = "os_sensor";

    pub.publish(marker_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bbox");

    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1, callback);
    ros::Subscriber sub = n.subscribe("/detector/obstacle", 1, callback);
     
    pub = n.advertise<visualization_msgs::Marker>("/detector/bbox",1);

    while (ros::ok())
    {
       ros::spinOnce(); 
    }

    return 0;
}
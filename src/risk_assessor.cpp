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

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1, callback);
   // ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1, callback);
     
    pub = n.advertise<sensor_msgs::PointCloud2>("/detector/obstacle",1);

    ros::Rate rate(10);

    while (ros::ok())
    {  
        ros::spinOnce(); 
        rate.sleep();
    }

    return 0;
}
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

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  //ROS_INFO("subscribed!");

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL (*input, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

  pcl::PointCloud<pcl::PointXYZ> res_pc;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_res_pc (new pcl::PointCloud<pcl::PointXYZ>);
/////////////////////////////////////////////////////////////


  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (temp_cloud);
  vg.setLeafSize (0.25f, 0.25f, 0.25f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*


  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (2); // 2m
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  pcl::PointXYZ closest_center, center;
  pcl::PointXYZ origin(0,0,0);
  float min_dist, dist;
  min_dist = INFINITY;
  for (const auto& cluster : cluster_indices)
  {
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*cloud_filtered)[idx]);
      centroid.add((*cloud_filtered)[idx]);
    } //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    centroid.get(center);
    dist = pcl::euclideanDistance(origin,center);
    if (dist < min_dist)
    {
      min_dist = dist;
      res_pc = *cloud_cluster;
    }

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::cout << "Distance to centroid is " << dist << std::endl;
    std::stringstream ss;
    ss << "~/Desktop/" << "cloud_cluster_" << j << ".pcd";
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
    j++;
  }

//////////////////////////////////////////////////////////////  
  // Convert to ROS data type

  sensor_msgs::PointCloud2 output;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_res_pc (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_res_pc ;
  ptr_res_pc = res_pc.makeShared();
  ptr_res_pc->header.frame_id = "os_sensor";
  //std::cout << ptr_res_pc->points[0] << std::endl;
  pcl::toROSMsg(*ptr_res_pc, output);
  pub.publish(output);   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster");

    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1, callback);
    ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1, callback);
     
    pub = n.advertise<sensor_msgs::PointCloud2>("/detector/obstacle",1);

    while (ros::ok())
    {
       ros::spinOnce(); 
    }

    return 0;
}
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

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

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    //ROS_INFO("subscribed!");

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL (*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    pcl::PointCloud<pcl::PointXYZ> res_pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_res_pc (new pcl::PointCloud<pcl::PointXYZ>);

    // res_pc.header.frame_id = "base_link";
    // ptr_res_pc->header.frame_id = "base_link";

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    // ax + by + cz + d = 0

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);    
    proj.setInputCloud (temp_cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter(*ptr_res_pc);

    // // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*ptr_res_pc, output);


    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detector");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1, callback);
     
    pub = n.advertise<sensor_msgs::PointCloud2>("/2d_cloud",1);

    while (ros::ok())
    {
       ros::spinOnce(); 
    }

    return 0;
}
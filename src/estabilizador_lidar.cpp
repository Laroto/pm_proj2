#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

ros::Publisher pub;

tf2::Quaternion q;
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
bool new_ptc = false;

void callback_imu (const sensor_msgs::ImuConstPtr& imu)
{
    
    q.setX (imu->orientation.x);
    q.setY (imu->orientation.y);
    q.setZ (imu->orientation.z);
    q.setW (imu->orientation.w);
    q = q.inverse();
}

void callback_pointcloud (const sensor_msgs::PointCloud2ConstPtr& ptc)
{
    // ROS_WARN("subscribed");

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL (*ptc, pcl_pc2);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    new_ptc = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "estabilizador");
    ros::NodeHandle nh;

    ros::Subscriber sub_imu, sub_points;

    sub_imu = nh.subscribe("/imu_nav/data", 1, callback_imu);
    sub_points = nh.subscribe("/os_cloud_node/points", 1, callback_pointcloud);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/estabilizador/stable_cloud", 1);

    ros::Rate rate(50);

    while(ros::ok())
    {
        if (new_ptc == true)
        {
            Eigen::Matrix3f mat3 = Eigen::Quaternionf(q.getW(), q.getX(), q.getY(), q.getZ()).toRotationMatrix();
            Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
            mat4.block(0,0,3,3) = mat3;

            pcl::PointCloud<pcl::PointXYZ> res_pc;
            pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_res_pc (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*temp_cloud, *ptr_res_pc, mat4);

            ptr_res_pc->header.frame_id = "os_sensor";

            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*ptr_res_pc,msg);
            pub.publish(msg);

            new_ptc = false;
        }
        

        ros::spinOnce();
    }
    
    
    
}
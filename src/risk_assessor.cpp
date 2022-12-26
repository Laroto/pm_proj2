#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <ctime>
#include <vector>
#include <cmath>
#include <iostream>

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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>
//help me

ros::Publisher pub, debug_pub;
geometry_msgs::Twist odom;
double dist;
double min_dist = INFINITY;
double ttc;
bool debug_mode;
double roll, pitch, yaw;
double vx_rob, vy_rob;

void callback_odom (const nav_msgs::OdometryConstPtr& input)
{
    odom = input->twist.twist;
    odom.linear.z = 0;
}

bool distancia(double ponto1_x,double ponto1_y,double ponto2_x,double ponto2_y,double velocidade_x,double velocidade_y)
{
    double temp, temp2;
    dist = -1;

    //ROS_WARN("pontosX: %f %f pontosY: %f %f",ponto1_x,ponto2_x, ponto1_y, ponto2_y);
    
    if (abs(ponto1_x) - abs(ponto2_x) < 0.0001)
    {
        //std::cout << "entrou";
        temp=ponto1_x/velocidade_x;
        temp2=velocidade_y*temp;
        if ((ponto1_y<temp2 && temp2<ponto2_y)||(ponto1_y>temp2 && temp2>ponto2_y))
        {
            dist = sqrt(pow(ponto1_x,2)+pow(temp2,2));
            //std::cout<< dist<<"\n";
        }
        return true;
    }
    
    if (abs(ponto1_y) - abs(ponto2_y) < 0.0001)
    {
        //std::cout << "entrou";
        temp=ponto1_y/velocidade_y;
        temp2=velocidade_x*temp;
        if ((ponto1_x<temp2 && temp2<ponto2_x)||(ponto1_x>temp2 && temp2>ponto2_x))
        {
            dist = sqrt(pow(ponto1_x,2)+pow(temp2,2));
            //std::cout<< dist <<"\n";
        }
        return true;
    }
    std::cout << dist ;
    return false;
} //retorna true se vai bater e false se não bate


void callback_markers(const visualization_msgs::Marker::ConstPtr& input)
{
    vx_rob = - ( odom.linear.x * cos(yaw) - odom.linear.y * sin(yaw) );
    vy_rob = - ( odom.linear.x * sin(yaw) + odom.linear.y * cos(yaw) );

    if (debug_mode)
    {
        visualization_msgs::Marker out_msg;

        out_msg.color.a = 1.0;
        out_msg.color.b = 1.0f;

        out_msg.type = out_msg.LINE_LIST;

        geometry_msgs::Point p1,p2;

        int k = 25;

        p1.x = 0;
        p1.y = 0;
        p1.z = 0;

        p2.x = k * vx_rob;
        p2.y = k * vy_rob;
        p2.z = 0;

        out_msg.points.push_back(p1);
        out_msg.points.push_back(p2);

        out_msg.scale.x = 0.1;
        out_msg.scale.y = 0.1;
        out_msg.header.frame_id = "os_sensor";

        debug_pub.publish(out_msg);
    }

    bool vai_bater = false;

    for (int i=0; i<12; i++)
    {
        if ( distancia(input->points[i*2].x,
                    input->points[i*2].y,
                    input->points[i*2+1].x,
                    input->points[i*2+1].y,
                    vx_rob,
                    vy_rob) )
            vai_bater = true;
        if ((dist < min_dist) && (dist > 0))
            min_dist = dist;
    }

    ROS_WARN("%f", min_dist);

    if (vai_bater)
        ttc = min_dist / sqrt(pow(odom.linear.x,2)+pow(odom.linear.y,2));    
    else
        ttc = -1;

    std::cout << "vai bater em: " << - ttc << " segundos" << std::endl; 
}

void callback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

   // q = q.inverse();

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "risk_assessor");

    ros::NodeHandle n;

    ros::Subscriber sub_odom = n.subscribe("/gps/rtkfix", 1, callback_odom);
    ros::Subscriber sub_markers = n.subscribe("/detector/bbox", 1, callback_markers);
    ros::Subscriber sub_imu = n.subscribe("/imu_nav/data", 1, callback_imu);
     
    pub = n.advertise<sensor_msgs::PointCloud2>("/detector/obstacle",1);

    n.param<bool>("/debug", debug_mode, "False");

    std::cout << debug_mode << std::endl;

    if (debug_mode)
    {
        ROS_WARN("debug mode enabled");
        debug_pub = n.advertise<visualization_msgs::Marker>("/risk_assessor/debug",1);
    }

    ros::Rate rate(50);

    while (ros::ok())
    {  
        ros::spinOnce(); 
        rate.sleep();

        
    }

    return 0;
}

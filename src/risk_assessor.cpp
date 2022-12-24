#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void callback ()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster");

    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1, callback);
   // ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1, callback);
     
    pub = n.advertise<sensor_msgs::PointCloud2>("/detector/obstacle",1);

    while (ros::ok())
    {
       ros::spinOnce(); 
    }

    return 0;
}
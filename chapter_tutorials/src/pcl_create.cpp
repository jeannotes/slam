#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "pcl_create");
    ros::NodeHandle nh;

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;

    //fill in
    cloud.width = 100;
    cloud.height = 1;
    cloud.points.resize(cloud.height * cloud.width);

    for (size_t i = 0; i < cloud.width; i++)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    //convert to rosmsg
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";

    ros::Rate loop_rate(1);

    while(ros::ok()){
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
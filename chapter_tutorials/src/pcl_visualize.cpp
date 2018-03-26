#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class cloudHandler{
public:
  cloudHandler():viewer("pcl viewer"){
      pcl_sub = nh.subscribe("pcl_output", 10, &cloudHandler::cloudCB, this);
      viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::TimerCB, this);
  }

  void TimerCB(const ros::TimerEvent&){
      if (viewer.wasStopped()){
          ros::shutdown();
      }
  }

  void cloudCB(const sensor_msgs::PointCloud2 &input){
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(input, cloud);

      viewer.showCloud(cloud.makeShared());
  }

protected:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  pcl::visualization::CloudViewer viewer;
  ros::Timer viewer_timer;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "pcl_visualization");
    cloudHandler cloudhandler;

    ros::spin();
}
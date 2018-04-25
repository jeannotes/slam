#include <ndt_map/ndt_map_hmt.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/cell_vector.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <Eigen/Eigen>
#include <string>

void ndtCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
	int i=0;
    ROS_INFO ("%d Received %d data points with the following fields: %s",i, (int)(msg->width * msg->height),
              pcl::getFieldsList (*msg).c_str ());

    lslgeneric::NDTMap nd(new lslgeneric::LazyGrid(0.4));

    Eigen::Affine3d T;
    T.setIdentity();
    nd.addPointCloud(T.translation(), cloud);
    nd.computeNDTCells();

    nd.writeToJFF("1.jff");// const char*

    i++;
}

void convert() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    ROS_INFO ("start converting");
	Eigen::Vector3d localMapSize(3,3,5);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("1524226013228103.pcd", cloud) == -1) {
        ROS_INFO ("error of load points");
        return;
    }
    ROS_INFO ("loading ok %d!", cloud.height * cloud.width);
    lslgeneric::NDTMap nd(new lslgeneric::LazyGrid(0.4));

    Eigen::Affine3d T;
    T.setIdentity();

    nd.initialize(T.translation()(0), T.translation()(1), T.translation()(2), 300, 300, 5);

	pcl::PointCloud<pcl::PointXYZ> first(cloud);
	first.resize(100);
    //nd.addPointCloud(T.translation(), first, 0.1, 100.0, 0.1);	
	//nd.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, T.translation(), 0.1);
	//map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);

	nd.addPointCloudMeanUpdate(T.translation(), cloud, localMapSize, 100e5, 2550, 5, 0.1);

    nd.writeToJFF("loam.jff");// const char*
}

int main (int argc, char** argv) {

    ros::init(argc, argv, "ndt_builder");
    ros::NodeHandle n;
    //ros::Subscriber sub1 = n.subscribe("/velodyne_cloud_registered", 100, ndtCallback);
    //ros::spin();
    convert();

    return (0);
}




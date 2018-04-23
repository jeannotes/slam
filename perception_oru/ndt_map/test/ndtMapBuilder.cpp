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

static int ctr = 0, ctr2 = 0;
static boost::mutex mutex;
static bool fresh = false;

void ndtCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	std::string name = "map";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
	name+=std::to_string(ctr);
    ROS_INFO ("Received %d data points with the following fields: %s", (int)(msg->width * msg->height),
              pcl::getFieldsList (*msg).c_str ());

    lslgeneric::NDTMap nd(new lslgeneric::LazyGrid(0.4));

    Eigen::Affine3d T;
    T.setIdentity();
    nd.addPointCloud(T.translation(), cloud);
    nd.computeNDTCells();

    ROS_INFO("Loaded point cloud, %d's time", ctr+1);
	
	nd.writeToJFF(name);// const char*

    ctr++;
}

void convert(){
	pcl::PointCloud<pcl::PointXYZ> cloud;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("1422133392235211.pcd", cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
	lslgeneric::NDTMap nd(new lslgeneric::LazyGrid(0.4));

    Eigen::Affine3d T;
    T.setIdentity();
    nd.addPointCloud(T.translation(), cloud);
    nd.computeNDTCells();

    ROS_INFO("Loaded point cloud, %d's time", ctr+1);
	
	nd.writeToJFF(name);// const char*
}

int
main (int argc, char** argv) {

    ros::init(argc, argv, "ndt_builder");
    //ros::NodeHandle n;
    //ros::Subscriber sub1 = n.subscribe("/velodyne_cloud_registered_all", 100, ndtCallback);
    //ros::spin();
	convert();

    return (0);
}




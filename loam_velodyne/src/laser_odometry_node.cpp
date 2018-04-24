#include <ros/ros.h>
#include <loam_velodyne/LaserOdometry.h>

/** Main node entry point. */
int main(int argc, char **argv) {
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    loam::LaserOdometry laserOdom(0.1);

    //ros::Timer timer1 = node.createTimer(ros::Duration(1), &loam::LaserOdometry::callback, &laserOdom);

    if (laserOdom.setup(node, privateNode)) {
        // initialization successful
        laserOdom.spin();
    }

    return 0;
}

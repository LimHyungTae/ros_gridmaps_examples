//
// Created by shapelim on 21. 10. 18..
//
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cmath>
#include <string>
#include <map>
#include <iostream>
#include <vector>
#include <memory>

#include <ros/ros.h>

// ROS msgs
#include <nav_msgs/OccupancyGrid.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "occugrid_test");
    ros::NodeHandle nh;

    // Set ROS visualization publishers
    ros::Publisher MapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("/occugrid", 100, true);
    ros::Rate loop_rate(10);

    float max_range_x = 500.0;
    float max_range_y = 500.0;
    float resolution = 0.2;
    float pos_x = 0.0;
    float pos_y = 0.0;
    int width = int(max_range_x / resolution);
    int height = int(max_range_y / resolution);
    nav_msgs::OccupancyGrid gridmap;
    gridmap.info.resolution = resolution;
    geometry_msgs::Pose origin;
    origin.position.x = pos_x;
    origin.position.y = pos_y;
    origin.orientation.w = 1;
    gridmap.info.origin = origin;
    gridmap.info.width = width;
    gridmap.info.height = height;

    const int w = gridmap.info.width;
    const int h = gridmap.info.height;

    for(int i=0;i < w * h; i++) {
        gridmap.data.push_back(-1);
    }

    for(int y = 0; y < h; y++) {
        for(int x = 0; x < w; x++) {
            int ttmpdata = -1; //Unknown
            if (x == 2) ttmpdata = 100;
            if (x > 2 && x < 10) ttmpdata = 0;
            gridmap.data.at(x + w * y) = ttmpdata;
        }
    }
    while (ros::ok()) {
        MapPublisher.publish(gridmap);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
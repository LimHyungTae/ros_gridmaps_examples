//
// Created by shapelim on 16.12.22.
//

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace grid_map;

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_simple_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    ros::Publisher TransformedPublisher = nh.advertise<grid_map_msgs::GridMap>("grid_map_transformed", 1, true);

    // Create grid map.
    GridMap map({"elevation", "steppable"});
    map.setFrameId("map");
    float x_size = 1.2;
    float y_size = 1.6;
    float resolution = 0.2;
    map.setGeometry(Length(x_size, y_size), resolution);
    map["elevation"].setConstant(0);
    map["steppable"].setConstant(-1);
    map.setPosition(Position(x_size / 2, y_size / 2));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map.getLength().x(), map.getLength().y(),
             map.getSize()(0), map.getSize()(1));

    int count = 0;
    Position position_tmp;
    // Just print it out
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        map.getPosition(*it, position_tmp);
        std::cout << count++ << ", " << position_tmp(0) << ", " << position_tmp(1) << std::endl;
    }
    // Visualization would be weird, but the values are fine!

    // Work with grid map in a loop.
    ros::Rate rate(30.0);

     // Add data to grid map.
    for (int i = 0; i < 7; ++i) {
        Index idx_gridmap(0, i);
        map.at("steppable", idx_gridmap) = 2.0;
        idx_gridmap(0) = 2;
        map.at("steppable", idx_gridmap) = 2.0;
    }
    for (int j = 0; j < 3; ++j) {
        Index idx_gridmap(j, 1);
        map.at("steppable", idx_gridmap) = -2.0;

    }

//    for (int i = 0; i < 10; ++i) {
//        Index idx_gridmap(1, i);
//        map.at("steppable", idx_gridmap) = -2.0;
//    }

    // Circle iterator
    Position center(0.0, 0.0);
    double radius = 0.4;

    // https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html
    // 40 deg. tilted
    Eigen::Matrix4d tf_test = Eigen::Matrix4d::Identity();
//    tf_test(0, 0) = 0.7660444; tf_test(0, 1) = 0.0; tf_test(0, 2) = -0.6427876;
//    tf_test(1, 0) = 0.0;       tf_test(1, 1) = 1.0; tf_test(1, 2) = 0.0;
//    tf_test(2, 0) = 0.6427876; tf_test(2, 1) = 1.0; tf_test(2, 2) = 0.7660444;

    tf_test(0, 0) = 0.8660254; tf_test(0, 1) = -0.5;
    tf_test(1, 0) = 0.5;       tf_test(1, 1) = 0.8660254;

    Eigen::Matrix3d rot_mat = tf_test.block<3, 3>(0, 0);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(4.0, 2.0, 0);
    transform.linear() = rot_mat;
    std::cout << transform.matrix() << std::endl;
    auto map_tf = map.getTransformedMap(transform, "steppable", "map", 0.0);
    std::cout << map.getPosition() << std::endl;
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map_tf.getLength().x(), map_tf.getLength().y(),
             map_tf.getSize()(0), map_tf.getSize()(1));
    for (GridMapIterator it(map_tf); !it.isPastEnd(); ++it) {
        map_tf.getPosition(*it, position_tmp);
        std::cout << count++ << ", " << position_tmp(0) << ", " << position_tmp(1) << std::endl;
    }
    while (nh.ok()) {
//        for (grid_map::CircleIterator iterator(map, center, radius);
//             !iterator.isPastEnd(); ++iterator) {
//            map.at("steppable", *iterator) = map.at("steppable", *iterator) + 0.5;
//        }
//
//        center(0) = 0.6; //        center(1) = 1.2;
//        radius = 0.2;
//
//        for (grid_map::CircleIterator iterator(map, center, radius);
//             !iterator.isPastEnd(); ++iterator) {
//            map.at("steppable", *iterator) = 1.0;
//        }

        // Publish grid map.
        ros::Time time = ros::Time::now();
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message, message_tf;
        GridMapRosConverter::toMessage(map, message);
        GridMapRosConverter::toMessage(map_tf, message_tf);
        publisher.publish(message);
        TransformedPublisher.publish(message_tf);
//        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

        // Wait for next cycle.
        rate.sleep();
    }

    return 0;
}
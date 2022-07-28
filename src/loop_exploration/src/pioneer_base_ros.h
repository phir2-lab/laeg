/**
 * This file is part of LAEG
 *
 * Copyright 2022:
 * - Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * - Renan Maffei <rqmaffei at in dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * For more information see <https://github.com/phir2-lab/laeg>
 *
 * LAEG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LAEG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LAEG If not, see <https://www.gnu.org/licenses/>.
**/

#ifndef PIONEERBASE_ROS_H
#define PIONEERBASE_ROS_H

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <std_msgs/Float64.h>

#include "pioneer_base.h"
#include "grid.h"
#include "utils.h"

enum SlamSystem {GMAPPING, FAKE};

class PioneerBase_ROS : public PioneerBase
{
public:
    PioneerBase_ROS();

    void Configure(Configuration* config);

    // ROS stuff
    bool Initialize();

    // Navigation stuff
    void ResumeMovement();
    bool IsMoving();

    // Sensors stuff
    bool ReadOdometryAndSensors();

private:

    SlamSystem slam_system_;

    ros::NodeHandle* n_;
    ros::Rate* rate_;
    tf::TransformListener* listener;

    ros::WallTime start, last, current;

    nav_msgs::Odometry odom_ros_;
    geometry_msgs::PoseStamped pose_ros_;
    nav_msgs::Odometry rel_true_pose_ros_;
    nav_msgs::Odometry abs_true_pose_ros_;
    nav_msgs::OccupancyGrid map_ros_;
    nav_msgs::OccupancyGrid map_visited_ros_;
    sensor_msgs::LaserScan laser_ros_;
    geometry_msgs::Twist twist_ros_;

    // Ros topics subscribers
    ros::Publisher pub_twist_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_rel_true_pose_;
    ros::Subscriber sub_abs_true_pose_;
    ros::Subscriber sub_laser_;
    ros::Subscriber sub_slam_map_;
    ros::Subscriber sub_slam_map_visited_;
    ros::Subscriber sub_slam_map_entropy_;

    void ReceiveOdomFromRosaria(const nav_msgs::Odometry::ConstPtr &value);
    void ReadOdomFromRosariaAndGMapping();
    void ReceiveRelTruePose(const nav_msgs::Odometry::ConstPtr &value);
    void ReceiveAbsTruePose(const nav_msgs::Odometry::ConstPtr &value);
    void ReceiveLaser(const sensor_msgs::LaserScan::ConstPtr &value);

    void ReceiveMapFromGMapping(const nav_msgs::OccupancyGrid::ConstPtr &value);
    void ReceiveMapVisitedFromGMapping(const nav_msgs::OccupancyGrid::ConstPtr &value);
    void ReceiveEntropyFromGMapping(const std_msgs::Float64::ConstPtr &value);
};

#endif // PIONEERBASE_ROS_H

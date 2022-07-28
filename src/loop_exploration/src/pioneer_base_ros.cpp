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

#include "pioneer_base_ros.h"

PioneerBase_ROS::PioneerBase_ROS(): PioneerBase()
{
    int argc=0;
    char** argv=NULL;

    ros::init(argc, argv, "control_P3DX");

    ROS_INFO("control_P3DX!");

    n_ = new ros::NodeHandle("~");
    rate_ = new ros::Rate(20);

    start = ros::WallTime::now();
    last = start;

    listener = new tf::TransformListener;
}

void PioneerBase_ROS::Configure(Configuration *config)
{
    if(config->GetString("slam_system") == "GMAPPING") slam_system_ = GMAPPING;
    else std::cout << std::endl << "[ERROR] Can't read which is the SLAM System from the configuration file!" << std::endl;

    // Initialize publishers and subscribers
    pub_twist_ = n_->advertise<geometry_msgs::Twist>("/rosaria_phi/cmd_vel", 1);

    sub_rel_true_pose_ = n_->subscribe("/rosaria_phi/relTruePose", 100, &PioneerBase_ROS::ReceiveRelTruePose, this);
    sub_abs_true_pose_ = n_->subscribe("/rosaria_phi/absTruePose", 100, &PioneerBase_ROS::ReceiveAbsTruePose, this);
    sub_laser_ = n_->subscribe("/rosaria_phi/laser_laserscan", 100, &PioneerBase_ROS::ReceiveLaser, this);

    //Receive map and tracked pose
    if(slam_system_ == GMAPPING)
    {
        grid_->pad_x = grid_->pad_y = config->GetInt("map_width_height")/2;
        sub_odom_ = n_->subscribe("/rosaria_phi/pose", 100, &PioneerBase_ROS::ReceiveOdomFromRosaria, this);
        sub_slam_map_ = n_->subscribe("/map", 100, &PioneerBase_ROS::ReceiveMapFromGMapping, this);
        sub_slam_map_visited_ = n_->subscribe("/map_visited", 100, &PioneerBase_ROS::ReceiveMapVisitedFromGMapping, this);
        sub_slam_map_entropy_ = n_->subscribe("/slam_gmapping/entropy", 100, &PioneerBase_ROS::ReceiveEntropyFromGMapping, this);
    }
    else
        std::cout << std::endl << "[ERROR] Undefined SLAM System" << std::endl;

}

//######################################################
//# READING METHODS
//######################################################

bool PioneerBase_ROS::ReadOdometryAndSensors()
{
    double roll, pitch, yaw;

    sim_absolute_true_pose_.x = abs_true_pose_ros_.pose.pose.position.x;
    sim_absolute_true_pose_.y = abs_true_pose_ros_.pose.pose.position.y;

    tf::Quaternion q2(abs_true_pose_ros_.pose.pose.orientation.x,
                        abs_true_pose_ros_.pose.pose.orientation.y,
                        abs_true_pose_ros_.pose.pose.orientation.z,
                        abs_true_pose_ros_.pose.pose.orientation.w);

    tf::Matrix3x3 m2(q2);
    m2.getRPY(roll,pitch,yaw);
    sim_absolute_true_pose_.theta = RAD2DEG(yaw);

    sim_relative_true_pose_.x = rel_true_pose_ros_.pose.pose.position.x;
    sim_relative_true_pose_.y = rel_true_pose_ros_.pose.pose.position.y;

    tf::Quaternion q3(rel_true_pose_ros_.pose.pose.orientation.x,
                        rel_true_pose_ros_.pose.pose.orientation.y,
                        rel_true_pose_ros_.pose.pose.orientation.z,
                        rel_true_pose_ros_.pose.pose.orientation.w);

    tf::Matrix3x3 m3(q3);
    m3.getRPY(roll,pitch,yaw);
    sim_relative_true_pose_.theta = RAD2DEG(yaw);

    this->ReadOdomFromRosariaAndGMapping();

    return true;
}

//Rosaria

void PioneerBase_ROS::ReceiveOdomFromRosaria(const nav_msgs::Odometry::ConstPtr &value)
{
    odom_ros_.header = value->header;
    odom_ros_.child_frame_id = value->child_frame_id;
    odom_ros_.pose = value->pose;
    odom_ros_.twist = value->twist;
}

void PioneerBase_ROS::ReceiveAbsTruePose(const nav_msgs::Odometry::ConstPtr &value)
{
    abs_true_pose_ros_.header = value->header;
    abs_true_pose_ros_.child_frame_id = value->child_frame_id;
    abs_true_pose_ros_.pose = value->pose;
    abs_true_pose_ros_.twist = value->twist;
}

void PioneerBase_ROS::ReceiveRelTruePose(const nav_msgs::Odometry::ConstPtr &value)
{
    rel_true_pose_ros_.header = value->header;
    rel_true_pose_ros_.child_frame_id = value->child_frame_id;
    rel_true_pose_ros_.pose = value->pose;
    rel_true_pose_ros_.twist = value->twist;
}

void PioneerBase_ROS::ReceiveLaser(const sensor_msgs::LaserScan::ConstPtr &value)
{
    laser_ros_.header = value->header;

    laser_ros_.angle_min = value->angle_min;
    laser_ros_.angle_max = value->angle_max;
    laser_ros_.angle_increment = value->angle_increment;

    laser_ros_.time_increment = value->time_increment;
    laser_ros_.scan_time = value->scan_time;

    laser_ros_.range_min = value->range_min;
    laser_ros_.range_max = value->range_max;

    laser_ros_.ranges = value->ranges;
    laser_ros_.intensities = value->intensities;
}

//GMapping

void PioneerBase_ROS::ReadOdomFromRosariaAndGMapping()
{
    odometry_.x = odom_ros_.pose.pose.position.x;
    odometry_.y = odom_ros_.pose.pose.position.y;

    tf::Quaternion q1(odom_ros_.pose.pose.orientation.x,
                     odom_ros_.pose.pose.orientation.y,
                     odom_ros_.pose.pose.orientation.z,
                     odom_ros_.pose.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll,pitch,yaw);

    odometry_.theta = RAD2DEG(yaw);

    tf::StampedTransform transform;

    bool goodSLAMPose = true;

    try
    {
        listener->lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ros::Duration(0.05).sleep();
        goodSLAMPose=false;
    }

    if(goodSLAMPose)
    {
        slam_pose_.x = transform.getOrigin().x();
        slam_pose_.y = transform.getOrigin().y();
        geometry_msgs::Pose p;
        quaternionTFToMsg(transform.getRotation(),p.orientation);

        tf::Quaternion q4(p.orientation.x,
                         p.orientation.y,
                         p.orientation.z,
                         p.orientation.w);
        tf::Matrix3x3 m4(q4);
        m4.getRPY(roll,pitch,yaw);

        slam_pose_.theta = RAD2DEG(yaw);
    }
}

void PioneerBase_ROS::ReceiveMapFromGMapping(const nav_msgs::OccupancyGrid::ConstPtr &value)
{
    map_ros_.header = value->header;

    map_ros_.info = value->info;

    map_ros_.data = value->data;

    // Waiting to avoid to change and use the grid at the same time
    while(!grid_->ready_to_update())
    {
        usleep(10000);
    }

    // find limits
    grid_->map_limits.min_x = grid_->map_limits.min_y =  1000000;
    grid_->map_limits.max_x = grid_->map_limits.max_y = -1000000;

    for(int j=0; j<map_ros_.info.height; j++){
        for(int i=0; i<map_ros_.info.width; i++){
            if(map_ros_.data[i+j*map_ros_.info.height] > -1){
                if(grid_->map_limits.min_x > i)
                    grid_->map_limits.min_x = i;
                if(grid_->map_limits.min_y > j)
                    grid_->map_limits.min_y = j;
                if(grid_->map_limits.max_x < i)
                    grid_->map_limits.max_x = i;
                if(grid_->map_limits.max_y < j)
                    grid_->map_limits.max_y = j;
            }
        }
    }

    if(grid_->old_map_limits.min_x > grid_->old_map_limits.max_x)
    {
        grid_->old_map_limits.min_x = grid_->map_limits.min_x;
        grid_->old_map_limits.min_y = grid_->map_limits.min_y;
        grid_->old_map_limits.max_x = grid_->map_limits.max_x;
        grid_->old_map_limits.max_y = grid_->map_limits.max_y;
    }

    int pad_x = grid_->pad_x;
    int pad_y = grid_->pad_y;

    //clear outer data
    for(int j = grid_->old_map_limits.min_y; j<= grid_->map_limits.min_y; j++)
    {
        for(int i = grid_->old_map_limits.min_x; i<= grid_->old_map_limits.max_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            c->slam_value = -1;
            c->potential = 0.5;
        }
    }

    for(int j = grid_->map_limits.max_y; j<= grid_->old_map_limits.max_y; j++)
    {
        for(int i = grid_->old_map_limits.min_x; i<= grid_->old_map_limits.max_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            c->slam_value = -1;
            c->potential = 0.5;
        }
    }

    for(int j = grid_->old_map_limits.min_y; j<= grid_->old_map_limits.max_y; j++)
    {
        for(int i = grid_->old_map_limits.min_x; i<= grid_->map_limits.min_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            c->slam_value = -1;
            c->potential = 0.5;
        }
    }

    for(int j = grid_->old_map_limits.min_y; j<= grid_->old_map_limits.max_y; j++)
    {
        for(int i = grid_->map_limits.max_x; i<= grid_->old_map_limits.max_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            c->slam_value = -1;
            c->potential = 0.5;
        }
    }

    //copy data
    for(int j = grid_->map_limits.min_y; j <= grid_->map_limits.max_y; j++)
    {
        for(int i = grid_->map_limits.min_x; i<= grid_->map_limits.max_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            c->slam_value = (int) map_ros_.data[i+j*map_ros_.info.height];
        }
    }

    grid_->ReleaseToUpdate();
    grid_->NotifyMapChanges();
}

void PioneerBase_ROS::ReceiveMapVisitedFromGMapping(const nav_msgs::OccupancyGrid::ConstPtr &value)
{
    map_visited_ros_.header = value->header;
    map_visited_ros_.info = value->info;
    map_visited_ros_.data = value->data;

    // Waiting to avoid to change and use the grid at the same time
    while(!grid_->ready_to_update())
    {
        usleep(10000);
    }

    // find limits
    grid_->map_visited_limits.min_x = grid_->map_visited_limits.min_y =  1000000;
    grid_->map_visited_limits.max_x = grid_->map_visited_limits.max_y = -1000000;

    for(int j=0; j<map_visited_ros_.info.height; j++)
    {
        for(int i=0; i<map_visited_ros_.info.width; i++)
        {
            if(map_visited_ros_.data[i+j*map_visited_ros_.info.height] > -1)
            {
                if(grid_->map_visited_limits.min_x > i)
                    grid_->map_visited_limits.min_x = i;
                if(grid_->map_visited_limits.min_y > j)
                    grid_->map_visited_limits.min_y = j;
                if(grid_->map_visited_limits.max_x < i)
                    grid_->map_visited_limits.max_x = i;
                if(grid_->map_visited_limits.max_y < j)
                    grid_->map_visited_limits.max_y = j;
            }
        }
    }

    if(grid_->old_map_visited_limits.min_x > grid_->old_map_visited_limits.max_x)
    {
        grid_->old_map_visited_limits.min_x = grid_->map_visited_limits.min_x;
        grid_->old_map_visited_limits.min_y = grid_->map_visited_limits.min_y;
        grid_->old_map_visited_limits.max_x = grid_->map_visited_limits.max_x;
        grid_->old_map_visited_limits.max_y = grid_->map_visited_limits.max_y;
    }

    int pad_x = grid_->pad_x;
    int pad_y = grid_->pad_y;

    //clear outer data
    for(int j = grid_->old_map_visited_limits.min_y; j<= grid_->map_visited_limits.min_y; j++)
    {
        for(int i = grid_->old_map_visited_limits.min_x; i<= grid_->old_map_visited_limits.max_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            c->is_visited = false;
        }
    }

    for(int j = grid_->map_visited_limits.max_y; j<= grid_->old_map_visited_limits.max_y; j++)
    {
        for(int i = grid_->old_map_visited_limits.min_x; i<= grid_->old_map_visited_limits.max_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            c->is_visited = false;
        }
    }

    for(int j = grid_->old_map_visited_limits.min_y; j<= grid_->old_map_visited_limits.max_y; j++)
    {
        for(int i = grid_->old_map_visited_limits.min_x; i<= grid_->map_visited_limits.min_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            c->is_visited = false;
        }
    }

    for(int j = grid_->old_map_visited_limits.min_y; j<= grid_->old_map_visited_limits.max_y; j++)
    {
        for(int i = grid_->map_visited_limits.max_x; i<= grid_->old_map_visited_limits.max_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            c->is_visited = false;
        }
    }

    //copy data
    for(int j = grid_->map_visited_limits.min_y; j <= grid_->map_visited_limits.max_y; j++)
    {
        for(int i = grid_->map_visited_limits.min_x; i<= grid_->map_visited_limits.max_x; i++)
        {
            Cell *c = grid_->cell(i-pad_x,j-pad_y);
            if((int) map_visited_ros_.data[i+j*map_visited_ros_.info.height] == 0)
                c->is_visited = true;
            else
                c->is_visited = false;
        }
    }

    grid_->ReleaseToUpdate();
    grid_->NotifyMapVisitedChanges();
}

void PioneerBase_ROS::ReceiveEntropyFromGMapping(const std_msgs::Float64::ConstPtr &value)
{
    slam_pose_entropy_ = value->data;
}

//######################################################
//# NAVIGATION METHODS
//######################################################

void PioneerBase_ROS::ResumeMovement()
{
    twist_ros_.linear.x = linear_velocity_;
    twist_ros_.angular.z = angular_velocity_;

    // Update robot motion
    pub_twist_.publish(twist_ros_);

    ros::spinOnce();
    rate_->sleep();
}

bool PioneerBase_ROS::IsMoving()
{
    if(linear_velocity_ != 0 || angular_velocity_ != 0) return true;
    else return false;
}

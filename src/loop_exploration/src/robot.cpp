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

#include "robot.h"

//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot(Configuration* config, Logger* logger)
{
    configuration = config;

    logger_ = logger;

    ready_ = false;
    running_ = true;
    initial_iterations_counter_=0;

    grid = new Grid(config);

    base_.grid(grid);

    base_.Configure(config);

    plan = new Planning(config, logger);
    plan->grid(grid);

    int laser_max_range = 10;
    config->GetInt("laser_max_range", laser_max_range);

    // variables used for mapping
    minX_=minY_=INT_MAX;
    maxX_=maxY_=INT_MIN;
}

Robot::~Robot()
{
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::Initialize()
{
}

void Robot::Run()
{
    bool success = base_.ReadOdometryAndSensors();
    if(!success)
    {
        usleep(50000);
        return;
    }
    else if(initial_iterations_counter_<5)
        initial_iterations_counter_++;
    else
        ready_ = true;

    current_pose_ = base_.slam_pose();

    plan->SetNewRobotPose(current_pose_);

    // Save path traversed by the robot
    if(base_.IsMoving())
        StorePath();

    // Navigation
    switch(motion_mode_)
    {
        case POTFIELD:
            FollowPotentialField();
            break;
        case ENDING:
            running_=false;
            break;
        default:
            break;
    }

    base_.ResumeMovement();

    usleep(100000);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::Move(MovingDirection dir)
{
    if(motion_mode_==MANUAL_SIMPLE)
        base_.SetMovementSimple(dir);
}

void Robot::FollowPotentialField()
{
    float linear_vel = 0.1;
    float angular_vel = 0.0;

    Cell* c;
    c = grid->cell(current_pose_.x*grid->map_scale(),current_pose_.y*grid->map_scale());

    float dir_x, dir_y;
    dir_x = c->direction_x;
    dir_y = c->direction_y;

    if(dir_x != 0.0 || dir_y != 0.0)
    {
        float phi = RAD2DEG(atan2(dir_y,dir_x)) - current_pose_.theta;
        phi = NormalizeAngleDEG(phi);

        if(phi < -90.0)
        {
            linear_vel = 0.0;
            angular_vel = -0.5;
        }
        else if(phi > 90.0)
        {
            linear_vel = 0.0;
            angular_vel = 0.5;
        }
        else
            angular_vel = (phi/90.0)*(linear_vel*3.0);
    }

    base_.SetLinAngVelocity(linear_vel, angular_vel);
}

////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::Draw(double xRobot, double yRobot, double angRobot)
{
    double scale = grid->map_scale();
    glTranslatef(xRobot,yRobot,0.0);

    DrawPotGradient(scale);

    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // robot draw in cm
    base_.DrawRobotBody();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

void Robot::DrawPotGradient(double scale)
{
    Cell* c;
    int robotX=current_pose_.x*scale;
    int robotY=current_pose_.y*scale;
    c = grid->cell(robotX,robotY);

    glColor3fv(color_robot_pointer_potential);
    glLineWidth(3);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(5*c->direction_x, 5*c->direction_y);
    }
    glEnd();
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::ready()
{
    return ready_;
}

bool Robot::running()
{
    return running_;
}

float Robot::CalculatePathSize()
{
    float path_size = 0;

    if(path_odom_.size() > 1)
        for(unsigned int i=0;i<path_odom_.size()-1; i++)
            path_size += sqrt(pow(path_odom_[i].x-path_odom_[i+1].x, 2) + pow(path_odom_[i].y-path_odom_[i+1].y, 2));


    return path_size;
}

const Pose& Robot::current_pose()
{
    return current_pose_;
}

void Robot::StorePath()
{
    Pose odom = base_.odometry();
    logger_->LogOdometry(odom.x, odom.y, odom.theta);
    path_odom_.push_back(odom);
    odom = base_.relative_true_pose();
    logger_->LogGroundTruth(odom.x, odom.y, odom.theta);
    path_gt_.push_back(odom);
    odom = base_.slam_pose();
    logger_->LogSLAMPose(odom.x, odom.y, odom.theta);
    logger_->LogEntropy(base_.slam_pose_entropy());
}

void Robot::DrawOdom()
{
    double scale = grid->map_scale();

    if(path_odom_.size() > 1)
    {
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_odom_.size()-1; i++)
            {
                glColor3fv(color_odometry);

                glVertex2f(path_odom_[i].x, path_odom_[i].y);
                glVertex2f(path_odom_[i+1].x, path_odom_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);
    }
}

void Robot::DrawGroundTruth()
{
    double scale = grid->map_scale();

    if(path_gt_.size() > 1)
    {
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_gt_.size()-1; i++)
            {
                glColor3fv(color_ground_truth);

                glVertex2f(path_gt_[i].x, path_gt_[i].y);
                glVertex2f(path_gt_[i+1].x, path_gt_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);
    }
}

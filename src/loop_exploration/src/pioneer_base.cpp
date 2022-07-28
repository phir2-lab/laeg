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

#include "pioneer_base.h"

PioneerBase::PioneerBase()
{
    ready_ = false;

    // wheels' velocities
    vel_left_ = vel_right_ = 0.0;
}

///////////////////////////
///// DRAWING METHODS /////
///////////////////////////

void PioneerBase::DrawRobotBody()
{
    glColor3fv(color_robot_body);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();

    glColor3fv(color_robot_pointer_forward);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();
}

////////////////////////////////////////////////////////////////
////// METHODS FOR READING ODOMETRY & SENSORS MEASUREMENTS /////
////////////////////////////////////////////////////////////////

void PioneerBase::grid(Grid *g)
{
    grid_ = g;
    ready_ = true;
}

const Pose& PioneerBase::odometry()
{
    return odometry_;
}

const Pose& PioneerBase::slam_pose()
{
    return slam_pose_;
}

const Pose& PioneerBase::absolute_true_pose()
{
    return sim_absolute_true_pose_;
}

const Pose& PioneerBase::relative_true_pose()
{
    return sim_relative_true_pose_;
}

const float& PioneerBase::slam_pose_entropy()
{
    return slam_pose_entropy_;
}


void PioneerBase::odometry(const Pose &o)
{
    odometry_ = o;
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void PioneerBase::SetMovementSimple(MovingDirection dir)
{
    double maxVel = 500;

    switch(dir)
    {
        case FRONT:
            vel_left_ = 300;
            vel_right_ = 300;
            break;
        case BACK:
            vel_left_ = -300;
            vel_right_ = -300;
            break;
        case LEFT:
            vel_left_ = -60;
            vel_right_ = 60;
            break;
        case RIGHT:
            vel_left_ = 60;
            vel_right_ = -60;
            break;
        case STOP:
            vel_left_  = 0;
            vel_right_ = 0;
            linear_velocity_ = 0;
            angular_velocity_ = 0;
    }

    if(vel_left_ > maxVel)
        vel_left_ = maxVel;
    else if(vel_left_ < -maxVel)
        vel_left_ = -maxVel;
    if(vel_right_ > maxVel)
        vel_right_ = maxVel;
    else if(vel_right_ < -maxVel)
        vel_right_ = -maxVel;

    SetLinAngVelocityFromWheelsVelocity(vel_left_,vel_right_);
}

void PioneerBase::SetLinAngVelocityFromWheelsVelocity(float lv, float rv)
{
    float b=0.38;

    // the robot's velocity is given in mm/s
    lv /= 1000.0;
    rv /= 1000.0;

    linear_velocity_ = (lv + rv)/2.0;
    angular_velocity_ = (rv - lv)/b;

    if(linear_velocity_ > 1.0)
        linear_velocity_ = 1.0;
    else if(linear_velocity_ < -1.0)
        linear_velocity_ = -1.0;

    if(angular_velocity_ > DEG2RAD(90.0))
        angular_velocity_ = DEG2RAD(90.0);
    else if(angular_velocity_ < DEG2RAD(-90.0))
        angular_velocity_ = DEG2RAD(-90.0);

}

void PioneerBase::SetWheelsVelocityFromLinAngVelocity(float lin_vel, float ang_vel)
{
    float b=0.38;

    vel_left_  = lin_vel - ang_vel*b/(2.0);
    vel_right_ = lin_vel + ang_vel*b/(2.0);

    // the robot's velocity is given in mm/s
    vel_left_ *= 1000;
    vel_right_ *= 1000;

    double maxVel = 500;
    if(vel_left_ > maxVel)
        vel_left_ = maxVel;
    else if(vel_left_ < -maxVel)
        vel_left_ = -maxVel;
    if(vel_right_ > maxVel)
        vel_right_ = maxVel;
    else if(vel_right_ < -maxVel)
        vel_right_ = -maxVel;
}


void PioneerBase::SetLinAngVelocity(float lin_vel, float ang_vel)
{
    linear_velocity_ = lin_vel;
    angular_velocity_ = ang_vel;

    SetWheelsVelocityFromLinAngVelocity(linear_velocity_,angular_velocity_);
}


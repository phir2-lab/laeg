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

#ifndef PIONEERBASE_H
#define PIONEERBASE_H

#include <GL/glut.h>
#include <sstream>
#include <iomanip>

#include "utils.h"
#include "grid.h"

class PioneerBase
{
public:
    PioneerBase();

    // Drawing stuff
    void DrawRobotBody();

    // Navigation stuff
    void SetMovementSimple(MovingDirection dir);
    void SetLinAngVelocityFromWheelsVelocity(float lv, float rv);
    void SetWheelsVelocityFromLinAngVelocity(float lin_vel, float ang_vel);
    void SetLinAngVelocity(float lin_vel, float ang_vel);

    // Sensors stuff
    const Pose& odometry();
    const Pose& slam_pose();
    const Pose& absolute_true_pose();
    const Pose& relative_true_pose();
    const float& slam_pose_entropy();


    void grid(Grid* grid);
    void odometry(const Pose &o);


protected:
    Pose odometry_;
    Pose slam_pose_;
    Pose sim_absolute_true_pose_;
    Pose sim_relative_true_pose_;
    Grid* grid_;

    float slam_pose_entropy_;

    bool ready_;

    // Navigation stuff
    double vel_left_, vel_right_;
    double linear_velocity_, angular_velocity_;

};

#endif // PIONEERBASE_H

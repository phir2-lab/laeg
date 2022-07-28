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

#pragma once

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <unistd.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "grid.h"
#include "pioneer_base_ros.h"
#include "planning.h"

#include "utils.h"
#include "configuration.h"
#include "logger.h"

#include "colors.h"

class Robot
{
public:
    Robot(Configuration* config, Logger* logger);
    ~Robot();

    void Initialize();
    void Run();

    void Move(MovingDirection dir);
    void Draw(double xRobot, double yRobot, double angRobot);

    const Pose& current_pose();
    const Pose& abs_current_pose();

    void StorePath();
    void DrawOdom();
    void DrawGroundTruth();

    bool ready();
    bool running();

    float CalculatePathSize();

    Grid* grid;
    Planning* plan;
    MotionMode motion_mode_;

    Configuration* configuration;

protected:

    Pose current_pose_;
    Pose abs_current_pose_;
    std::vector<Pose> path_odom_;
    std::vector<Pose> path_gt_;

    bool ready_;
    bool running_;

    int initial_iterations_counter_;

    // ARIA
    PioneerBase_ROS base_;

    // Log
    Logger* logger_;

    // Navigation
    void FollowPotentialField();
    void DrawPotGradient(double scale);

    // Mapping
    int minX_, minY_, maxX_, maxY_;



};

#endif // ROBOT_H

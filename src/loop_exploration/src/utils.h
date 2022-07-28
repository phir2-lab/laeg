/**
 * This file is part of LAEG
 *
 * Copyright 2022:
 * - Renan Maffei <rqmaffei at in dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * - Vitor Jorge
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

#ifndef UTILS_H
#define UTILS_H

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>

#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <errno.h>
#include <string.h>

#include <GL/glut.h>

enum MotionMode {MANUAL_SIMPLE, POTFIELD, ENDING};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT};

#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

float NormalizeAngleDEG(float a);
float NormalizeAngleRAD(float a);

class Pose{
    public:
        Pose();
        Pose(double a, double b, double c);

        float norm();
        float dotProduct(Pose v);
        float angleBetween(Pose v);

        friend std::ostream& operator<<(std::ostream& os, const Pose& p);

        double x, y, theta;
        bool up;
};

class Timer{
    public:
        Timer();

        void StartCounting();
        void StartLap();
        void StopCounting();
        void WaitTime(float);

        float GetTotalTime();
        float GetLapTime();

    private:
        struct timeval tstart_, tlapstart_, tnow_;
};

void DrawNumbers(std::string number, double x, double y);

float EuclideanDistance(float x1, float x2, float y1, float y2);

#endif // UTILS_H

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

#include "utils.h"

float NormalizeAngleDEG(float a)
{
    while(a>180.0)
        a -= 360.0;
    while(a<=-180.0)
        a += 360.0;
    return a;
}

float NormalizeAngleRAD(float a)
{
    while(a>M_PI)
        a -= 2*M_PI;
    while(a<=-M_PI)
        a += 2*M_PI;
    return a;
}

/////////////////////////////////
///// METHODS OF CLASS POSE /////
/////////////////////////////////

Pose::Pose()
{
    x=y=theta=0.0;
    up=false;
}

Pose::Pose(double a, double b, double c)
{
    x=a; y=b; theta=c;
}

std::ostream& operator<<(std::ostream& os, const Pose& p)
{
    os << std::setprecision(3) << "(" << p.x << ',' << p.y << ',' << p.theta << ")";
    return os;
}

float Pose::norm()
{
    float vx = this->x;
    float vy = this->y;

    return sqrt( vx*vx + vy*vy);
}

float Pose::dotProduct(Pose v)
{
    float vx = v.x;
    float vy = v.y;

    return this->x*vx + this->y*vy;
}

float Pose::angleBetween(Pose v)
{
    float num = this->dotProduct(v);
    float den = this->norm()*v.norm();
    return acos(num/den);
}

//////////////////////////////////
///// METHODS OF CLASS TIMER /////
//////////////////////////////////

Timer::Timer()
{
    StartCounting();
}

void Timer::StartCounting()
{
    gettimeofday(&tstart_, NULL);
    gettimeofday(&tlapstart_, NULL);
}

void Timer::StartLap()
{
    gettimeofday(&tlapstart_, NULL);
}

void Timer::StopCounting()
{
    gettimeofday(&tnow_, NULL);
}

float Timer::GetTotalTime()
{
    gettimeofday(&tnow_, NULL);

    if (tstart_.tv_usec > tnow_.tv_usec) {
        tnow_.tv_usec += 1000000;
        tnow_.tv_sec--;
    }

    return (float)(tnow_.tv_sec - tstart_.tv_sec) +
           ((float)tnow_.tv_usec - (float)tstart_.tv_usec)/1000000.0;
}

float Timer::GetLapTime()
{
    gettimeofday(&tnow_, NULL);

    if (tlapstart_.tv_usec > tnow_.tv_usec) {
        tnow_.tv_usec += 1000000;
        tnow_.tv_sec--;
    }
    return (float)(tnow_.tv_sec - tlapstart_.tv_sec) +
           ((float)tnow_.tv_usec - (float)tlapstart_.tv_usec)/1000000.0;
}

void Timer::WaitTime(float t)
{
    float l;
    do{
        usleep(1000);
        l = GetLapTime();
    }while(l < t);
    StartLap();
}

////////////////////////////////////////////
///  Draw Seven-Segment Digits in OpenGL ///
////////////////////////////////////////////

static double SEGMENT_WIDTH = 1 / 5.0;
static double SEGMENT_HEIGHT = 1 / 20.0;
static double SEGMENT_SHORT_WIDTH_FACTOR = 2.25 / 3.0;

static double SEGMENT_HALF_WIDTH = SEGMENT_WIDTH/2;
static double SEGMENT_QUARTER_WIDTH = SEGMENT_WIDTH/4;

static double SEGMENT_HALF_HEIGHT = SEGMENT_HEIGHT/2;

static double DIGIT_WIDTH = SEGMENT_WIDTH + SEGMENT_HEIGHT;
static double DIGIT_HALF_WIDTH = DIGIT_WIDTH/2;
static double DIGIT_QUARTER_WIDTH = DIGIT_WIDTH/4;
static double DIGIT_SPACING = DIGIT_WIDTH/4.0;

void drawLedSegment(double rotation, double x, double y)
{
    glColor3f(0.0,0.0,0.0);

    glRotated(rotation, 0, 0, 1);
    glTranslated(x, y, 0);

    glBegin(GL_POLYGON);
        glVertex2d(SEGMENT_HALF_WIDTH * SEGMENT_SHORT_WIDTH_FACTOR, SEGMENT_HALF_HEIGHT);
        glVertex2d(SEGMENT_HALF_WIDTH, 0);
        glVertex2d(SEGMENT_HALF_WIDTH * SEGMENT_SHORT_WIDTH_FACTOR, -SEGMENT_HALF_HEIGHT);
        glVertex2d(-SEGMENT_HALF_WIDTH * SEGMENT_SHORT_WIDTH_FACTOR, -SEGMENT_HALF_HEIGHT);
        glVertex2d(-SEGMENT_HALF_WIDTH, 0);
        glVertex2d(-SEGMENT_HALF_WIDTH * SEGMENT_SHORT_WIDTH_FACTOR, SEGMENT_HALF_HEIGHT);
    glEnd();

    glTranslated(-x, -y, 0);
    glRotated(-rotation, 0, 0, 1);
}

typedef struct
{
    bool a,b,c,d,e,f,g;
} SevenSegmentDigit;

void drawDigit(char digit, double x, double y)
{

    if(digit == '.')
    {
        glTranslated(x, y, 0);

        glColor3f(0.0,0.0,0.0);

        glRotated(0, 0, 0, 1);
        glTranslated(0-SEGMENT_QUARTER_WIDTH, -SEGMENT_WIDTH, 0);

        glBegin(GL_POLYGON);
            glVertex2d(SEGMENT_QUARTER_WIDTH * SEGMENT_SHORT_WIDTH_FACTOR, SEGMENT_HALF_HEIGHT);
            glVertex2d(SEGMENT_QUARTER_WIDTH, 0);
            glVertex2d(SEGMENT_QUARTER_WIDTH * SEGMENT_SHORT_WIDTH_FACTOR, -SEGMENT_HALF_HEIGHT);
            glVertex2d(-SEGMENT_QUARTER_WIDTH * SEGMENT_SHORT_WIDTH_FACTOR, -SEGMENT_HALF_HEIGHT);
            glVertex2d(-SEGMENT_QUARTER_WIDTH, 0);
            glVertex2d(-SEGMENT_QUARTER_WIDTH * SEGMENT_SHORT_WIDTH_FACTOR, SEGMENT_HALF_HEIGHT);
        glEnd();

        glTranslated(0+SEGMENT_QUARTER_WIDTH, SEGMENT_WIDTH, 0);
        glRotated(0, 0, 0, 1);

        glTranslated(-x, -y, 0);
        return;
    }


   SevenSegmentDigit ssDigit;
   switch(digit)
   {
    case '0': ssDigit.a=true;  ssDigit.b=true;  ssDigit.c=true;  ssDigit.d=true;
            ssDigit.e=true;  ssDigit.f=true;  ssDigit.g=false;
            break;
    case '1': ssDigit.a=false; ssDigit.b=true;  ssDigit.c=true;  ssDigit.d=false;
            ssDigit.e=false; ssDigit.f=false; ssDigit.g=false;
            break;
    case '2': ssDigit.a=true;  ssDigit.b=true;  ssDigit.c=false; ssDigit.d=true;
            ssDigit.e=true;  ssDigit.f=false; ssDigit.g=true;
            break;
    case '3': ssDigit.a=true;  ssDigit.b=true;  ssDigit.c=true;  ssDigit.d=true;
            ssDigit.e=false; ssDigit.f=false; ssDigit.g=true;
            break;
    case '4': ssDigit.a=false; ssDigit.b=true;  ssDigit.c=true;  ssDigit.d=false;
            ssDigit.e=false; ssDigit.f=true;  ssDigit.g=true;
            break;
    case '5': ssDigit.a=true; ssDigit.b=false; ssDigit.c=true; ssDigit.d=true;
            ssDigit.e=false; ssDigit.f=true; ssDigit.g=true;
            break;
    case '6': ssDigit.a=true; ssDigit.b=false; ssDigit.c=true; ssDigit.d=true;
            ssDigit.e=true; ssDigit.f=true; ssDigit.g=true;
            break;
    case '7': ssDigit.a=true; ssDigit.b=true; ssDigit.c=true; ssDigit.d=false;
            ssDigit.e=false; ssDigit.f=false; ssDigit.g=false;
            break;
    case '8': ssDigit.a=true; ssDigit.b=true; ssDigit.c=true; ssDigit.d=true;
            ssDigit.e=true; ssDigit.f=true; ssDigit.g=true;
            break;
    case '9': ssDigit.a=true; ssDigit.b=true; ssDigit.c=true; ssDigit.d=true;
            ssDigit.e=false; ssDigit.f=true; ssDigit.g=true;
            break;
   }

    glTranslated(x, y, 0);

    if(ssDigit.a) drawLedSegment(0, 0, SEGMENT_WIDTH);
    if(ssDigit.b) drawLedSegment(90, SEGMENT_HALF_WIDTH, -SEGMENT_HALF_WIDTH);
    if(ssDigit.c) drawLedSegment(90, -SEGMENT_HALF_WIDTH, -SEGMENT_HALF_WIDTH);
    if(ssDigit.d) drawLedSegment(0, 0, -SEGMENT_WIDTH);
    if(ssDigit.e) drawLedSegment(90, -SEGMENT_HALF_WIDTH, SEGMENT_HALF_WIDTH);
    if(ssDigit.f) drawLedSegment(90, SEGMENT_HALF_WIDTH, SEGMENT_HALF_WIDTH);
    if(ssDigit.g) drawLedSegment(0, 0, 0);

    glTranslated(-x, -y, 0);
}

void DrawNumbers(std::string number, double x, double y)
{
    glTranslated(x, y, 0);

    int max_elements = std::min(4, (int)number.size()+1);


    double number_screen_size = 0;

    for (int i = 0; i <= max_elements; i++) {
        if((char) number[i] == '.') number_screen_size += DIGIT_HALF_WIDTH;
        else number_screen_size += DIGIT_WIDTH;

        if(i < max_elements) number_screen_size += DIGIT_QUARTER_WIDTH;
    }

    double offsetX = DIGIT_HALF_WIDTH - number_screen_size/2;

    glTranslated(offsetX, 0, 0);

    double position = 0;
    for (int i = 0; i <= max_elements; i++) {
        drawDigit(number[i], position, 0);

        if((char) number[i] == '.')
            position += DIGIT_HALF_WIDTH;
        else position += DIGIT_WIDTH;

        position += DIGIT_SPACING;
    }

    glTranslated(-offsetX, 0, 0);

    glTranslated(-x, -y, 0);
}

float EuclideanDistance(float x1, float x2, float y1, float y2)
{
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

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

#include "glut_class.h"

/////////////////////////////////////////////
///// CONSTRUCTOR & CLASS INSTANTIATION /////
/////////////////////////////////////////////

GlutClass::GlutClass(){}

GlutClass* GlutClass::instance_ = 0;

GlutClass* GlutClass::instance ()
{
    if (instance_ == 0)
    {
        instance_ = new GlutClass;
    }
    return instance_;
}

//////////////////////////
///// PUBLIC METHODS /////
//////////////////////////

void GlutClass::Initialize()
{
    glut_scale_aux_ = robot_->configuration->GetInt("screen_scale_init");
    glut_x_aux_ = robot_->configuration->GetInt("screen_x_init");
    glut_y_aux_ = robot_->configuration->GetInt("screen_y_init");
    glut_window_width_ = robot_->configuration->GetInt("screen_width_init");
    glut_window_height_ = robot_->configuration->GetInt("screen_height_init");

    glut_window_half_width_ = ceil(glut_window_width_/2);
    glut_window_half_height_ = ceil(glut_window_height_/2);
    aspect_ratio_ = (double)glut_window_width_/(double)glut_window_height_;

    // Wait for the robot's initialization
    while(robot_->ready() == false)
    {
        usleep(100000);
    }

    grid_ = robot_->grid;

	int argc=0;char** argv=0;
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize (glut_window_width_,glut_window_height_);

    id_ = glutCreateWindow("Janela");

    lock_camera_on_robot_ = false;
    frame = 0;

    glClearColor (1.0, 1.0, 1.0, 0.0);
    glClear (GL_COLOR_BUFFER_BIT);

    glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glutDisplayFunc(Display);
    glutReshapeFunc(Reshape);
    glutKeyboardFunc(Keyboard);
    glutSpecialFunc(SpecialKeys);

    grid_->ChangeWindowName();
}

void GlutClass::Process()
{
    glutMainLoop();
}

void GlutClass::Terminate()
{
    robot_->plan->RequestToFinish();
}

void GlutClass::robot(Robot *r)
{
    robot_=r;
}

///////////////////////////
///// PRIVATE METHODS /////
///////////////////////////

void GlutClass::Render()
{
    if(!robot_->running())
    {
        exit(0);
    }

    int mapWidth = grid_->map_max_width();

    int scale = grid_->map_scale();

    Pose robotPose;

    robotPose = robot_->current_pose();

    double xRobot = robotPose.x*scale;
    double yRobot = robotPose.y*scale;
    double angRobot = robotPose.theta;

    double xCenter, yCenter;
    if(lock_camera_on_robot_)
    {
        xCenter=xRobot;
        yCenter=yRobot;
    }
    else {
        xCenter = 0;
        yCenter = 0;
    }

    // Update window region
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glOrtho (((int)(xCenter) + glut_x_aux_ - glut_scale_aux_)*aspect_ratio_, ((int)(xCenter) + glut_x_aux_ + glut_scale_aux_-1)*aspect_ratio_,
             (int)(yCenter) - glut_y_aux_ - glut_scale_aux_, (int)(yCenter) - glut_y_aux_ + glut_scale_aux_-1,-1, 50);
    glMatrixMode (GL_MODELVIEW);
    glClearColor(1.0, 1.0, 1.0, 0);
    glClear (GL_COLOR_BUFFER_BIT);

    // Compute limits of visible section of the grid
    int xi, yi, xf, yf;
    int x = xCenter + mapWidth/2 - 1;
    int y = mapWidth/2 - yCenter;

    xi = x + glut_x_aux_ - glut_window_half_width_;
    if( xi < 0 )
    {
        xi = 0;
        xf = glut_window_half_width_*2 - 1;
    }
    else
    {
        xf = x + glut_x_aux_ + glut_window_half_width_ - 1;
        if( xf > mapWidth - 1)
        {
            xi = mapWidth - 2*glut_window_half_width_;
            xf = mapWidth - 1;
        }
    }

    yi = y + glut_y_aux_ - glut_window_half_height_;
    if( yi < 0 )
    {
        yi = 0;
        yf = glut_window_half_height_*2 - 1;
    }
    else
    {
        yf = y + glut_y_aux_ + glut_window_half_height_ - 1;
        if( yf > mapWidth - 1)
        {
            yi = mapWidth - 2*glut_window_half_height_;
            yf = mapWidth - 1;
        }
    }

    grid_->Draw(xi, yi, xf, yf);

    grid_->DrawGraph(scale);

    // Draw robot path
    if(grid_->show_path())
    {
        //robot_->DrawOdom();
        robot_->DrawGroundTruth();
    }

    robot_->Draw(xRobot,yRobot,angRobot);

    glutSwapBuffers();
    glutPostRedisplay();

    usleep(50000);
}

/////////////////////////////////////////////////////
///// STATIC FUNCTIONS PASSED AS GLUT CALLBACKS /////
/////////////////////////////////////////////////////

void GlutClass::Display()
{
    instance_->Render();
}

void GlutClass::Reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    instance_->glut_window_width_ = w;
    instance_->glut_window_height_ = h;
    instance_->glut_window_half_width_ = ceil(w/2);
    instance_->glut_window_half_height_ = ceil(h/2);
    instance_->aspect_ratio_ = (double)w/(double)h;
}

void GlutClass::Keyboard(unsigned char key, int x, int y)
{
    // key: the value of the pressed key
    switch(key) {
        case 27:
            instance_->Terminate();
            break;
        case ' ':
            instance_->robot_->motion_mode_ = MANUAL_SIMPLE;
            instance_->robot_->Move(STOP);
            break;
        case '1':
            instance_->robot_->motion_mode_ = POTFIELD;
            break;

        case 'c': //Lock camera centered in the robot
            if(instance_->lock_camera_on_robot_ == true)
            {
                instance_->lock_camera_on_robot_ = false;
                Pose p = instance_->robot_->current_pose();
                instance_->glut_x_aux_ = p.x*instance_->grid_->map_scale();
                instance_->glut_y_aux_ = -p.y*instance_->grid_->map_scale();
            }
            else
            {
                instance_->lock_camera_on_robot_ = true;
                instance_->glut_x_aux_ = 0;
                instance_->glut_y_aux_ = 0;
            }
            break;
        case 'f':
            instance_->grid_->ChangeShowArrows();
            break;
        case 'l':
            instance_->grid_->ChangeShowFrontiers();
            break;
        case 'p':
            instance_->grid_->ChangeShowPath();
            break;
        case 'n': //view mode
            instance_->grid_->ChangeViewMode(-1);
            break;
        case 'm': //view mode
            instance_->grid_->ChangeViewMode(1);
            break;
        case 'w':
            instance_->glut_y_aux_ -= 10;
            break;
        case 'd':
            instance_->glut_x_aux_ += 10;
            break;
        case 'a':
            instance_->glut_x_aux_ -= 10;
            break;
        case 's':
            instance_->glut_y_aux_ += 10;
            break;
        case '-':
            instance_->glut_scale_aux_ += 10;
            if(instance_->glut_scale_aux_ > instance_->grid_->map_max_width()/2)
                instance_->glut_scale_aux_ = instance_->grid_->map_max_width()/2;
            break;
        case '+': 
            instance_->glut_scale_aux_ -= 10;
            if(instance_->glut_scale_aux_ < instance_->grid_->map_scale())
                instance_->glut_scale_aux_ = instance_->grid_->map_scale();
            break;
        default:
            break;
    }
}

void GlutClass::SpecialKeys(int key, int x, int y)
{
    // key: the value of the pressed key
    if(instance_->robot_->motion_mode_ == MANUAL_SIMPLE)
        switch(key)
        {
            case GLUT_KEY_UP:
                instance_->robot_->Move(FRONT);
                break;
            case GLUT_KEY_RIGHT:
                instance_->robot_->Move(RIGHT);
                break;
            case GLUT_KEY_LEFT:
                instance_->robot_->Move(LEFT);
                break;
            case GLUT_KEY_DOWN:
                instance_->robot_->Move(BACK);
                break;
            default:
                break;
        }
}


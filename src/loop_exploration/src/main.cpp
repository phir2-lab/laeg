/**
 * This file is part of LAEG
 *
 * Copyright 2022:
 * - Renan Maffei <rqmaffei at in dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * - Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
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

#include <pthread.h>
#include <iostream>
#include <string.h>

#include "robot.h"
#include "planning.h"
#include "glut_class.h"
#include "configuration.h"
#include "utils.h"
#include "logger.h"

pthread_mutex_t* p_mutex;

void* StartRobotThread (void* ref)
{
    Robot* robot = (Robot*) ref;

    robot->Initialize();

    while(robot->running())
        robot->Run();

    sleep(2000000); // Wait 2 seconds

    return nullptr;
}

void* StartGlutThread (void* ref)
{
    GlutClass* glut = GlutClass::instance();

    glut->robot((Robot*) ref);

    glut->Initialize();

    glut->Process();

    return nullptr;
}

void* StartPlanningThread (void* ref)
{
    Robot* robot = (Robot*) ref;

    float iteration_interval_time = 1.0/robot->configuration->GetFloat("planning_iterations_per_second");

    while(!robot->ready()) // Planning waits for the robot
        usleep(100000);

    robot->plan->Initialize();
    robot->motion_mode_ = POTFIELD;

    Timer planning_timer_;

    while(robot->running())
    {
        planning_timer_.StartLap();
        if(!robot->plan->Run())
        {
            robot->plan->SaveMap();
            robot->motion_mode_ = ENDING;
        }
        planning_timer_.WaitTime(iteration_interval_time);
    }

    return nullptr;
}

int main(int argc, char* argv[])
{
    Configuration *configuration = new Configuration();

    if(argc > 1)
    {
        if(!configuration->Load(argv[1]))
        {
            std::cout << std::endl << "[ERROR] Can't read configuration file!" << std::endl;
            return 0;
        }
    }
    else
    {
        std::cout << std::endl << "[ERROR] Can't find configuration file!" << std::endl;
        return 0;
    }

    Logger* logger = new Logger();

    if(configuration->GetBool("save_log"))
    {
        if(argc > 2)
            logger->folder_path(argv[2]);
        else
            logger->folder_path(configuration->GetString("output_address"));
        logger->Enable();
    }

    Robot* robot;
    robot = new Robot(configuration, logger);

    pthread_t robot_thread, glut_thread, planning_thread;
    p_mutex = new pthread_mutex_t;
    pthread_mutex_unlock(p_mutex);

    pthread_create(&(robot_thread),nullptr,StartRobotThread,(void*)robot);
    pthread_create(&(glut_thread),nullptr,StartGlutThread,(void*)robot);
    pthread_create(&(planning_thread),nullptr,StartPlanningThread,(void*)robot);

    pthread_join(robot_thread, nullptr);
    pthread_join(glut_thread, nullptr);
    pthread_join(planning_thread, nullptr);

    return 0;
}


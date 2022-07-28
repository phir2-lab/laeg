/**
 * This file is part of LAEG
 *
 * Copyright 2022 Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
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

#include <iostream>

#include "logger.h"

Logger::Logger()
{
    start_time_ = chrono::steady_clock::now();
    enable_ = false;
    folder_path_ = "";
}

void Logger::Enable()
{
    enable_ = true;

    this->Initilize();
}

void Logger::folder_path(string folder_path)
{
    folder_path_ = folder_path;
}

void Logger::Initilize()
{
    this->InitializeLogOdometry();
    this->InitializeLogGroundTruth();
    this->InitializeLogSLAMPose();
    this->InitializeLogTimeExplorationIteration();
    this->InitializeLogTimeRailsExtraction();
    this->InitializeLogEntropy();
    this->InitializeLogTimeGraphExtraction();
    this->InitializeLogTimeNextGoal();
    this->InitializeLogTimePath();
}

double Logger::get_time()
{
    return chrono::duration_cast<std::chrono::duration<double> >(chrono::steady_clock::now() - start_time_).count();
}

void Logger::InitializeLogOdometry()
{
    if(!enable_) return;

    std::stringstream address;
    address << folder_path_ << "/logs/odometry.txt";
    address >> odometry_file_;

    cout << "[INFO] Storing the odometry to: " << odometry_file_ << endl;

    ofstream file;
    file.open(odometry_file_);
    file << "time(s);x;y;theta" << endl;
    file.close();
}

void Logger::LogOdometry(float x, float y, float theta)
{
    if(!enable_) return;

    ofstream file;
    file.open(odometry_file_, fstream::app);
    file << fixed << this->get_time() << ";" << x << ";" << y << ";" << theta << endl;
    file.close();
}

void Logger::InitializeLogGroundTruth()
{
    if(!enable_) return;

    std::stringstream address;
    address << folder_path_ << "/logs/ground_truth.txt";
    address >> ground_truth_file_;

    cout << "[INFO] Storing the ground truth to: " << ground_truth_file_ << endl;

    ofstream file;
    file.open(ground_truth_file_);
    file << "time(s);x;y;theta" << endl;
    file.close();
}

void Logger::LogGroundTruth(float x, float y, float theta)
{
    if(!enable_) return;

    ofstream file;
    file.open(ground_truth_file_, fstream::app);
    file << fixed << this->get_time() << ";" << x << ";" << y << ";" << theta << endl;
    file.close();
}

void Logger::InitializeLogSLAMPose()
{
    if(!enable_) return;

    std::stringstream address;
    address << folder_path_ << "/logs/slam_pose.txt";
    address >> pose_file_;

    cout << "[INFO] Storing the SLAM's estimated pose to: " << pose_file_ << endl;

    ofstream file;
    file.open(pose_file_);
    file << "time(s);x;y;theta" << endl;
    file.close();
}

void Logger::LogSLAMPose(float x, float y, float theta)
{
    if(!enable_) return;

    ofstream file;
    file.open(pose_file_, fstream::app);
    file << fixed << this->get_time() << ";" << x << ";" << y << ";" << theta << endl;
    file.close();
}

void Logger::InitializeLogTimeExplorationIteration()
{
    if(!enable_) return;

    std::stringstream address;
    address << folder_path_ << "/logs/time_exploration_iteration.txt";
    address >> time_exploration_iteration_file_;

    cout << "[INFO] Storing the planning step duration to: " << time_exploration_iteration_file_ << endl;

    ofstream file;
    file.open(time_exploration_iteration_file_);
    file << "time(s);duration(ms)" << endl;
    file.close();
}

void Logger::StartLogTimeExplorationIteration()
{
    if(!enable_) return;
    timer_exploration_iteration_ = chrono::steady_clock::now();
}

void Logger::LogTimeExplorationIteration()
{
    if(!enable_) return;

    double diff = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - timer_exploration_iteration_).count();

    ofstream file;
    file.open(time_exploration_iteration_file_, fstream::app);

    file << fixed << this->get_time() << ";" << diff << endl;

    file.close();
}

void Logger::InitializeLogTimeRailsExtraction()
{
    if(!enable_) return;

    std::stringstream address;
    address << folder_path_ << "/logs/time_rails_extraction.txt";
    address >> time_rails_extraction_file_;

    cout << "[INFO] Storing the rails extraction duration to: " << time_rails_extraction_file_ << endl;

    ofstream file;
    file.open(time_rails_extraction_file_);
    file << "time(s);duration(ms)" << endl;
    file.close();
}

void Logger::StartLogTimeRailsExtraction()
{
    if(!enable_) return;
    timer_rails_extraction_ = chrono::steady_clock::now();
}

void Logger::LogTimeRailsExtraction()
{
    if(!enable_) return;

    double diff = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - timer_rails_extraction_).count();

    ofstream file;
    file.open(time_rails_extraction_file_, fstream::app);

    file << fixed << this->get_time() << ";" << diff << endl;

    file.close();
}

void Logger::InitializeLogTimeGraphExtraction()
{
    if(!enable_) return;

    std::stringstream address;
    address << folder_path_ << "/logs/time_graph_extraction.txt";
    address >> time_graph_extraction_file_;

    cout << "[INFO] Storing the Graph extraction duration to: " << time_graph_extraction_file_ << endl;

    ofstream file;
    file.open(time_graph_extraction_file_);
    file << "time(s);duration(ms)" << endl;
    file.close();
}

void Logger::StartLogTimeGraphExtraction()
{
    if(!enable_) return;
    timer_graph_extraction_ = chrono::steady_clock::now();
}

void Logger::LogTimeGraphExtraction()
{
    if(!enable_) return;

    double diff = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - timer_graph_extraction_).count();

    ofstream file;
    file.open(time_graph_extraction_file_, fstream::app);

    file << fixed << this->get_time() << ";" << diff*0.001 << endl;

    file.close();
}

void Logger::InitializeLogTimeNextGoal()
{
    if(!enable_) return;

    std::stringstream address;
    address << folder_path_ << "/logs/time_next_goal.txt";
    address >> time_next_goal_file_;

    cout << "[INFO] Storing the Graph extraction duration to: " << time_next_goal_file_ << endl;

    ofstream file;
    file.open(time_next_goal_file_);
    file << "time(s);duration(ms)" << endl;
    file.close();
}

void Logger::StartLogTimeNextGoal()
{
    if(!enable_) return;
    timer_next_goal_ = chrono::steady_clock::now();
}

void Logger::LogTimeNextGoal()
{
    if(!enable_) return;

    double diff = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - timer_next_goal_).count();

    ofstream file;
    file.open(time_next_goal_file_, fstream::app);

    file << fixed << this->get_time() << ";" << diff*0.001 << endl;

    file.close();
}

void Logger::InitializeLogTimePath()
{
    if(!enable_) return;

    std::stringstream address;
    address << folder_path_ << "/logs/time_path.txt";
    address >> time_path_file_;

    cout << "[INFO] Storing the Graph extraction duration to: " << time_path_file_ << endl;

    ofstream file;
    file.open(time_path_file_);
    file << "time(s);duration(ms)" << endl;
    file.close();
}

void Logger::StartLogTimePath()
{
    if(!enable_) return;
    timer_path_ = chrono::steady_clock::now();
}

void Logger::LogTimePath()
{
    if(!enable_) return;

    double diff = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - timer_path_).count();

    ofstream file;
    file.open(time_path_file_, fstream::app);

    file << fixed << this->get_time() << ";" << diff << endl;

    file.close();
}

void Logger::SaveMap(vector<vector<int> > map, int root_y, int root_x)
{
    if(!enable_) return;

    string map_file_;

    std::stringstream address;
    address << folder_path_ << "/logs/map.txt";
    address >> map_file_;

    cout << "[INFO] Storing the map to: " << map_file_ << endl;

    ofstream file;
    file.open(map_file_);

    file << "Size (y,x):" << endl << map.size() << ";" << map[0].size() << endl;

    file << "Roots (y,x):" << endl << root_y << ";" << root_x << endl;


    for(int i = 0; i < map.size(); i++)
    {
        for(int ii = 0; ii < map[0].size(); ii++)
        {
            file << map[i][ii] << ";";
        }
        file << endl;
    }

    file.close();
}

void Logger::InitializeLogEntropy()
{
    if(!enable_) return;

    std::stringstream address;
    address << folder_path_ << "/logs/entropy.txt";
    address >> entropy_file_;

    cout << "[INFO] Storing the particle entropy to: " << entropy_file_ << endl;

    ofstream file;
    file.open(entropy_file_);
    file << "time(s);entropy" << endl;
    file.close();
}

void Logger::LogEntropy(float value)
{
    if(!enable_) return;

    ofstream file;
    file.open(entropy_file_, fstream::app);
    file << fixed << this->get_time() << ";" << value << endl;
    file.close();
}



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

#ifndef LOGGER_H
#define LOGGER_H

#include <string.h>
#include <sstream>

#include <fstream>
#include <chrono>
#include <vector>

using namespace std;

class Logger
{
public:
    Logger();

    void Initilize();

    void Enable();

    void InitializeLogOdometry();
    void LogOdometry(float x, float y, float theta);

    void InitializeLogGroundTruth();
    void LogGroundTruth(float x, float y, float theta);

    void InitializeLogSLAMPose();
    void LogSLAMPose(float x, float y, float theta);

    void InitializeLogTimeExplorationIteration();
    void StartLogTimeExplorationIteration();
    void LogTimeExplorationIteration();

    void InitializeLogTimeRailsExtraction();
    void StartLogTimeRailsExtraction();
    void LogTimeRailsExtraction();

    void InitializeLogTimeGraphExtraction();
    void StartLogTimeGraphExtraction();
    void LogTimeGraphExtraction();

    void InitializeLogTimeNextGoal();
    void StartLogTimeNextGoal();
    void LogTimeNextGoal();

    void InitializeLogTimePath();
    void StartLogTimePath();
    void LogTimePath();

    void SaveMap(vector<vector<int> > map, int root_y, int root_x);

    void InitializeLogEntropy();
    void LogEntropy(float value);

    void folder_path(string folder_path);

    double get_time();

private:

    string folder_path_;
    string odometry_file_;
    string pose_file_;
    string ground_truth_file_;
    string entropy_file_;
    string time_exploration_iteration_file_;
    string time_rails_extraction_file_;
    string time_graph_extraction_file_;
    string time_next_goal_file_;
    string time_path_file_;

    bool enable_;

    std::chrono::steady_clock::time_point start_time_;

    std::chrono::steady_clock::time_point timer_exploration_iteration_;
    std::chrono::steady_clock::time_point timer_rails_extraction_;
    std::chrono::steady_clock::time_point timer_graph_extraction_;
    std::chrono::steady_clock::time_point timer_next_goal_;
    std::chrono::steady_clock::time_point timer_path_;

};

#endif // LOGGER_H

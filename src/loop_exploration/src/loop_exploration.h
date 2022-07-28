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

#ifndef LOOP_EXPLORATION_H
#define LOOP_EXPLORATION_H

#include <vector>
#include <queue>
#include <iostream>
#include <math.h>

#include "grid.h"
#include "configuration.h"
#include "laeg.h"
#include "logger.h"

# define M_PI 3.14159265358979323846  /* pi */

typedef struct
{
     Cell* cell;
     int distance;
     Node* father;
     bool unknown, unvisited;
} RequesterGraphMerge;

typedef struct
{
     Cell* cell;
     int distance;
     Node* father;
     vector<RequesterGraphMerge*> requested_connections;
     int graph_code;
     bool unknown, unvisited;
     bool blocked_way;
} RailsToGraphHelper;

class LoopExploration
{
public:
    LoopExploration(Configuration* config, Logger* logger);

    void Initialize(Grid* grid);

    bool Run(Pose *current_pose);

    bool DefineNextGoal();

    bool FindLocalGoal();

    Cell* FindNearestCenterCell(Cell *root, Cell* closest_to = NULL);

    float SmallerhypotheticalDistance();
    float BiggerhypotheticalDistance();
    float AveragehypotheticalDistance();
    float StandardDeviationhypotheticalDistance(float average);

    float SmallerFromRobotDistance(vector<Node *> candidates);
    float AverageFromRobotDistance(vector<Node *> candidates);
    float StandardDeviationFromRobotDistance(vector<Node *> candidates, float average);

    // Navigation
    void InitializePotential();
    double IteratePotential();
    void UpdateGradient();

private:
    bool ready_;

    int plan_counter_;
    int size_offset_;

    Grid* grid_;

    Node* goal_;

    Cell* robot_cell_;
    Cell* robot_rails_cell_;
    Cell* old_robot_rails_cell_;
    Cell* local_goal_cell_;

    void DefineGraph();
    void UpdateDistanceFromGoalToRails(bool outer_allowed);
    void RemoveCenterCellsAtTinySpace();
    bool HasObstacleNeighbor(Cell *cell);
    bool HasUnvisitedNeighbor(Cell *cell);
    bool HasUnknowNeighbor(Cell *cell);
    bool HasNearFrontier(Cell *cell);

    std::vector< std::pair<Node*, std::vector< std::pair<Node*, int> > > > hypothetical_distances_;
    std::vector< std::pair<Node*, std::vector< std::pair<Node*, int> > > > loop_distances_;

    // Navigation
    int potential_window_half_;
    int actual_potential_window_half_;

    // Parameters
    int local_goal_radius_;
    int max_goal_radius_;
    int min_dist_to_obstacles_from_unknown_cells_;
    int min_dist_to_obstacles_from_free_cells_;
    float alpha_;

    // Misc
    Configuration* config_;
    Logger* logger_;

};

#endif // LOOP_EXPLORATION_H

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

#include "planning.h"

Planning::Planning(Configuration* configuration, Logger *logger)
{
    inflate_obstacles_pad_size_ = configuration->GetInt("inflate_obstacles_pad_size");

    free_threshold_ = configuration->GetInt("free_threshold");
    obstacle_threshold_ = configuration->GetInt("obstacle_threshold");

    external_border_ = configuration->GetInt("external_border");

    exploration_ = new LoopExploration(configuration, logger);

    logger_ = logger;

    iteration_ = 0;

    new_pose_ = new Pose();
    new_pose_->x = 0;
    new_pose_->y = 0;

    new_update_window_.min_x = new_update_window_.min_y = 1000;
    new_update_window_.max_x = new_update_window_.max_y = -1000;

    requested_to_stop_ = false;
}

Planning::~Planning()
{}

void Planning::grid(Grid *g)
{
    grid_ = g;
}

void Planning::SaveMap()
{
    // Waiting to avoid to change and use the grid at the same time
    while(!grid_->ready_to_update())
        usleep(10000);

    Cell* c;

    int min_x, min_y, max_x, max_y;

    min_x = std::min(grid_->map_limits.min_x-grid_->pad_x, grid_->old_map_limits.min_x-grid_->pad_x)-inflate_obstacles_pad_size_;
    min_y = std::min(grid_->map_limits.min_y-grid_->pad_y, grid_->old_map_limits.min_y-grid_->pad_y)-inflate_obstacles_pad_size_;
    max_x = std::max(grid_->map_limits.max_x-grid_->pad_x, grid_->old_map_limits.max_x-grid_->pad_x)+inflate_obstacles_pad_size_;
    max_y = std::max(grid_->map_limits.max_y-grid_->pad_y, grid_->old_map_limits.max_y-grid_->pad_y)+inflate_obstacles_pad_size_;

    vector<vector<int> > vector_map;

    for(int j = max_y; j >= min_y; j--)
    {
        vector<int> line;
        for(int i = min_x; i <= max_x; i++)
        {
            c = grid_->cell(i,j);
            if(c->slam_value < free_threshold_ && c->slam_value >= 0)
                line.push_back(1);
            else if(c->slam_value > obstacle_threshold_)
                line.push_back(0);
            else
                line.push_back(-1);
        }
        vector_map.push_back(line);
    }

    logger_->SaveMap(vector_map, max_y, -min_x+1);

    grid_->ReleaseToUpdate();
}

void Planning::SetNewRobotPose(Pose p)
{
    new_pose_->x = (int)(p.x*grid_->map_scale());
    new_pose_->y = (int)(p.y*grid_->map_scale());
}

void Planning::Initialize()
{
    exploration_->Initialize(grid_);

    update_window_.min_x = 0;
    update_window_.max_x = 0;
    update_window_.min_y = 0;
    update_window_.max_y = 0;
}

bool Planning::Run()
{
    current_pose_ = new_pose_;

    if(requested_to_stop_)
    {
        cout << "[EXPLORATION] Ending by user request." << endl;
        return false;
    }

    // Waiting to avoid to change and use the grid at the same time
    while(!grid_->ready_to_update())
        usleep(10000);

    if(grid_->Changed())
    {
        update_window_.min_x = grid_->map_limits.min_x-grid_->pad_x-inflate_obstacles_pad_size_;
        update_window_.max_x = grid_->map_limits.max_x-grid_->pad_x+inflate_obstacles_pad_size_;
        update_window_.min_y = grid_->map_limits.min_y-grid_->pad_y-inflate_obstacles_pad_size_;
        update_window_.max_y = grid_->map_limits.max_y-grid_->pad_y+inflate_obstacles_pad_size_;

        UpdateCellsTypes();
        ExpandObstacles();

        logger_->StartLogTimeRailsExtraction();
        UpdateCenterCells(false, true);
        logger_->LogTimeRailsExtraction();

        grid_->old_map_limits.min_x = grid_->map_limits.min_x;
        grid_->old_map_limits.min_y = grid_->map_limits.min_y;
        grid_->old_map_limits.max_x = grid_->map_limits.max_x;
        grid_->old_map_limits.max_y = grid_->map_limits.max_y;

        grid_->old_map_visited_limits.min_x = grid_->map_visited_limits.min_x;
        grid_->old_map_visited_limits.min_y = grid_->map_visited_limits.min_y;
        grid_->old_map_visited_limits.max_x = grid_->map_visited_limits.max_x;
        grid_->old_map_visited_limits.max_y = grid_->map_visited_limits.max_y;

    }
    else //Not run when the map does not change
    {
        grid_->ReleaseToUpdate();
        return true; 
    }

    logger_->StartLogTimeExplorationIteration();

    if(!exploration_->Run(current_pose_))
    {
        logger_->LogTimeExplorationIteration();
        cout << "[EXPLORATION] Ending the exploration." << endl;
        grid_->ReleaseToUpdate();
        return false;
    }

    logger_->LogTimeExplorationIteration();

    iteration_++;
    grid_->plan_iteration = iteration_;

    grid_->ReleaseToUpdate();

    usleep(10000);

    return true;
}

void Planning::UpdateCellsTypes()
{
    Cell* c;

    int min_x, min_y, max_x, max_y;

    min_x = std::min(grid_->map_limits.min_x-grid_->pad_x, grid_->old_map_limits.min_x-grid_->pad_x)-inflate_obstacles_pad_size_;
    min_y = std::min(grid_->map_limits.min_y-grid_->pad_y, grid_->old_map_limits.min_y-grid_->pad_y)-inflate_obstacles_pad_size_;
    max_x = std::max(grid_->map_limits.max_x-grid_->pad_x, grid_->old_map_limits.max_x-grid_->pad_x)+inflate_obstacles_pad_size_;
    max_y = std::max(grid_->map_limits.max_y-grid_->pad_y, grid_->old_map_limits.max_y-grid_->pad_y)+inflate_obstacles_pad_size_;

    for(int i = min_x; i <= max_x; i++)
    {
        for(int j = min_y; j <= max_y; j++)
        {
            c = grid_->cell(i,j);
            if(c->slam_value<0)
            {
                // Count neigboors to fill free area
                int free_neigbors = 0;
                for(int x=-1; x<=1; x++)
                {
                    for(int y=-1; y<=1; y++)
                    {
                        float cell_slam_value = grid_->cell(i+x,j+y)->slam_value;
                        if(cell_slam_value >= 0 && cell_slam_value < free_threshold_)
                            free_neigbors++;
                    }
                }
                if(free_neigbors >= 6)
                    c->type = FREE;
                else
                {
                    c->type = UNEXPLORED;
                    c->potential = 0.0;
                }
            }
            else if(c->slam_value < free_threshold_)
            {
                c->type = FREE;
            }
            else if(c->slam_value > obstacle_threshold_)
            {
                c->type = OCCUPIED;
                c->potential = 1.0;
            }
        }
    }
}

void Planning::ExpandObstacles()
{
    Cell *c, *n;

    int min_x, min_y, max_x, max_y;

    min_x = std::min(grid_->map_limits.min_x-grid_->pad_x, grid_->old_map_limits.min_x-grid_->pad_x);
    min_y = std::min(grid_->map_limits.min_y-grid_->pad_y, grid_->old_map_limits.min_y-grid_->pad_y);
    max_x = std::max(grid_->map_limits.max_x-grid_->pad_x, grid_->old_map_limits.max_x-grid_->pad_x);
    max_y = std::max(grid_->map_limits.max_y-grid_->pad_y, grid_->old_map_limits.max_y-grid_->pad_y);

    for(int i = min_x; i <= max_x; i++)
    {
        for(int j = min_y; j <= max_y; j++)
        {
            c = grid_->cell(i,j);

            if(c->type == OCCUPIED)
            {
                for(int x=i-inflate_obstacles_pad_size_;x<=i+inflate_obstacles_pad_size_;x++)
                {
                    for(int y=j-inflate_obstacles_pad_size_;y<=j+inflate_obstacles_pad_size_;y++)
                    {
                        n = grid_->cell(x,y);
                        if(n->type == FREE || n->type == UNEXPLORED)
                        {
                            n->type = NEAROBSTACLE;
                            n->potential = 1;
                        }
                    }
                }
            }

        }
    }
}

void Planning::UpdateCenterCells(bool remove_spurs, bool traverse_unexplored)
{
    Cell *c;

    int width = update_window_.max_x - update_window_.min_x+1 + 2*external_border_;
    int height = update_window_.max_y - update_window_.min_y+1 + 2*external_border_;

    unsigned char* map;
    map = new unsigned char[width*height];
    unsigned char* result;
    result = new unsigned char[width*height];

    int counter=0;
    int cellsT=0, cellsF=0;
    for (int y = update_window_.max_y+external_border_; y >= update_window_.min_y-external_border_; y--)
    {
        for (int x = update_window_.min_x-external_border_; x <= update_window_.max_x+external_border_; x++)
        {

            c=grid_->cell(x,y);

            if(c->type == OCCUPIED || c->type == NEAROBSTACLE ||
               (!traverse_unexplored && c->type == UNEXPLORED) ||
               x==update_window_.max_x+external_border_ || x==update_window_.min_x-external_border_ ||
               y==update_window_.max_y+external_border_ || y==update_window_.min_y-external_border_ )
            {
                map[counter]=0x00;
            }
            else
            {
                map[counter]=0xFF;
            }
            counter++;
        }
    }

    Thinning thinning;
    thinning.Thinning_Using_GUOandHALL(map, result, width, height);

    if(remove_spurs)
        thinning.spur_removal(40, result, width, height);

    // Let's copy the thinning to the map
    counter=0;
    cellsT=0, cellsF=0;
    for (int y = update_window_.max_y+external_border_; y >= update_window_.min_y-external_border_; y--)
    {
        for (int x = update_window_.min_x-external_border_; x <= update_window_.max_x+external_border_; x++)
        {

            c=grid_->cell(x,y);

            if(result[counter] != 0xFF)
            {
                c->is_center=false;
            }
            else
            {
                c->is_center=true;
            }

            counter++;
        }
    }

    delete [] map;
    delete [] result;

}

void Planning::RequestToFinish()
{
    requested_to_stop_ = true;
}

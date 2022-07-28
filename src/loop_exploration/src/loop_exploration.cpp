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

#include "loop_exploration.h"

//eight-neighbor offset
int offset[][8]={{-1,  1},
                 { 0,  1},
                 { 1,  1},
                 { 1,  0},
                 { 1, -1},
                 { 0, -1},
                 {-1, -1},
                 {-1,  0}
                };

LoopExploration::LoopExploration(Configuration* config, Logger* logger)
{
    ready_ = false;
    
    config_ = config;
    logger_ = logger;

    potential_window_half_ = config->GetInt("potential_window_size")/2;
    max_goal_radius_ = config->GetInt("max_goal_radius");
    local_goal_radius_ = max_goal_radius_;

    min_dist_to_obstacles_from_unknown_cells_ = config->GetInt("min_dist_to_obstacles_from_unknown_cells");
    min_dist_to_obstacles_from_free_cells_ = config->GetInt("min_dist_to_obstacles_from_free_cells");

    alpha_ = config->GetFloat("alpha");
}

void LoopExploration::Initialize(Grid *grid)
{
    double radius;
    radius=1.0*(double)grid->map_scale();

    radius=2*(double)grid->map_scale();

    size_offset_ = 8;

    grid_ = grid;

    old_robot_rails_cell_ = NULL;
}

bool LoopExploration::Run(Pose *current_pose)
{

    robot_cell_ = grid_->cell(current_pose->x, current_pose->y);

    robot_rails_cell_ = FindNearestCenterCell(robot_cell_, old_robot_rails_cell_);
    
    if(robot_rails_cell_ == NULL)
    {   
        cout << "[EXPLORATION] Cannot find robot in rails" << endl;
        return false;
    }

    if(DefineNextGoal())
    {
        logger_->StartLogTimePath();
        if(FindLocalGoal())
        {
            InitializePotential();
            for(int i=0; i<100; i++)
                IteratePotential();
            UpdateGradient();
        }
        logger_->LogTimePath();
    }
    else
    {
        return false;
    }

    old_robot_rails_cell_ = robot_rails_cell_;

    return true;
}

bool LoopExploration::FindLocalGoal()
{
    Cell *root, *c, *nc, *local_goal = nullptr;

    std::vector<Cell*> actualCells;
    std::vector<Cell*> futureCells;

    std::vector<Cell*> candidateCells;

    actual_potential_window_half_ = potential_window_half_;

    plan_counter_++;
    robot_rails_cell_->plan_counter=plan_counter_;

    actualCells.push_back(robot_rails_cell_);

    int max_radius = 0;

    while(max_radius <= max_goal_radius_)
    {
        max_radius++;

        futureCells.clear();

        for(size_t c1=0; c1<actualCells.size();c1++)
        {
            c = actualCells[c1];

            if(max_radius >= sqrt(pow(robot_rails_cell_->x - c->x, 2) + pow(robot_rails_cell_->y - c->y, 2)))
            {
                if(goal_)
                    if(c->x == goal_->cell()->x && c->y == goal_->cell()->y)
                    {
                        local_goal_cell_ = goal_->cell();
                        return true;
                    }

                if(c->is_center)
                {
                    if(local_goal)
                    {
                        if(local_goal->distance_from_goal > c->distance_from_goal)
                            local_goal = c;
                    }
                    else local_goal = c;
                }

                for(int n=0;n<size_offset_;n++)
                {
                    nc = grid_->cell(c->x+offset[n][0],c->y+offset[n][1]);
                    if(nc->type == OCCUPIED || nc->plan_counter == plan_counter_)
                        continue;

                    futureCells.push_back(nc);
                    nc->plan_counter = plan_counter_;

                }
            }
            else
            {
                futureCells.push_back(c);
            }
        }

        actualCells = futureCells;
    }

    if(!local_goal)
    {
        local_goal_cell_ = NULL;
        return false;
    }

    local_goal_cell_ = local_goal;
    return true;
}

Cell* LoopExploration::FindNearestCenterCell(Cell *root, Cell* closest_to)
{
    Cell *c, *nc;

    std::vector<Cell*> actual_cells;
    std::vector<Cell*> future_cells;

    std::vector<Cell*> candidate_cells;

    plan_counter_++;
    root->plan_counter = plan_counter_;

    actual_cells.push_back(root);

    float allowed_distance = 0;
    bool found = false;

    vector<Cell*> candidates;

    int max_distance = 500;
    while(!found && (allowed_distance <= max_distance))
    {
        allowed_distance += 0.25;
        future_cells.clear();

        for(size_t c1=0; c1<actual_cells.size();c1++)
        {
            c = actual_cells[c1];

            if(sqrt(pow(root->x - c->x, 2) + pow(root->y - c->y, 2)) <= allowed_distance)
            {

                if(c->is_center)
                {
                    found = true;
                    candidate_cells.push_back(c);
                }

                if(!found)
                {
                    for(int n=0;n<8;n++)
                    {
                        nc = grid_->cell(c->x+offset[n][0],c->y+offset[n][1]);

                        if(nc->type == OCCUPIED || nc->plan_counter == plan_counter_)
                            continue;

                        future_cells.push_back(nc);
                        nc->plan_counter = plan_counter_;
                    }
                }
            }
            else
            {
                future_cells.push_back(c);
            }
        }

        if(!future_cells.empty())
            actual_cells = future_cells;
    }

    if(candidate_cells.empty())
        return NULL;

    if(candidate_cells.size() == 1 || closest_to == NULL)
        return candidate_cells[0];

    c = candidate_cells[0];
    float small_dist_to_old = sqrt(pow(closest_to->x*c->x, 2) + pow(closest_to->y*c->y, 2));
    for(size_t i = 1; i < candidate_cells.size(); i++)
    {
        nc = candidate_cells[i];
        float dist = sqrt(pow(closest_to->x*nc->x, 2) + pow(closest_to->y*nc->y, 2));
        if(dist < small_dist_to_old)
        {
            dist = small_dist_to_old;
            c = nc;
        }
    }

    return c;
}

float LoopExploration::SmallerhypotheticalDistance()
{
    float smaller_size = INFINITE_EDGE;
    for(size_t i = 0; i < hypothetical_distances_.size(); i++)
    {
        for(size_t ii = i+1; ii < hypothetical_distances_.size(); ii++)
        {
            if(hypothetical_distances_[i].second[ii].second < smaller_size && hypothetical_distances_[i].second[ii].second < INFINITE_EDGE)
                smaller_size = hypothetical_distances_[i].second[ii].second;
        }
    }
    return smaller_size;
}

// Unused function
float LoopExploration::BiggerhypotheticalDistance()
{
    float bigger_size = 0;
    for(size_t i = 0; i < hypothetical_distances_.size(); i++)
    {
        for(size_t ii = i+1; ii < hypothetical_distances_.size(); ii++)
        {
            if(hypothetical_distances_[i].second[ii].second > bigger_size && hypothetical_distances_[i].second[ii].second < INFINITE_EDGE)
                bigger_size = hypothetical_distances_[i].second[ii].second;
        }
    }
    return bigger_size;
}

float LoopExploration::AveragehypotheticalDistance()
{
    float total = 0;
    int amount = 0;
    for(size_t i = 0; i < hypothetical_distances_.size(); i++)
    {
        for(size_t ii = i+1; ii < hypothetical_distances_.size(); ii++)
        {
            if(hypothetical_distances_[i].second[ii].second < INFINITE_EDGE)
            {
                total += hypothetical_distances_[i].second[ii].second;
                amount++;
            }

        }
    }

    if(total == 0) return 0;

    return total/(float)amount;
}

float LoopExploration::StandardDeviationhypotheticalDistance(float average)
{
    float sum = 0;
    int amount = 0;

    for(size_t i = 0; i < hypothetical_distances_.size(); i++)
    {
        for(size_t ii = i+1; ii < hypothetical_distances_.size(); ii++)
        {
            if(hypothetical_distances_[i].second[ii].second < INFINITE_EDGE)
            {
                sum += pow(hypothetical_distances_[i].second[ii].second - average, 2);
                amount++;
            }

        }
    }

    if(sum == 0) return 0;

    return sqrt(sum/amount);;
}

float LoopExploration::SmallerFromRobotDistance(vector<Node *> candidates)
{
    float small = -1;
    for(size_t i = 0; i < candidates.size(); i++)
    {
        if(candidates[i]->cell()->distance_from_robot < small || small == -1)
            small = candidates[i]->cell()->distance_from_robot;
    }
    return small;
}

float LoopExploration::AverageFromRobotDistance(vector<Node *> candidates)
{
    float total = 0;
    for(size_t i = 0; i < candidates.size(); i++)
        total += candidates[i]->cell()->distance_from_robot;

    return total/(float)candidates.size();
}

float LoopExploration::StandardDeviationFromRobotDistance(vector<Node *> candidates, float average)
{
    float sum = 0;

    for(size_t i = 0; i < candidates.size(); i++)
        sum += pow(candidates[i]->cell()->distance_from_robot - average, 2);

    return sqrt(sum/(float)candidates.size());
}

bool LoopExploration::DefineNextGoal()
{
    this->RemoveCenterCellsAtTinySpace();
    
    logger_->StartLogTimeGraphExtraction();
    this->DefineGraph();
    logger_->LogTimeGraphExtraction();

    logger_->StartLogTimeNextGoal();
    vector<Node*> candidates = grid_->laeg()->candidates();

    if(candidates.size() < 1 && !ready_)
        return true;
    else
        ready_ = true;

    vector<float> min_dist_candidate;

    hypothetical_distances_ = grid_->laeg()->ComputehypotheticalDistancesUsingChebyshevRails();
    loop_distances_ = grid_->laeg()->ComputeLoopDistances();

    int interesting_frontiers = 0;
    float max_attractiveness = 0;
    float smaller_hypothetical_distance = SmallerhypotheticalDistance();
    float average_of_hypothetical_distances = AveragehypotheticalDistance();
    float variance_of_hypothetical_distances = pow(StandardDeviationhypotheticalDistance(average_of_hypothetical_distances),2);
    float max_hypothetical_distances_density = (1/sqrt(2*M_PI*variance_of_hypothetical_distances));

    // Evaluate candidates
    int candidates_size = candidates.size();
    for(size_t i = 0; i < candidates_size; i++)
    {
        //# Inner frontiers #
        if(candidates[i]->type() == INNER_FRONTIER)
        {
            int loops = 0;
            for(size_t ii = 0; ii < loop_distances_.size() && loops == 0; ii++)
            {
                if(loop_distances_[ii].first->id() == candidates[i]->id())
                {
                    loops++;
                    for(size_t iii = 0; iii < loop_distances_.size(); iii++)
                    {
                        if(ii == iii) continue;

                        if(loop_distances_[ii].second[iii].second != INFINITE_EDGE)
                            loops++;
                    }
                }
            }

            min_dist_candidate.push_back(0);

            // Eq. 2
            if(loops > 1)
            {
                interesting_frontiers++;
                candidates[i]->attractiveness(loops);
            }
            else
                candidates[i]->attractiveness(0);
        }
        //# Outer frontiers #
        else
        {
            interesting_frontiers++;
            if(variance_of_hypothetical_distances == 0)
            {
                min_dist_candidate.push_back(smaller_hypothetical_distance);
                candidates[i]->attractiveness(1);
                continue;
            }

            float min_dist = 999999;
            float total = 0;

            for(size_t ii = 0; ii < hypothetical_distances_.size(); ii++)
            {
                if(hypothetical_distances_[ii].first->id() == candidates[i]->id())
                {
                    // Eq. 3
                    for(size_t iii = 0; iii < hypothetical_distances_.size(); iii++)
                    {
                        if(ii == iii) continue;

                        if(hypothetical_distances_[ii].second[iii].second < INFINITE_EDGE)
                        {
                            if(hypothetical_distances_[ii].second[iii].second < min_dist)
                                min_dist = hypothetical_distances_[ii].second[iii].second;

                            total += (1/sqrt(2*M_PI*variance_of_hypothetical_distances))
                                    * exp(-0.5 * pow((hypothetical_distances_[ii].second[iii].second-smaller_hypothetical_distance),2)
                                    /(variance_of_hypothetical_distances));
                        }
                    }
                    break;
                }
            }

            if(total == 0) min_dist_candidate.push_back(0);
            else min_dist_candidate.push_back(min_dist);

            candidates[i]->attractiveness(total);
            if(total > max_attractiveness) max_attractiveness = total;
        }
    }

    // Checks whether the exploration is complete
    if(interesting_frontiers == 0)
    {
        logger_->LogTimeNextGoal();
        std::cout << "[EXPLORATION] Nor outer frontiers neither inner frontiers to be explored!" << std::endl;
        return false;
    }

    if(variance_of_hypothetical_distances != 0)
    {
        for(size_t i = 0; i < candidates_size; i++)
        {
            if(candidates[i]->type() != INNER_FRONTIER)
            {      
                // Eq. 4
                float attractiveness_normalized = candidates[i]->attractiveness()/max_attractiveness;

                attractiveness_normalized *= (1/sqrt(2*M_PI*variance_of_hypothetical_distances))
                                        * exp(-0.5 * pow((min_dist_candidate[i]-smaller_hypothetical_distance),2)
                                        /(variance_of_hypothetical_distances))/max_hypothetical_distances_density;

                // Eq. 5
                candidates[i]->attractiveness(alpha_ * (1.0 + attractiveness_normalized));
            }
        }
    }

    float total = 0;

    // Get only the candidates with some attractiveness
    vector<Node*> attractive_candidates;
    for(size_t i = 0; i < candidates_size; i++)
        if(candidates[i]->attractiveness() > 0)
            attractive_candidates.push_back(candidates[i]);

    float smaller_from_robot_distance = SmallerFromRobotDistance(attractive_candidates);
    float avegare_from_robot_distance = AverageFromRobotDistance(attractive_candidates);
    float variance_of_from_robot_distance = pow(StandardDeviationFromRobotDistance(attractive_candidates, avegare_from_robot_distance),2);
    float max_distance_density = (1/sqrt(2*M_PI*variance_of_from_robot_distance));

    // Considering the distance between candidate and robot
    if(variance_of_from_robot_distance != 0)
        for(size_t i = 0; i < attractive_candidates.size(); i++)
        {
            total = attractive_candidates[i]->attractiveness();

            float distance_density_probability = (1/sqrt(2*M_PI*variance_of_from_robot_distance))
                                    * exp(-0.5 * pow((attractive_candidates[i]->cell()->distance_from_robot-smaller_from_robot_distance),2)
                                    /(variance_of_from_robot_distance));
            total *= distance_density_probability/max_distance_density;

            attractive_candidates[i]->attractiveness(total);
        }
    
    // Get the candidate with big attractiveness
    int index = 0;
    max_attractiveness = candidates[index]->attractiveness();
    for(int c = 1; c < candidates_size; c++)
    {
        if(candidates[c]->attractiveness() > max_attractiveness)
        {
            index = c;
            max_attractiveness = candidates[c]->attractiveness();
        }
    }

    goal_ = candidates[index];

    this->UpdateDistanceFromGoalToRails(true);

    logger_->LogTimeNextGoal();

    return true;
}

void LoopExploration::DefineGraph()
{
    std::vector<RailsToGraphHelper*> queue_visited, queue_unvisited, new_queue;

    RailsToGraphHelper* helper;

    Cell *cell;

    Cell *root = grid_->cell(robot_rails_cell_->x,robot_rails_cell_->y);

    plan_counter_++;
    root->graph_code = grid_->laeg()->next_graph_code();
    root->plan_counter=plan_counter_;
    root->distance_from_robot = 0;

    grid_->laeg()->ResetGraph();

    Node* node = grid_->laeg()->AddNode(root, ROBOT_NODE);

    RailsToGraphHelper* new_helper = new RailsToGraphHelper();
    new_helper->cell = root;
    new_helper->father = node;
    new_helper->distance = 0;
    new_helper->graph_code = root->graph_code;
    new_helper->unknown = (root->type == UNEXPLORED);
    new_helper->unvisited = !root->is_visited;
    new_helper->blocked_way = false;

    queue_visited.push_back(new_helper);

    while(!queue_visited.empty() || !queue_unvisited.empty())
    {
        if(!queue_visited.empty())
        {
            helper = queue_visited.front();
            queue_visited.erase(queue_visited.begin());
        }
        else {
            helper = queue_unvisited.front();
            queue_unvisited.erase(queue_unvisited.begin());
        }

        helper->cell->graph_code = helper->graph_code;

        bool add_out = false;
        bool add_in = false;
        bool add_connector = false;

        int rails_neighbors = 0;
        for(int n=0;n<size_offset_;n++)
        {
            cell = grid_->cell(helper->cell->x+offset[n][0],helper->cell->y+offset[n][1]);

            if(!cell->is_center)
                continue;

            if(cell->graph_code != helper->cell->graph_code)
                rails_neighbors++;

            if(cell->plan_counter == plan_counter_)
                continue;

            cell->distance_from_goal=0;
            cell->distance_from_robot=helper->cell->distance_from_robot+1;

            RailsToGraphHelper* new_helper = new RailsToGraphHelper();
            new_helper->cell = cell;
            new_helper->graph_code = helper->graph_code;
            new_helper->cell->graph_code = helper->graph_code;
            new_helper->unknown = (cell->type == UNEXPLORED || helper->unknown);
            new_helper->unvisited = (!cell->is_visited || helper->unvisited);
            new_helper->father = helper->father;
            new_helper->distance = helper->distance+1;

            new_queue.push_back(new_helper);
        }

        // CONNECTOR
        if(rails_neighbors > 1)
        {
            if(helper->cell->distance_from_robot > 0 && !(helper->distance < 5 && helper->father->type() == CONNECTOR))
            {
                add_connector = true;
                Cell* nc;
                for(int n=0;n<size_offset_;n++)
                {
                    nc = grid_->cell(helper->cell->x+offset[n][0],helper->cell->y+offset[n][1]);
                    if(this->HasNearFrontier(nc))
                    {
                        add_connector = false;
                        break;
                    }

                }
            }
        }

        // INNER FRONTIER
        if(helper->cell->is_visited && helper->cell->type == FREE)
        {
            if(this->HasUnvisitedNeighbor(helper->cell) || this->HasUnknowNeighbor(helper->cell))
                add_in = true;
        }

        // OUTER FRONTIER
        else if(helper->cell->type == FREE)
        {
            if(this->HasUnknowNeighbor(helper->cell))
                add_out = true;
        }

        if(add_out)
        {
            node = grid_->laeg()->AddNode(helper->cell, OUTER_FRONTIER);
            grid_->laeg()->AddEdge(helper->father, node, helper->distance, helper->unknown, helper->unvisited);
        }
        else if(add_in)
        {
            node = grid_->laeg()->AddNode(helper->cell, INNER_FRONTIER);
            grid_->laeg()->AddEdge(helper->father, node, helper->distance, helper->unknown, helper->unvisited);
        }
        else if(add_connector)
        {
            node = grid_->laeg()->AddNode(helper->cell, CONNECTOR);
            grid_->laeg()->AddEdge(helper->father, node, helper->distance, helper->unknown, helper->unvisited);
        }

        // If needs to add any new Node
        if(add_out || add_in || add_connector)
        {
            for(int n=0;n<size_offset_;n++) // Looks the surrounding area
            {
                cell = grid_->cell(helper->cell->x+offset[n][0],helper->cell->y+offset[n][1]);
                if(cell->plan_counter == plan_counter_ && cell->graph_code != helper->graph_code)
                {
                    for (size_t i = 0; i < queue_visited.size(); i++)
                        if(queue_visited[i]->cell->x == cell->x && queue_visited[i]->cell->y == cell->y)
                        {
                            RequesterGraphMerge* request = new RequesterGraphMerge();
                            request->cell = queue_visited[i]->cell;
                            request->father = node;
                            request->unknown = queue_visited[i]->unknown;
                            request->unvisited = queue_visited[i]->unvisited;
                            request->distance = queue_visited[i]->distance + 1;
                            queue_visited[i]->requested_connections.push_back(request);
                        }

                    for (size_t i = 0; i < queue_unvisited.size(); i++)
                        if(queue_unvisited[i]->cell->x == cell->x && queue_unvisited[i]->cell->y == cell->y)
                        {
                            RequesterGraphMerge* request = new RequesterGraphMerge();
                            request->cell = queue_unvisited[i]->cell;
                            request->father = node;
                            request->unknown = queue_unvisited[i]->unknown;
                            request->unvisited = queue_unvisited[i]->unvisited;
                            request->distance = queue_unvisited[i]->distance + 1;
                            queue_unvisited[i]->requested_connections.push_back(request);
                      }
                }
                cell->plan_counter = plan_counter_;
            }

            for(size_t i=0; i < helper->requested_connections.size(); i++)
                grid_->laeg()->AddEdge(helper->requested_connections[i]->father, node, helper->requested_connections[i]->distance,
                                                   (node->cell()->type == UNEXPLORED || helper->requested_connections[i]->unknown),
                                                   (!node->cell()->is_visited || helper->requested_connections[i]->unvisited));

            int next_graph_code = grid_->laeg()->next_graph_code();
            while(!new_queue.empty())
            {
                helper=new_queue.front();

                helper->father = node;
                helper->distance = 1;
                helper->graph_code = next_graph_code;
                helper->cell->graph_code = helper->graph_code;
                helper->unknown = (helper->cell->type == UNEXPLORED);
                helper->unvisited = !helper->cell->is_visited;

                if(helper->cell->type == FREE)
                    queue_visited.push_back(helper);
                else
                    queue_unvisited.push_back(helper);
                new_queue.erase(new_queue.begin());
            }
        }
        // Searching encounters (i.e. same cell reached by both sides)
        else
        { 
            for(int n=0;n<size_offset_;n++)
            {
                cell = grid_->cell(helper->cell->x+offset[n][0],helper->cell->y+offset[n][1]);
                if(cell->plan_counter == plan_counter_ && cell->graph_code != helper->graph_code)
                {
                    for (size_t i = 0; i < queue_visited.size(); i++)
                        if(queue_visited[i]->cell->x == cell->x && queue_visited[i]->cell->y == cell->y)
                        {
                            RequesterGraphMerge* request = new RequesterGraphMerge();
                            request->cell = queue_visited[i]->cell;
                            request->father = helper->father;
                            request->unknown = queue_visited[i]->unknown;
                            request->unvisited = queue_visited[i]->unvisited;
                            request->distance = queue_visited[i]->distance + helper->distance;
                            queue_visited[i]->requested_connections.push_back(request);
                        }

                    for (size_t i = 0; i < queue_unvisited.size(); i++)
                        if(queue_unvisited[i]->cell->x == cell->x && queue_unvisited[i]->cell->y == cell->y)
                        {
                            RequesterGraphMerge* request = new RequesterGraphMerge();
                            request->cell = queue_unvisited[i]->cell;
                            request->father = helper->father;
                            request->unknown = queue_unvisited[i]->unknown;
                            request->unvisited = queue_unvisited[i]->unvisited;
                            request->distance = queue_unvisited[i]->distance + helper->distance;
                            queue_unvisited[i]->requested_connections.push_back(request);
                        }
                }
                cell->plan_counter = plan_counter_;
            }

            for(size_t i=0; i < helper->requested_connections.size(); i++)
                grid_->laeg()->AddEdge(helper->requested_connections[i]->father, helper->father,
                                                   helper->requested_connections[i]->distance,
                                                   (helper->cell->type == UNEXPLORED || helper->requested_connections[i]->unknown),
                                                   (!helper->cell->is_visited || helper->requested_connections[i]->unvisited));

            while(!new_queue.empty())
            {
                helper=new_queue.front();

                if(helper->cell->type == FREE)
                    queue_visited.push_back(helper);
                else
                    queue_unvisited.push_back(helper);
                new_queue.erase(new_queue.begin());
            }
        }
    }

    grid_->laeg()->MergingAndPrunningNodes();
}

// Update the distanceFromGoal of all center cells
void LoopExploration::UpdateDistanceFromGoalToRails(bool outer_allowed)
{
    std::queue<Cell*> cell_queue;

    Cell *c, *nc;

    cell_queue.push(goal_->cell());
    plan_counter_++;
    while(!cell_queue.empty())
    {
        c=cell_queue.front();
        c->plan_counter=plan_counter_;

        for(int n=0;n<size_offset_;n++)
        {
            nc = grid_->cell(c->x+offset[n][0],c->y+offset[n][1]);
            if(!nc->is_center || (nc->type==UNEXPLORED && !outer_allowed) || nc->plan_counter == plan_counter_)
                continue;

            nc->distance_from_goal=c->distance_from_goal+1;
            nc->plan_counter = plan_counter_;
            cell_queue.push(nc);

        }
        cell_queue.pop();
    }
}

void LoopExploration::RemoveCenterCellsAtTinySpace()
{
    Cell *c, *nc, *nnc;
    queue<Cell*> cell_queue;

    // Find nearest center cell
    c = robot_rails_cell_;

    plan_counter_++;

    cell_queue.push(c);
    c->plan_counter = plan_counter_;

    while(!cell_queue.empty())
    {
        c=cell_queue.front();
        cell_queue.pop();
        
        // Avoids cross through small frontiers
        if(this->HasObstacleNeighbor(c))
            c->is_center = false;

        for(int n=0;n<8;n++)
        {
            nc = grid_->cell(c->x+offset[n][0],c->y+offset[n][1]);

            if(!nc->is_center || nc->plan_counter == plan_counter_)
                continue;

            nc->plan_counter = plan_counter_;
            
            cell_queue.push(nc);
        }
    }
}

bool LoopExploration::HasObstacleNeighbor(Cell *cell)
{
    Cell* nc;

    int pad = 0;

    if(cell->type == UNEXPLORED)
        pad = min_dist_to_obstacles_from_unknown_cells_;
    else if(cell->type == FREE)
        pad = min_dist_to_obstacles_from_free_cells_;
    else return false;

    for(int x=-pad; x<=pad;x++)
    {
        for(int y=-pad; y<=pad;y++)
        {
            if(sqrt(pow(x,2)+pow(y,2)) > pad)
                continue;

            nc = grid_->cell(cell->x+x,cell->y+y);
            if(nc->type == OCCUPIED || nc->type == NEAROBSTACLE)
                return true;
        }
    }

    return false;
}

bool LoopExploration::HasUnvisitedNeighbor(Cell* cell)
{
    Cell* nc;
    for(int n=0;n<size_offset_;n++)
    {
        nc = grid_->cell(cell->x+offset[n][0],cell->y+offset[n][1]);
        if(!nc->is_visited && nc->is_center)
            return true;
    }

    return false;
}

bool LoopExploration::HasUnknowNeighbor(Cell *cell)
{
    Cell* nc;
    for(int n=0;n<size_offset_;n++)
    {
        nc = grid_->cell(cell->x+offset[n][0],cell->y+offset[n][1]);
        if(nc->type == UNEXPLORED && nc->is_center)
            return true;
    }

    return false;
}

bool LoopExploration::HasNearFrontier(Cell *cell)
{
    // OUTER FRONTIER
    if(cell->type == FREE)
    {
        if(this->HasUnknowNeighbor(cell))
            return true;
    }

    // INNER FRONTIER
    else if(cell->is_visited)
    {
        if(this->HasUnvisitedNeighbor(cell))
            return true;
    }

    return false;
}

void LoopExploration::InitializePotential()
{
    Cell *c;
    for(int i=robot_cell_->x-2*actual_potential_window_half_; i<=robot_cell_->x+2*actual_potential_window_half_; i++)
    {
        for(int j=robot_cell_->y-2*actual_potential_window_half_; j<=robot_cell_->y+2*actual_potential_window_half_; j++)
        {
            if(i>=robot_cell_->x-actual_potential_window_half_ && i<=robot_cell_->x+actual_potential_window_half_ &&
               j>=robot_cell_->y-actual_potential_window_half_ && j<=robot_cell_->y+actual_potential_window_half_ )
                continue;

            c = grid_->cell(i,j);

            c->potential = 0.5;
            c->direction_x = 0.0;
            c->direction_y = 0.0;
        }
    }

    for(int i=robot_cell_->x-actual_potential_window_half_-1; i<=robot_cell_->x+actual_potential_window_half_+1; i++)
    {
        c = grid_->cell(i,robot_cell_->y-actual_potential_window_half_-1);
        c->potential = 1.0;
        c = grid_->cell(i,robot_cell_->y+actual_potential_window_half_+1);
        c->potential = 1.0;
    }

    for(int j=robot_cell_->y-actual_potential_window_half_-1; j<=robot_cell_->y+actual_potential_window_half_+1; j++)
    {
        c = grid_->cell(robot_cell_->x-actual_potential_window_half_-1,j);
        c->potential = 1.0;
        c = grid_->cell(robot_cell_->x+actual_potential_window_half_+1,j);
        c->potential = 1.0;
    }

    for(int i=robot_cell_->x-actual_potential_window_half_; i<=robot_cell_->x+actual_potential_window_half_; i++)
    {
        for(int j=robot_cell_->y-actual_potential_window_half_; j<=robot_cell_->y+actual_potential_window_half_; j++)
        {
            c = grid_->cell(i,j);

            if(c->type == OCCUPIED || c->type == NEAROBSTACLE)
                c->potential = 1.0;
        }
    }

    local_goal_cell_->potential = 0.0;
}

double LoopExploration::IteratePotential()
{
    Cell *c,*l,*r,*u,*d;
    double totalVariation = 0.0;

    for(int i=robot_cell_->x-actual_potential_window_half_; i<=robot_cell_->x+actual_potential_window_half_; i++)
    {
        for(int j=robot_cell_->y-actual_potential_window_half_; j<=robot_cell_->y+actual_potential_window_half_; j++)
        {
            c = grid_->cell(i,j);

            
            if(c->type != OCCUPIED && c->type != NEAROBSTACLE && !(c->x == local_goal_cell_->x && c->y == local_goal_cell_->y))
            {
                double prev = c->potential;

                l=grid_->cell(i-1,j);
                r=grid_->cell(i+1,j);
                u=grid_->cell(i,j+1);
                d=grid_->cell(i,j-1);

                c->potential = 0.25*(l->potential + r->potential + d->potential + u->potential);

                totalVariation += fabs(c->potential - prev);
            }
        }
    }
    return totalVariation;
}

void LoopExploration::UpdateGradient()
{
    Cell *c,*l,*r,*u,*d;

    for(int i=robot_cell_->x-actual_potential_window_half_; i<=robot_cell_->x+actual_potential_window_half_; i++)
    {
        for(int j=robot_cell_->y-actual_potential_window_half_; j<=robot_cell_->y+actual_potential_window_half_; j++)
        {
            c = grid_->cell(i,j);

            if(c->type==FREE)
            {
                l=grid_->cell(i-1,j);
                r=grid_->cell(i+1,j);
                u=grid_->cell(i,j+1);
                d=grid_->cell(i,j-1);

                c->direction_x = l->potential - r->potential;
                c->direction_y = d->potential - u->potential;

                double norm = sqrt(c->direction_x*c->direction_x + c->direction_y*c->direction_y);
                if(norm==0)
                {
                    c->direction_x = 0.0;
                    c->direction_y = 0.0;
                }
                else
                {
                    c->direction_x *= 1.0/norm;
                    c->direction_y *= 1.0/norm;
                }

            }
            else
            {
                c->direction_x = 0.0;
                c->direction_y = 0.0;
            }
        }
    }
    
}
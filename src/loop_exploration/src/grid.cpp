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

#include "grid.h"

mutex Grid::update_mutex;

Grid::Grid (Configuration *config)
{
    map_resolution_ = config->GetFloat("map_resolution");
    map_scale_ = 1/map_resolution_;
    map_max_width_ = map_max_height_ = config->GetInt("map_width_height");
    num_cells_in_row_=map_max_width_;
    half_num_cells_in_row_=map_max_width_/2;

    pad_x = 0;
    pad_y = 0;

    laeg_ = new LAEG(config);

    cells_ = new Cell[map_max_width_*map_max_height_];

    for (unsigned int j = 0; j < num_cells_in_row_; ++j)
    {
        for (unsigned int i = 0; i < num_cells_in_row_; ++i)
        {
            unsigned int c = j*num_cells_in_row_ + i;
            cells_[c].x = -half_num_cells_in_row_ + 1 + i;
            cells_[c].y =  half_num_cells_in_row_ - j;

            cells_[c].Reset();
        }
    }

    // Mark rails helper cells
    for (unsigned int j = 0; j < num_cells_in_row_; j=j+2)
    {
        for (unsigned int i = 0; i < num_cells_in_row_; i=i+2)
        {
            cells_[j*num_cells_in_row_ + i].is_rails_helper=true;
        }
    }

    num_view_modes_ = 3;
    view_mode_ = 1;

    show_arrows_ = false;
    show_path_ = false;
    show_laeg_ = 2;

    map_limits.min_x = map_limits.min_y =  1000000;
    map_limits.max_x = map_limits.max_y = -1000000;
    old_map_limits.min_x = old_map_limits.min_y =  1000000;
    old_map_limits.max_x = old_map_limits.max_y = -1000000;

    map_visited_limits.min_x = map_visited_limits.min_y =  1000000;
    map_visited_limits.max_x = map_visited_limits.max_y = -1000000;
    old_map_visited_limits.min_x = old_map_visited_limits.min_y =  1000000;
    old_map_visited_limits.max_x = old_map_visited_limits.max_y = -1000000;

    map_changed_ = false;
    map_visited_changed_ = false;
    ready_to_update_ = true;
}

Cell* Grid::cell (int x, int y)
{
    int i=x+half_num_cells_in_row_-1;
    int j=half_num_cells_in_row_-y;
    return &(cells_[j*num_cells_in_row_ + i]);
}

float Grid::map_resolution()
{
    return map_resolution_;
}

int Grid::map_scale()
{
    return map_scale_;
}

int Grid::map_max_width()
{
    return map_max_width_;
}

int Grid::map_max_height()
{
    return map_max_height_;
}

LAEG *Grid::laeg()
{
    return laeg_;
}

void Grid::NotifyMapChanges()
{
    map_changed_ = true;
}

void Grid::NotifyMapVisitedChanges()
{
    map_visited_changed_ = true;
}

bool Grid::Changed()
{
    if(!map_changed_ || !map_visited_changed_) return false;
    map_changed_ = false;
    map_visited_changed_= false;
    return true;
}

bool Grid::ready_to_update()
{
    unique_lock<mutex> lock(update_mutex);
    if(ready_to_update_)
    {
        ready_to_update_ = false;
        return true;
    }
    return false;
}

void Grid::ReleaseToUpdate()
{
    unique_lock<mutex> lock(update_mutex);
    ready_to_update_ = true;
}


//###############################
// VISUALIZATION FUNCTIONS
//###############################

void Grid::ChangeShowArrows()
{
    show_arrows_ = !show_arrows_;
    ChangeWindowName();
}

void Grid::ChangeShowPath()
{
    show_path_ = !show_path_;
    ChangeWindowName();
}

void Grid::ChangeShowFrontiers()
{
    show_laeg_++;
    if(show_laeg_ > 3) show_laeg_ = 0;
    ChangeWindowName();
}

void Grid::ChangeViewMode(int step)
{
    view_mode_ += step;
    if(view_mode_ == num_view_modes_)
        view_mode_ = 0;
    else if(view_mode_ == -1)
        view_mode_ = num_view_modes_-1;

    ChangeWindowName();
}

const char *Grid::ChangeWindowName()
{
    std::string s = "Unamed";
    switch(view_mode_) {
        case 0:
            s = "Map expanded with Rails";
            break;
        case 1:
            s = "Map expanded";
            break;
        case 2:
            s = "White board";
            break;
    }

    if(show_path_) s += " + Path";
    if(show_laeg_ == 1) s += " + Frontiers (big)";
    else if(show_laeg_ == 2) s += " + Frontiers (small)";
    else if(show_laeg_ == 3) s += " + Frontiers (tiny)";
    if(show_arrows_) s += " + Potential Field";

    glutSetWindowTitle(s.c_str());
    return s.c_str();
}

bool Grid::show_path()
{
    if(view_mode_ == 2) return false;
    return show_path_;
}

void Grid::Draw(int xi, int yi, int xf, int yf)
{
    glLoadIdentity();

    for(int i=xi; i<=xf; ++i)
    {
        for(int j=yi; j<=yf; ++j)
        {
            DrawCell(i+j*num_cells_in_row_);
        }
    }

    if(view_mode_ == 2) return;

    if(show_arrows_)
    {
        glPointSize(2);
        for(int i=xi; i<=xf; ++i)
        {
            for(int j=yi; j<=yf; ++j)
            {
                DrawVector(i+j*num_cells_in_row_);
            }
        }
    }
}

void Grid::DrawCell(unsigned int n)
{
    float aux;

    if(view_mode_== 0) // With rails
    {
        if(cells_[n].is_center)
            glColor3fv(color_rails);
        else if(cells_[n].type == UNEXPLORED)
            glColor3fv(color_cell_unexplored);
        else if(cells_[n].type == OCCUPIED)
            glColor3fv(color_cell_occupied);
        else if(cells_[n].type == NEAROBSTACLE)
            glColor3fv(color_cell_near_occupied);
        else if(cells_[n].is_visited)
            glColor3fv(color_cell_visited);
        else
            glColor3fv(color_cell_free);
    }
    else if(view_mode_==1) //Without rails
    {
        if(cells_[n].type == UNEXPLORED)
            glColor3fv(color_cell_unexplored);
        else if(cells_[n].type == OCCUPIED)
            glColor3fv(color_cell_occupied);
        else if(cells_[n].type == NEAROBSTACLE)
            glColor3fv(color_cell_near_occupied);
        else if(cells_[n].is_visited)
            glColor3fv(color_cell_visited);
        else
            glColor3fv(color_cell_free);
    }
    else if(view_mode_==2) //White board
    {
        glColor3f(1,1,1);
    }

    glBegin( GL_QUADS );
    {
        glVertex2f(cells_[n].x+1, cells_[n].y+1);
        glVertex2f(cells_[n].x+1, cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y+1);
    }
    glEnd();
}

void Grid::DrawCellWithColor(int x, int y, GLfloat const color[])
{
    Cell* c = cell(x,y);

    glColor3fv(color);

    glBegin( GL_QUADS );
    {
        glVertex2f(c->x+1, c->y+1);
        glVertex2f(c->x+1, c->y  );
        glVertex2f(c->x  , c->y  );
        glVertex2f(c->x  , c->y+1);
    }
    glEnd();
}

void Grid::DrawVector(unsigned int n)
{
    if(cells_[n].type == FREE)
    {
        if(cells_[n].direction_x != 0.0 || cells_[n].direction_y != 0.0)
        {
            glColor3fv(color_potential_arrow);
            glLineWidth(1);
            glBegin( GL_LINES );
            {
                glVertex2f(cells_[n].x+0.5, cells_[n].y+0.5);
                glVertex2f(cells_[n].x+0.5+cells_[n].direction_x, cells_[n].y+0.5+cells_[n].direction_y);
            }
            glEnd();
            glBegin( GL_POINTS );
            {
                glVertex2f(cells_[n].x+0.5+cells_[n].direction_x, cells_[n].y+0.5+cells_[n].direction_y);
            }
            glEnd();
        }
    }
}

void Grid::DrawGraph(float scale)
{
    if(show_laeg_ == 0) return;

    laeg_->DrawGraph(scale, show_laeg_);
}

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

#ifndef __GRID_H__
#define __GRID_H__

#include <string>
#include <set>
#include <vector>
#include <math.h>
#include <cmath>

#include <cstdio>
#include <GL/glut.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <mutex>

#include "utils.h"
#include "cell.h"
#include "laeg.h"

#include "colors.h"

typedef struct
{
    int min_x, max_x, min_y, max_y;
} BoundingBox;

class Grid
{
public:
    Grid(Configuration* config);
    Cell* cell(int x, int y);

    float map_resolution();
    int map_scale();
    int map_max_width();
    int map_max_height();

    void ChangeShowArrows();
    void ChangeShowFrontiers();
    void ChangeShowPath();
    void ChangeViewMode(int step);
    const char *ChangeWindowName();

    LAEG *laeg();

    void Draw(int xi, int yi, int xf, int yf);
    void DrawGraph(float scale);

    int plan_iteration;

    void NotifyMapChanges();
    void NotifyMapVisitedChanges();
    bool Changed();

    bool show_path();

    BoundingBox map_limits;
    BoundingBox old_map_limits;

    BoundingBox map_visited_limits;
    BoundingBox old_map_visited_limits;

    int pad_x, pad_y; // to translate the map's origin to math with slam ones

    static std::mutex update_mutex;
    bool ready_to_update();
    void ReleaseToUpdate();

private:
    float map_resolution_;
    int map_scale_;
    int map_max_width_, map_max_height_; // in cells
    int num_cells_in_row_, half_num_cells_in_row_;

    Cell* cells_;

    LAEG* laeg_;

    int num_view_modes_;
    int view_mode_;
    bool show_arrows_;
    bool show_path_;
    int show_laeg_;

    bool map_changed_;
    bool map_visited_changed_;

    bool ready_to_update_;

    void DrawCell(unsigned int i);
    void DrawCellWithColor(int x, int y, GLfloat const color[]);
    void DrawVector(unsigned int i);
};

#endif // __GRID_H__

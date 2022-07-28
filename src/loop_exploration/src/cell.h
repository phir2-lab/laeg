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

#ifndef CELL_H
#define CELL_H

enum CellType {OCCUPIED, UNEXPLORED, FREE, NEAROBSTACLE};

typedef struct
{
    int x,y;
} RobotCell;

class Cell
{
public:
    Cell();

    CellType type;

    void Reset();

    int x, y;

    int slam_value;

    double potential, direction_x, direction_y;

    bool is_visited, is_center, is_visited_center, is_rails_helper;

    int distance_from_robot, distance_from_goal;

    int plan_counter;

    int graph_code;
};

#endif // CELL_H

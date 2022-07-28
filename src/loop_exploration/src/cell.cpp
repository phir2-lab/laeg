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

#include "cell.h"

Cell::Cell (){}

void Cell::Reset()
{
    slam_value=-1;
    potential=0.5;
    distance_from_goal=-1;
    direction_x=0.0;
    direction_y=0.0;

    is_center=false;
    is_visited_center=false;
    is_rails_helper=false;

    plan_counter = 0;
    graph_code = 0;

    is_visited=false;
    type = UNEXPLORED;
}

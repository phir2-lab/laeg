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

#ifndef COLORS_H
#define COLORS_H

//CELLS COLOR
GLfloat const color_cell_unexplored[] = {1,1,1};
GLfloat const color_cell_occupied[] = {0,0,0};
GLfloat const color_cell_near_occupied[] = {0.3,0.3,0.3};
GLfloat const color_cell_visited[] = {1,0.95,0.5};
GLfloat const color_cell_free[] = {0.85,0.85,0.85};
GLfloat const color_potential_arrow[] = {1,0.0,0.0};

//RAILS
GLfloat const color_rails[] = {0.0,0.8,0.0};
GLfloat const color_rails_goal[] = {0.0,0.4,0.0};
GLfloat const color_rails_frontier[] = {0.4,0.4,0.0};
GLfloat const color_rails_text[] = {0.5,0.0,0.0};

//ROBOT
GLfloat const color_robot_body[] = {1.0,0.0,0.0};
GLfloat const color_robot_pointer_potential[] = {0.0,0.2,0.6};
GLfloat const color_robot_pointer_forward[] = {0.0,0.0,0.0};
GLfloat const color_odometry[] = {0.7,0.3,1.0};
GLfloat const color_ground_truth[] = {0.3,0.5,1.0};

//LAEG
GLfloat const color_graph_node_outer_frontier[] = {1.0,0.7,0.2};
GLfloat const color_graph_node_inner_frontier[] = {0.2,0.7,1.0};
GLfloat const color_graph_node_robot[] = {1.0,0.5,0.5};
GLfloat const color_graph_node_connector[] = {0.5,0.5,0.5};
GLfloat const color_graph_node_border[] = {0,0,0};
GLfloat const color_graph_edge_hypothetical_loop[] = {0.8,0.5,0.3};
GLfloat const color_graph_edge_visited_path[] = {0.3,0.3,0.3};
GLfloat const color_graph_edge_free_path[] = {0.3,0.5,0.7};
GLfloat const color_graph_edge_standard[] = {1,1,0};

#endif // COLORS_H

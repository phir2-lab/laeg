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

#ifndef __LAEG_H__
#define __LAEG_H__

#include <cstdio>
#include <GL/glut.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <string>
#include <set>
#include <vector>
#include <stack>
#include <math.h>
#include <algorithm>

#include "cell.h"
#include "utils.h"

#include "configuration.h"

#define INFINITE_EDGE 9999999

enum NodeType {INNER_FRONTIER, OUTER_FRONTIER, CONNECTOR, ROBOT_NODE};
class Node;

enum EdgeType {HYPOTHETICAL_PATH, VISITED_PATH, FREE_PATH};
class Edge
{
public:
    Edge(Node* node_1, Node* node_2, int length, bool unknown, bool unvisited);

    float length();
    void length(float distance);

    int id();

    bool HasNode(int node_id);

    Node* GetConnectedTo(Node* node);

    std::vector<Node*> nodes();

    std::vector<Cell*> position_to_draw;

    EdgeType type();

    bool unknown();

    bool unvisited();

    int iteration_code;

    static int next_id;

private:
    int edge_id_;

    Node *node_1_, *node_2_;

    EdgeType type_;

    float length_;

    bool unknown_;
    bool unvisited_;
};

class Node
{
public:
    Node(Cell* cell, NodeType type);

    NodeType type();

    Cell* cell();

    int id();

    float attractiveness();
    void attractiveness(float value);

    void AddNeighbor(Edge *edge);

    void RemoveEdge(Edge* edge);
    std::vector<Edge*> edges();

    int ConnectionLenghtWith(int node_id);

    int distance_min;

    static int next_id;

private:
    int node_id_;

    NodeType type_;

    Cell* cell_;
    float attractiveness_;

    std::vector<Edge*> edges_;

};

class LAEG
{
public:
    LAEG(Configuration* config);

    void ResetGraph();

    void MergingAndPrunningNodes();

    std::vector<std::pair<Node *, std::vector<std::pair<Node *, int> > > > ComputehypotheticalDistancesUsingChebyshevRails();
    std::vector<std::pair<Node *, std::vector<std::pair<Node *, int> > > > ComputeLoopDistances();

    bool AreWithinMaximumNodeDistanceKnownPath(Node *a, Node *b, int max_distance);

    Node* AddNode(Cell* cell, NodeType type);

    void AddEdge(Node* node_1, Node* node_2, int distance, bool unknown, bool unvisited);
    void RemoveEdge(Edge* edge);

    std::vector<Node*> nodes();
    std::vector<Node*> candidates();

    void DrawGraph(float scale, int view_mode);
    void DrawCircle(Node* node, float scale, float r, int num_segments);
    void DrawEdges(float scale);

    int next_graph_code();

private:
    std::vector<Node*> nodes_;
    std::vector<Edge*> edges_;

    std::vector< std::vector< std::pair<Node*,int> > > hypothetical_distances_; // Matrix of the hypothetical distances between outer frontiers
    std::vector< std::vector< std::pair<Node*,int> > > known_distances_; // Matrix of the known distances between elements
    std::vector< std::vector< std::pair<Node*,int> > > loop_distances_; // Matrix of the known distances between elements

    int next_graph_code_;

    //PARAMETERS
    int max_euclidean_dist_to_merge_frontiers_;
    int max_nodes_dist_to_merge_frontiers_;

};

#endif // LAEG_H

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

#include "laeg.h"
#include "grid.h"

int Node::next_id;
int Edge::next_id;

Node::Node(Cell *cell, NodeType type)
{
    cell_ = cell;
    type_ = type;
    node_id_ = next_id++;
    distance_min = INFINITE_EDGE;
}

NodeType Node::type()
{
    return type_;
}

Cell* Node::cell()
{
    return cell_;
}

int Node::id()
{
    return node_id_;
}

float Node::attractiveness()
{
    return attractiveness_;
}

void Node::attractiveness(float value)
{
    if(value < 0) attractiveness_ = 0;
    else attractiveness_ = value;
}

void Node::AddNeighbor(Edge* edge)
{
    edges_.push_back(edge);
}

void Node::RemoveEdge(Edge *edge)
{
    for (std::vector<Edge*>::iterator it = edges_.begin() ; it != edges_.end(); ++it)
    {
        if((*it)->id() == edge->id())
        {
            edges_.erase(it);
            return;
        }
    }
}

std::vector<Edge *> Node::edges()
{
    return edges_;
}

int Node::ConnectionLenghtWith(int node_id)
{
    if(node_id == this->id()) return 0;
    for (std::vector<Edge*>::iterator it = edges_.begin() ; it != edges_.end(); ++it)
    {
        if((*it)->HasNode(node_id))
            return (*it)->length();
    }
    return INFINITE_EDGE;
}

LAEG::LAEG(Configuration *config)
{
    Node::next_id = 0;
    Edge::next_id = 0;
    next_graph_code_ = 1;

    //Parameters
    max_euclidean_dist_to_merge_frontiers_ = config->GetInt("max_euclidean_dist_to_merge_frontiers");
    max_nodes_dist_to_merge_frontiers_ = config->GetInt("max_nodes_dist_to_merge_frontiers");
}

void LAEG::ResetGraph()
{
    nodes_.clear();
    edges_.clear();
    Node::next_id = 0;
    Edge::next_id = 0;
}

void LAEG::MergingAndPrunningNodes()
{
    std::vector<Node*> nodes_to_be_processed;
    std::vector<Node*> outer_frontiers;

    for(size_t i = 0; i < nodes_.size(); i++)
    {
        switch(nodes_[i]->type())
        {
            case CONNECTOR:
                if(nodes_[i]->edges().size() < 3)
                    nodes_to_be_processed.push_back(nodes_[i]);
                break;
            case OUTER_FRONTIER:
                nodes_to_be_processed.push_back(nodes_[i]);
                outer_frontiers.push_back(nodes_[i]);
                break;
            case INNER_FRONTIER:
                if(nodes_[i]->edges().size() < 2)
                    nodes_to_be_processed.push_back(nodes_[i]);
        }
    }

    while(!nodes_to_be_processed.empty())
    {
        Node* node = nodes_to_be_processed.front();
        nodes_to_be_processed.erase(nodes_to_be_processed.begin());

        // If there is a CONNECTOR
        if(node->type() == CONNECTOR)
        {
            if(node->edges().size() > 2)
                continue;

            std::vector<Edge*> node_edges = node->edges();
            if(node_edges.size() == 1) // Is a dead-end branch
            {
                std::vector<Node*> edge_nodes = node_edges[0]->nodes();
                edge_nodes[0]->RemoveEdge(node_edges[0]);
                edge_nodes[1]->RemoveEdge(node_edges[0]);

                this->RemoveEdge(node_edges[0]);

                Node* other_node = (edge_nodes[0]->id() != node->id()) ? edge_nodes[0] : edge_nodes[1];

                nodes_to_be_processed.push_back(other_node);

                std::vector<Node*>::iterator node_position = std::find(nodes_.begin(), nodes_.end(), node);
                if(node_position != nodes_.end()) nodes_.erase(node_position);
            }
            else if(node_edges.size() == 2) // Is a simple connection
            {
                std::vector<Node*> edge_nodes = node_edges[0]->nodes();
                edge_nodes[0]->RemoveEdge(node_edges[0]);
                edge_nodes[1]->RemoveEdge(node_edges[0]);
                Node* node_1 = (edge_nodes[0]->id() != node->id()) ? edge_nodes[0] : edge_nodes[1];
                
                edge_nodes = node_edges[1]->nodes();
                edge_nodes[0]->RemoveEdge(node_edges[1]);
                edge_nodes[1]->RemoveEdge(node_edges[1]);
                Node* node_2 = (edge_nodes[0]->id() != node->id()) ? edge_nodes[0] : edge_nodes[1];

                this->AddEdge(node_1, node_2, node_edges[0]->length() + node_edges[1]->length(), (node_edges[0]->unknown() || node_edges[1]->unknown()), (node_edges[0]->unvisited() || node_edges[1]->unvisited()));
                this->RemoveEdge(node_edges[0]);
                this->RemoveEdge(node_edges[1]);

                nodes_to_be_processed.push_back(node_1);
                nodes_to_be_processed.push_back(node_2);

                std::vector<Node*>::iterator node_position = std::find(nodes_.begin(), nodes_.end(), node);
                if(node_position != nodes_.end()) nodes_.erase(node_position);
            }
            else if(node_edges.size() == 0)
            {
                std::vector<Node*>::iterator node_position = std::find(nodes_.begin(), nodes_.end(), node);
                if(node_position != nodes_.end()) nodes_.erase(node_position);
            }
        }
        else if(node->type() == INNER_FRONTIER)
        {
            if(node->edges().size() == 0)
            {
                std::vector<Node*>::iterator node_position = std::find(nodes_.begin(), nodes_.end(), node);
                if(node_position != nodes_.end()) nodes_.erase(node_position);
            }
            else if(node->edges().size() == 1)
            {
                std::vector<Edge*> node_edges = node->edges();

                std::vector<Node*> edge_nodes = node_edges[0]->nodes();
                
                edge_nodes[0]->RemoveEdge(node_edges[0]);
                edge_nodes[1]->RemoveEdge(node_edges[0]);

                Node* other_node = (edge_nodes[0]->id() != node->id()) ? edge_nodes[0] : edge_nodes[1];

                nodes_to_be_processed.push_back(other_node);

                this->RemoveEdge(node_edges[0]);

                std::vector<Node*>::iterator node_position = std::find(nodes_.begin(), nodes_.end(), node);
                if(node_position != nodes_.end()) nodes_.erase(node_position);
            }

        }
        else if(node->type() == OUTER_FRONTIER)
        {
            // To be removed
            bool has_known_connection = false;
            std::vector<Edge*> node_edges = node->edges();
            for(size_t i = 0; i < node_edges.size(); i++)
            {
                if(node_edges[i]->type() == FREE_PATH)
                {
                    has_known_connection = true;
                    break;
                }
            }
            if(!has_known_connection)
            {
                std::vector<Edge*> node_edges = node->edges();

                if(node_edges.size() == 2)
                {
                    std::vector<Node*> edge_nodes = node_edges[0]->nodes();
                    edge_nodes[0]->RemoveEdge(node_edges[0]);
                    edge_nodes[1]->RemoveEdge(node_edges[0]);
                    Node* node_1 = (edge_nodes[0]->id() != node->id()) ? edge_nodes[0] : edge_nodes[1];
                    edge_nodes = node_edges[1]->nodes();
                    edge_nodes[0]->RemoveEdge(node_edges[1]);
                    edge_nodes[1]->RemoveEdge(node_edges[1]);
                    Node* node_2 = (edge_nodes[0]->id() != node->id()) ? edge_nodes[0] : edge_nodes[1];

                    nodes_to_be_processed.push_back(node_1);
                    nodes_to_be_processed.push_back(node_2);

                    this->AddEdge(node_1, node_2, node_edges[0]->length() + node_edges[1]->length(), (node_edges[0]->unknown() || node_edges[1]->unknown()), (node_edges[0]->unvisited() || node_edges[1]->unvisited()));
                    this->RemoveEdge(node_edges[0]);
                    this->RemoveEdge(node_edges[1]);
                }
                else if(node_edges.size() == 1)
                {
                    std::vector<Node*> edge_nodes = node_edges[0]->nodes();
                    edge_nodes[0]->RemoveEdge(node_edges[0]);
                    edge_nodes[1]->RemoveEdge(node_edges[0]);

                    Node* other_node = (edge_nodes[0]->id() != node->id()) ? edge_nodes[0] : edge_nodes[1];
                    nodes_to_be_processed.push_back(other_node);

                    this->RemoveEdge(node_edges[0]);

                }
                std::vector<Node*>::iterator node_position = std::find(nodes_.begin(), nodes_.end(), node);
                if(node_position != nodes_.end()) nodes_.erase(node_position);
                node_position = std::find(outer_frontiers.begin(), outer_frontiers.end(), node);
                if(node_position != outer_frontiers.end()) outer_frontiers.erase(node_position);
                node_position = std::find(nodes_to_be_processed.begin(), nodes_to_be_processed.end(), node);
                if(node_position != nodes_to_be_processed.end()) nodes_to_be_processed.erase(node_position);
            }
                
            // To be merged
            if(has_known_connection)
            {
                for(size_t i = 0; i < outer_frontiers.size(); i++)
                {
                    if(node->id() != outer_frontiers[i]->id())
                    {
                        Node* node_1 = outer_frontiers[i];
                        // If is close enough in "euclidean distance" to be merged
                        if(max_euclidean_dist_to_merge_frontiers_ >= EuclideanDistance(node->cell()->x, node_1->cell()->x, node->cell()->y, node_1->cell()->y))
                        {
                            // If is close enough in "node distance" to be merged
                            if(AreWithinMaximumNodeDistanceKnownPath(node, node_1, max_nodes_dist_to_merge_frontiers_))
                            {
                                std::vector<Edge*> node_edges = node_1->edges();

                                while(!node_edges.empty()) // Merge edges
                                {
                                    Edge* edge = node_edges.front();
                                    node_edges.erase(node_edges.begin());

                                    std::vector<Node*> edge_nodes = edge->nodes();
                                    edge_nodes[0]->RemoveEdge(edge);
                                    edge_nodes[1]->RemoveEdge(edge);

                                    Node* node_2 = (edge_nodes[0]->id() != node_1->id()) ? edge_nodes[0] : edge_nodes[1];
                                    nodes_to_be_processed.push_back(node_2);

                                    this->AddEdge(node_2, node, edge->length(), edge->unknown(), edge->unvisited());
                                    this->RemoveEdge(edge);
                                }

                                std::vector<Node*>::iterator node_position = std::find(nodes_.begin(), nodes_.end(), node_1);
                                if(node_position != nodes_.end()) nodes_.erase(node_position);
                                node_position = std::find(outer_frontiers.begin(), outer_frontiers.end(), node_1);
                                if(node_position != outer_frontiers.end()) outer_frontiers.erase(node_position);
                                i--;
                            }
                        }
                    }
                }
            }
        }
    }
}

std::vector< std::pair<Node*, std::vector< std::pair<Node*, int> > > > LAEG::ComputehypotheticalDistancesUsingChebyshevRails()
{
    // Make graph distance table
    std::vector<Node*> related_nodes;

    std::vector< std::pair<Node*, std::vector< std::pair<Node*, int> > > > hypothetical_distances;

    // Find the relevant nodes to be included
    for(size_t i = 0; i < nodes_.size(); i++)
    {
        std::vector<Edge*> node_edges = nodes_[i]->edges();
        for(size_t ii = 0; ii < node_edges.size(); ii++) // Insert nodes with hypothetical loop edges
            if(node_edges[ii]->type() == HYPOTHETICAL_PATH)
            {
                related_nodes.push_back(nodes_[i]);
                break;
            }
    }

    // Build the table only with the relevant nodes
    for(size_t i = 0; i < related_nodes.size(); i++)
    {
        Node* node = related_nodes[i];
        std::vector< std::pair<Node*, int> > distances;
        for(size_t ii = 0; ii < related_nodes.size(); ii++)
        {
            distances.push_back(std::make_pair(related_nodes[ii], node->ConnectionLenghtWith(related_nodes[ii]->id())));
        }
        hypothetical_distances.push_back(std::make_pair(node, distances));
    }

    //#########################
    // Uses Floyd-Warshall algorithm to compute shortest paths
    // k = the index of intermediator

    for (size_t k = 0; k < hypothetical_distances.size(); k++)
    {
        for (size_t i = 0; i < hypothetical_distances.size(); i++)
        {
            for (size_t ii = 0; ii < hypothetical_distances.size(); ii++)
            {
                hypothetical_distances[i].second[ii].second = std::min(hypothetical_distances[i].second[ii].second, hypothetical_distances[i].second[k].second + hypothetical_distances[k].second[ii].second);
            }
        }
    }

    std::vector< std::pair<Node*, std::vector< std::pair<Node*, int> > > > hypothetical_distances_only_frontiers;
    // Keep only the outer frontier nodes
    for(size_t i = 0; i < hypothetical_distances.size(); i++)
    {
        if(hypothetical_distances[i].first->type() == OUTER_FRONTIER)
        {
            std::vector< std::pair<Node*, int> > distances;
            for(size_t ii = 0; ii < hypothetical_distances[i].second.size(); ii++)
            {
                if(hypothetical_distances[i].second[ii].first->type() == OUTER_FRONTIER)
                    distances.push_back(hypothetical_distances[i].second[ii]);
            }
            hypothetical_distances_only_frontiers.push_back(std::make_pair(hypothetical_distances[i].first, distances));
        }
    }

    return hypothetical_distances_only_frontiers;
}

std::vector<std::pair<Node *, std::vector<std::pair<Node *, int> > > > LAEG::ComputeLoopDistances()
{
    std::vector<Node*> related_nodes;

    std::vector< std::pair<Node*, std::vector< std::pair<Node*, int> > > > loop_distances;

    // Find the relevant nodes to be included
    for(size_t i = 0; i < nodes_.size(); i++)
    {
        if(nodes_[i]->type() == INNER_FRONTIER || nodes_[i]->type() == CONNECTOR)
        {
            std::vector<Edge*> node_edges = nodes_[i]->edges();
            for(size_t ii = 0; ii < node_edges.size(); ii++) // Only insert nodes connected with free path edges
                if(node_edges[ii]->type() == FREE_PATH)
                {
                    related_nodes.push_back(nodes_[i]);
                    break;
                }
        }
    }

    // Build the table only with the relevant nodes
    for(size_t i = 0; i < related_nodes.size(); i++)
    {
        Node* node = related_nodes[i];
        std::vector< std::pair<Node*, int> > distances;
        for(size_t ii = 0; ii < related_nodes.size(); ii++)
        {
            distances.push_back(std::make_pair(related_nodes[ii], node->ConnectionLenghtWith(related_nodes[ii]->id())));
        }
        loop_distances.push_back(std::make_pair(node, distances));
    }

    //#########################
    // Uses Floyd-Warshall algorithm to compute shortest paths
    // k = the index of intermediator

    for (size_t k = 0; k < loop_distances.size(); k++)
    {
        for (size_t i = 0; i < loop_distances.size(); i++)
        {
            for (size_t ii = 0; ii < loop_distances.size(); ii++)
            {
                loop_distances[i].second[ii].second = std::min(loop_distances[i].second[ii].second, loop_distances[i].second[k].second + loop_distances[k].second[ii].second);
            }
        }
    }

    std::vector< std::pair<Node*, std::vector< std::pair<Node*, int> > > > loop_distances_only_frontiers;
    // Keep only the inner frontier nodes
    for(size_t i = 0; i < loop_distances.size(); i++)
    {
        if(loop_distances[i].first->type() == INNER_FRONTIER)
        {
            std::vector< std::pair<Node*, int> > distances;
            for(size_t ii = 0; ii < loop_distances[i].second.size(); ii++)
            {
                if(loop_distances[i].second[ii].first->type() == INNER_FRONTIER)
                    distances.push_back(loop_distances[i].second[ii]);
            }
            loop_distances_only_frontiers.push_back(std::make_pair(loop_distances[i].first, distances));
        }
    }

    return loop_distances_only_frontiers;
}

bool LAEG::AreWithinMaximumNodeDistanceKnownPath(Node *a, Node *b, int max_distance)
{
    // Pseudo A*
    std::stack< std::pair<Node*, int> > open_nodes;
    std::vector<Node*> closed_nodes;

    open_nodes.push(std::make_pair(a, 0));

    std::pair<Node*, int>  actual;

    while(!open_nodes.empty())
    {
        actual = open_nodes.top();

        if(actual.first->id() == b->id()) return true;

        open_nodes.pop();
        closed_nodes.push_back(actual.first);

        std::vector<Edge*> node_edges = actual.first->edges();
        for(size_t i = 0; i < node_edges.size(); i++)
        {
            if(node_edges[i]->type() == VISITED_PATH || node_edges[i]->type() == FREE_PATH)
            {
                Node* neighbor = node_edges[i]->GetConnectedTo(actual.first);
                if(actual.second+1 <= max_distance)
                    if(std::find(closed_nodes.begin(), closed_nodes.end(), neighbor) == closed_nodes.end())
                        open_nodes.push(std::make_pair(neighbor, actual.second+1));
            }
        }

    }


    return false;
}

Node *LAEG::AddNode(Cell *cell, NodeType type)
{
    Node* node = new Node(cell, type);
    nodes_.push_back(node);
    return node;
}

void LAEG::AddEdge(Node *node_1, Node *node_2, int distance, bool unknown, bool unvisited)
{
    Edge *new_edge = new Edge(node_1, node_2, distance, unknown, unvisited);
    for (std::vector<Edge*>::iterator it = edges_.begin() ; it != edges_.end(); ++it)
    {
        if((*it)->HasNode(node_1->id()) && (*it)->HasNode(node_2->id()) && new_edge->type() == (*it)->type())
        {
            if((*it)->length() > distance)
                (*it)->length(distance);

            return;
        }
    }

    node_1->AddNeighbor(new_edge);
    node_2->AddNeighbor(new_edge);

    edges_.push_back(new_edge);
}

void LAEG::RemoveEdge(Edge *edge)
{
    for (std::vector<Edge*>::iterator it = edges_.begin() ; it != edges_.end(); ++it)
    {
        if((*it)->id() == edge->id())
        {
            edges_.erase(it);
            return;
        }
    }
}

std::vector<Node *> LAEG::nodes()
{
    return nodes_;
}

std::vector<Node *> LAEG::candidates()
{
    std::vector<Node *> candidates;

    for(size_t c=0; c<nodes_.size(); c++)
        if(nodes_[c]->type() == INNER_FRONTIER || nodes_[c]->type() == OUTER_FRONTIER)
            candidates.push_back(nodes_[c]);

    return candidates;
}

void LAEG::DrawGraph(float scale, int view_mode)
{
    std::vector<Node*> top_nodes;
    std::vector<Node*> bottom_nodes;

    for(size_t c=0; c<nodes_.size(); c++)
    {
        if((nodes_[c]->type() == INNER_FRONTIER && nodes_[c]->attractiveness() > 0) || nodes_[c]->type() == OUTER_FRONTIER)
            top_nodes.push_back(nodes_[c]);
        else
            bottom_nodes.push_back(nodes_[c]);
    }

    float print_size = scale;

    float reducer = 3;
    if(view_mode == 2)
        print_size = scale/reducer;
    if(view_mode == 3)
    {
        reducer *= 2;
        print_size = scale/reducer;
    }

    glScalef(scale, scale, scale);

    this->DrawEdges(scale);

    for(int  c = bottom_nodes.size()-1; c >= 0; c--)
        DrawCircle(bottom_nodes[c], scale, 0.1*scale/reducer, 8);

    for(int c = top_nodes.size()-1; c >= 0; c--)
    {
        DrawCircle(top_nodes[c], scale, 0.1*print_size, 8);
        if(view_mode == 1)
        {
            std::string text = std::to_string(top_nodes[c]->attractiveness());
            DrawNumbers(text, top_nodes[c]->cell()->x/scale, top_nodes[c]->cell()->y/scale);
        }
    }

    glScalef(1.0/scale, 1.0/scale, 1.0/scale);
}

void LAEG::DrawCircle(Node *node, float scale, float r, int num_segments)
{

    float cx = node->cell()->x/scale;
    float cy = node->cell()->y/scale;

    float theta = 2 * M_PI / float(num_segments);
    float c = cos(theta);
    float s = sin(theta);
    float t;

    float x = r; // we start at angle = 0
    float y = 0;

    if(node->type() == OUTER_FRONTIER)
        glColor3fv(color_graph_node_outer_frontier);
    else if(node->type() == INNER_FRONTIER)
        glColor3fv(color_graph_node_inner_frontier);
    else if(node->type() == ROBOT_NODE)
        glColor3fv(color_graph_node_robot);
    else if(node->type() == CONNECTOR)
        glColor3fv(color_graph_node_connector);

    glBegin(GL_POLYGON);
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x + cx, y + cy); // output vertex

        // apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    glEnd();

    x = r;
    y = 0;

    glColor3fv(color_graph_node_border);
    glLineWidth(3);
    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x + cx, y + cy); // output vertex

        // apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    glEnd();
}

void LAEG::DrawEdges(float scale)
{
    std::vector<Node*> nodes;

    glLineWidth(3);
    glBegin(GL_LINES);
    int pad_x, pad_y;

    for(size_t i=0; i<edges_.size(); i++)
    {
        switch (edges_[i]->type())
        {
            case FREE_PATH:
                glColor3fv(color_graph_edge_free_path);
                pad_x = 0;
                pad_y = 0;
            break;

            case HYPOTHETICAL_PATH:
                glColor3fv(color_graph_edge_hypothetical_loop);
                pad_x = 1;
                pad_y = 1;
            break;

            case VISITED_PATH:
                glColor3fv(color_graph_edge_visited_path);
                pad_x = -1;
                pad_y = -1;
            break;

            default:
                glColor3fv(color_graph_edge_standard);
                pad_x = -1;
                pad_y = -1;
            break;
        }

        nodes = edges_[i]->nodes();
        glVertex2f((nodes[0]->cell()->x+pad_x)/scale, (nodes[0]->cell()->y+pad_y)/scale);
        glVertex2f((nodes[1]->cell()->x+pad_x)/scale, (nodes[1]->cell()->y+pad_y)/scale);
    }

    glEnd();

}

int LAEG::next_graph_code()
{
    if(next_graph_code_ > 99999) next_graph_code_ = 1;
    return next_graph_code_++;
}

Edge::Edge(Node *node_1, Node *node_2, int length, bool unknown, bool unvisited)
{
    node_1_ = node_1;
    node_2_ = node_2;
    length_ = length;
    edge_id_ = next_id++;

    unknown_ = unknown;
    unvisited_ = unvisited;

    iteration_code = 0;

    if(unknown_) type_ = HYPOTHETICAL_PATH;
    else if(unvisited_) type_ = FREE_PATH;
    else type_ = VISITED_PATH;
}

float Edge::length()
{
    return length_;
}

void Edge::length(float distance)
{
    length_ = distance;
}

bool Edge::unknown()
{
    return unknown_;
}

bool Edge::unvisited()
{
    return unvisited_;
}

int Edge::id()
{
    return edge_id_;
}

bool Edge::HasNode(int node_id)
{
    if(node_id == node_1_->id() || node_id == node_2_->id()) return true;
    return false;
}

Node *Edge::GetConnectedTo(Node *node)
{
    if(node_1_->id() == node->id()) return node_2_;
    return node_1_;
}

std::vector<Node *> Edge::nodes()
{
    std::vector<Node*> nodes;
    nodes.push_back(node_1_);
    nodes.push_back(node_2_);
    return nodes;
}

EdgeType Edge::type()
{
    return type_;
}

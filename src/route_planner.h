#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() const {return distance;}
    void AStarSearch();

    // The following methods have been made public so we can test them individually.
    void AddNeighbors(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *end_node);
    RouteModel::Node *NextNode();

  private:
    // Add private variables or methods declarations here.
    std::vector<RouteModel::Node*> open_list; // list of open Nodes during AStarSearch()
    RouteModel::Node *start_node;   // the Node that's closest to our start coordinates
    RouteModel::Node *end_node;     // the Node that's closest to our end coordinates

    float distance = 0.0f;          // distance of the final path after AStarSearch()
    RouteModel &m_Model;            // underlying map data model
};

#endif
#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) 
: Model(xml) // XML is parsed here during initialization
{
    // Create RouteModel Nodes, with consecutive indices.
    int counter = 0;
    for (Model::Node node : this->Nodes()) {
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++;
    }
    CreateNodeToRoadHashmap();
}

// creates a Hash lookup table for all Nodes returning the Roads they belong to
void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) { // for all Roads
        if (road.type != Model::Road::Type::Footway) { // excepts Footways
            for (int node_idx : Ways()[road.way].nodes) {  // for all Nodes of the Road's Way
                if (node_to_road.find(node_idx) == node_to_road.end()) {  // check if Node not in hash table
                    node_to_road[node_idx] = std::vector<const Model::Road *> (); // create empty vector
                }
                node_to_road[node_idx].push_back(&road); // add this Road to the vector
            }
        }
    }
}


RouteModel::Node *RouteModel::Node::FindClosestNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;

    for (int node_index : node_indices) {
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}

bool RouteModel::Node::Greater(const Node *a, const Node *b) {
  return (a->g_value + a->h_value > b->g_value + b->h_value);  
}


void RouteModel::Node::FindNeighbors() {
    for (auto & road : parent_model->node_to_road[this->index]) {
        RouteModel::Node *new_neighbor = this->FindClosestNeighbor(parent_model->Ways()[road->way].nodes);
        if (new_neighbor) {
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}


RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    Node input;
    input.x = x;
    input.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist;
    int closest_idx;

    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                dist = input.distance(SNodes()[node_idx]);
                if (dist < min_dist) {
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}
#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        Node * parent = nullptr;    // pointer to previous Node on the Route
        float h_value = std::numeric_limits<float>::max();  // the Heuristic function value
        float g_value = 0.0;        // the actual distance travelled value
        bool visited = false;
        std::vector<Node *> neighbors;  // list of neighboring Nodes

        void FindNeighbors();
        float distance(Node other) const {  // Euclidean distance to another Node
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }
        static bool Greater(const Node *a, const Node *b);  // required for std::sort()

        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        int index;
        Node * FindClosestNeighbor(std::vector<int> node_indices); // returns the closest Node out of a vector of candidates
        RouteModel * parent_model = nullptr;
    };

    RouteModel(const std::vector<std::byte> &xml);  // constructor with bytewise OSM map data XML 
    Node &FindClosestNode(float x, float y);
    auto &SNodes() { return m_Nodes; }
    std::vector<Node> path;
    
  private:
    void CreateNodeToRoadHashmap();
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;  // Hash table for looking up all Roads of a Node
    std::vector<Node> m_Nodes;

};

#endif

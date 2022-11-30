#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto& TempNode : current_node->neighbors){  // loop using references
        TempNode->parent = current_node;
        TempNode->h_value = CalculateHValue(TempNode);
        TempNode->g_value = current_node->g_value + TempNode->distance(*current_node);
        TempNode->visited = true;
        open_list.push_back(TempNode);
  }

}

RouteModel::Node *RoutePlanner::NextNode() {

std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *Node1, const RouteModel::Node *Node2){

      return (*Node1).h_value + (*Node1).g_value > (*Node2).h_value + (*Node2).g_value;
});

  RouteModel::Node *LowestPtr = open_list.back();
  open_list.pop_back();
  return LowestPtr;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node){

        path_found.push_back(*current_node);
        if(current_node->parent){
          distance += current_node->distance(*(current_node->parent));
        }
        current_node = current_node->parent;

    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    current_node = start_node;
    start_node->visited = true;
    open_list.emplace_back(start_node);

    while(!open_list.empty()){
    
      current_node = NextNode();
      if(current_node == end_node){
        m_Model.path = ConstructFinalPath(current_node);
        break;
      }
      else AddNeighbors(current_node);
    }
}
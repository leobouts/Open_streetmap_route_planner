#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  //the m_Model.FindClosestNode method to finds the closest nodes to the starting and ending coordinates.  
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
  

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return end_node->distance(*node);
}


//AddNeighbors method to expands the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    current_node->FindNeighbors();

    for(auto neighbor_node: current_node->neighbors){
        neighbor_node->parent = current_node;
        neighbor_node->g_value = current_node->g_value + current_node->distance(*neighbor_node);
        neighbor_node->h_value = CalculateHValue(neighbor_node);
        neighbor_node->visited = true;
        open_list.push_back(neighbor_node);
    }
}


bool Compare(RouteModel::Node *node_1, RouteModel::Node *node_2) {

  float f1 = node_1->g_value + node_1->h_value; // f1 = g1 + h1
  float f2 = node_2->g_value + node_2->h_value; // f2 = g2 + h2
  return f1 > f2;

}


RouteModel::Node *RoutePlanner::NextNode() {

    // - Sort the open_list according to the sum of the h value and g value.
    // the compare function does the g+h sorting
    std::sort(open_list.begin(), open_list.end(), Compare);

    // - Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node* lowest_sum_node = open_list.back();

    // - Remove that node from the open_list.
    open_list.pop_back();

    // - Return the pointer.
    return lowest_sum_node;

}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // - take the current (final) node as an argument and iteratively follow the 
    //   chain of parents of nodes until the starting node is found.
    while(current_node!= start_node){
        
        // - For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*current_node->parent);        

        // - The returned vector should be in the correct order: the start node should be the first element
        //   of the vector, the end node should be the last element.
        path_found.push_back(*current_node);

        //path_found.insert(path_found.begin(), *current_node->parent);

        current_node = current_node->parent;    
    }
    path_found.push_back(*start_node);
	std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    current_node->visited = true;
  
    while(current_node->distance(*end_node) != 0){
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    
    m_Model.path = ConstructFinalPath(current_node);
}
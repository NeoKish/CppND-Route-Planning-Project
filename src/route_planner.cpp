#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Finding the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  
  	start_node=&m_Model.FindClosestNode(start_x,start_y);
  	end_node=&m_Model.FindClosestNode(end_x,end_y);

}

// Method to calculate h-value

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

	return node->distance(*end_node);

}

// Method to add neighbors for the current node to the open list

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  
  	current_node->FindNeighbors();
  	for(auto neighbor_node:current_node->neighbors){
    
    	neighbor_node->parent=current_node;
      	neighbor_node->g_value=current_node->g_value+current_node->distance(*neighbor_node);
      	neighbor_node->h_value=RoutePlanner::CalculateHValue(neighbor_node);
      	if( !neighbor_node->visited){
        	
          	open_list.emplace_back(neighbor_node);
          	neighbor_node->visited=true;
        	
        }
    
    }

}

// boolean function for sorting the list 
bool Compare(RouteModel::Node const *nodeA, RouteModel::Node const *nodeB){

	return nodeA->g_value+nodeA->h_value > nodeB->g_value+nodeB->h_value;
}

// Method to sort the openlist and return the node with lowest sum of g and h value as pointer

RouteModel::Node *RoutePlanner::NextNode() {

   	std::sort(open_list.begin(),open_list.end(),Compare);
  	RouteModel::Node* low_sum=open_list.back();
    open_list.pop_back();  
  	
  	return low_sum;
  

}

// Function that takes in end node and constructing the final path upto start node using parent attribute

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
  
  	while(current_node->parent!=nullptr){
      
      path_found.emplace_back(*current_node);
      distance+=current_node->distance(*(current_node->parent));
      current_node=current_node->parent;
      
    }
  	if(current_node==start_node){
		path_found.emplace_back(*current_node);    
    }
  	std::reverse(path_found.begin(),path_found.end());
  

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// Method implementing the A-Star Search method

void RoutePlanner::AStarSearch() {
  
    RouteModel::Node *current_node = nullptr;
  	
  	start_node->visited=true;

  	current_node=start_node;
  	
  	RoutePlanner::AddNeighbors(current_node);
  
  	while(open_list.size()>0){
    
    	current_node=RoutePlanner::NextNode();
      	if(current_node==end_node){
        	m_Model.path=RoutePlanner::ConstructFinalPath(current_node);
          	std::cout<<"Reached end node"<<std::endl;

        }
      	else{
        RoutePlanner::AddNeighbors(current_node);
        }

    }

}
// Copyright (c) 2018 joydeepb@cs.umass.edu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <iostream>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "jps.h"
#include "simple_queue.h"
#include "util/timer.h"

using Eigen::Vector2i;
using std::make_pair;
using std::unordered_map;
using std::unordered_set;
using std::vector;

DEFINE_bool(nojps, false, "Disable JPS");
DEFINE_bool(nogui, false, "Disable Visualization");

namespace {
static const uint8_t kBlue[] = {0, 0, 255};
static const uint8_t kRed[] = {255, 0, 0};
static const uint8_t kGreen[] = {0, 255, 0};
static const uint8_t kYellow[] = {255, 255, 0};
static const uint8_t kGrey[] = {128, 128, 128};

}  // namespace

namespace astarplanner {

float Dist(const Node& v1, const Node& v2) {
  return ((v1 - v2).cast<float>().norm());
}

// Returns the heuristic of node n.
// h[n] = straight line distance to goal
float Heuristic(const Node& n, const Node& goal){
  return Dist(n,goal);
}

// Returns the path to goal, in reverse order.
// Note: I added the input param start to indicate the start Node
void AStarPlanner::GetPath(const NodeMap& parent_map,
                           const Node& goal,
			   const Node& start,
                           Path* path_ptr) {
  Node curr = goal;
  while(curr != start){ // while the current node has a parent
    path_ptr->push_back(curr);
    //debugging print
    //std::cout<<curr[0]<<","<<curr[1] <<std::endl; 
    curr = parent_map.at(curr); // don't use curr=parent_map[curr] because the hash map is a const. hash[key] actually creates an entry if it's not already in the hash map
  }
  path_ptr->push_back(start);
}

void AStarPlanner::InitVisualization(const Map& map) {
  display_ = new cimg_library::CImgDisplay(viz_image_, "A* with JPS");
  viz_image_ = cimg_library::CImg<uint8_t>(
      map.width(), map.height(), 1, 3, 0);
  // Draw map
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (map.Occupied(x, y)) {
        viz_image_(x, y, 0, 0) = 255;
        viz_image_(x, y, 0, 1) = 255;
        viz_image_(x, y, 0, 2) = 255;
      }
    }
  }
  acc_viz_image_ = viz_image_;
}

void AStarPlanner::Visualize(const Map& map,
                             const Node& start,
                             const Node& goal,
                             const Node& current,
                             const NodeMap& parent_map) {
  static double t_last_visualized = 0;
  static const double kMinVizInterval = 0.02;
  if (t_last_visualized > GetMonotonicTime() - kMinVizInterval) return;

  for (const auto& p : parent_map) {
    acc_viz_image_.draw_line(
      p.first.x(), p.first.y(), p.second.x(), p.second.y(), kBlue);
  }
  acc_viz_image_.draw_point(current.x(), current.y(), kYellow);

  viz_image_ = acc_viz_image_;

  viz_image_.draw_circle(current.x(), current.y(), 2, kYellow);
  viz_image_.draw_circle(start.x(), start.y(), 2, kRed);
  viz_image_.draw_circle(goal.x(), goal.y(), 2, kGreen);

  display_->display(viz_image_);
  // if (display_->is_key()) exit(0);
  t_last_visualized = GetMonotonicTime();
}

void AStarPlanner::DrawPath(const Path& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    viz_image_.draw_line(
        path[i].x(),
        path[i].y(),
        path[i + 1].x(),
        path[i + 1].y(),
        kGreen);
  }
  viz_image_.draw_circle(
      path[0].x(),
      path[0].y(),
      3,
      kGreen);
  viz_image_.draw_circle(
      path.back().x(),
      path.back().y(),
      3,
      kRed);
}


// Returns a node one step from the init node, in the direction specified as follows:
// direction: 0=right, 1=bottom-right, 2=bottom, 3=bottom-left, 4=left, 5=top-left, 6=top, 7=top-right.
Node AStarPlanner::Step(const Node& init, const int direction){
  Node next;
  switch(direction){
    case(0): // step right
      next = Node(init[0]+1,init[1]);
      break;
    case(1): // step bottom-right
      next = Node(init[0]+1,init[1]+1);
      break;
    case(2): // step bottom
      next = Node(init[0],init[1]+1);
      break;
    case(3): // step bottom-left
      next = Node(init[0]-1,init[1]+1);
      break;
    case(4): // step left
      next = Node(init[0]-1,init[1]);
      break;
    case(5): // step top-left
      next = Node(init[0]-1,init[1]-1);
      break;
    case(6): // step top
      next = Node(init[0],init[1]-1);
      break;
    case(7): // step top-right
      next = Node(init[0]+1,init[1]-1);
      break;
  }
  return next;
}

// helper function
// identifies the jump point from node init, heading in the direction specified by "direction"
// direction: 0=right, 1=bottom-right, 2=bottom, 3=bottom-left, 4=left, 5=top-left, 6=top, 7=top-right.
// Reference: "Online Graph Pruning for Pathfinding on Grid Maps", by Harabor and Grastien. Algorithm 2: Function jump.
Node AStarPlanner::Jump(const Map& map, const Node& init, const int direction, const Node& start, const Node& goal){
  	
  Node n = Step(init,direction);

  Node nullnode(-1,-1);
  // check is n is valid and not an obstacle
  if(!map.ValidNode(n) || map.Occupied(n)){
    return nullnode;
  } 
  // check if n is the goal
  if(n==goal){ return n; }

  // check if n has any forced neighbors
  if(direction==0 || direction==4){ // right or left 
    Node top = Node(n[0],n[1]-1);
    Node bot = Node(n[0],n[1]+1);
    if(map.ValidNode(top)){
      if(map.Occupied(top)) { return n; }
    }
    if(map.ValidNode(bot)){
      if(map.Occupied(bot)) { return n; }
    }
  }
  if(direction==2 || direction==6){ // bottom or top
    Node left = Node(n[0]-1,n[1]);
    Node right = Node(n[0]+1,n[1]);
    if(map.ValidNode(left)){
      if(map.Occupied(left)) { return n; }
    }
    if(map.ValidNode(right)){
      if(map.Occupied(right)) { return n; }
    }
  }
  
  if(direction==1){ // bottom-right
    Node bot = Node(n[0],n[1]+1);
    Node right = Node(n[0]+1,n[1]);
    if(map.ValidNode(bot)){
      if(map.Occupied(bot)) { return n; }
    }
    if(map.ValidNode(right)){
      if(map.Occupied(right)) { return n; }
    }
  }
  if(direction==3){ // bottom-left
    Node bot = Node(n[0],n[1]+1);
    Node left = Node(n[0]-1,n[1]);
    if(map.ValidNode(bot)){
      if(map.Occupied(bot)) { return n; }
    }
    if(map.ValidNode(left)){
      if(map.Occupied(left)) { return n; }
    }
  }  
  if(direction==5){ // top-left
    Node top = Node(n[0],n[1]-1);
    Node left = Node(n[0]-1,n[1]);
    if(map.ValidNode(top)){
      if(map.Occupied(top)) { return n; }
    }
    if(map.ValidNode(left)){
      if(map.Occupied(left)) { return n; }
    }
  }  
  if(direction==7){ // top-right
    Node top = Node(n[0],n[1]-1);
    Node right = Node(n[0]+1,n[1]);
    if(map.ValidNode(top)){
      if(map.Occupied(top)) { return n; }
    }
    if(map.ValidNode(right)){
      if(map.Occupied(right)) { return n; }
    }
  }

  // if d is diagonal
  if(direction==1 || direction==3 || direction==5 || direction==7){
    int d1;
    int d2;
    switch(direction){
      case(1): // bot right
        d1 = 0;
        d2 = 2;
        break;
      case(3): //bot left
        d1 = 2;
        d2 = 4;
        break;
      case(5): // top left
        d1 = 4;
        d2 = 6;
        break;
      case(7): // top right
        d1 = 6;
        d2 = 0;
        break; 
    }
    Node tmp1 = Jump(map, n, d1, start, goal);
    if(tmp1 != Node(-1,-1)){ return n; }
    Node tmp2 = Jump(map, n, d2, start, goal);
    if(tmp2 != Node(-1,-1)){ return n; }
  }

  return Jump(map,n,direction,start,goal);
}

// helper function
// returns the pruned neighbors of curr, taking into account the direction coming from its parent
// if curr is the start node, then we do not prune anything
vector<Node> AStarPlanner::Prune(const Map& map, const Node& curr, const Node& start){
    
   std::vector<Node> pruned_neighbors;
   int curr_x = curr[0];
   int curr_y = curr[1];   
   if(curr==start){
	    std::vector<Node> neighbors;
	    neighbors.push_back(Node(curr_x+1,curr_y));
	    neighbors.push_back(Node(curr_x+1,curr_y+1));
	    neighbors.push_back(Node(curr_x,curr_y+1));
	    neighbors.push_back(Node(curr_x-1,curr_y+1));
	    neighbors.push_back(Node(curr_x-1,curr_y));
	    neighbors.push_back(Node(curr_x-1,curr_y-1));
	    neighbors.push_back(Node(curr_x,curr_y-1));
	    neighbors.push_back(Node(curr_x+1,curr_y-1));

	    for(size_t i=0; i<neighbors.size(); i++){
	      Node tmp = neighbors[i];
	      if(map.ValidNode(tmp)){
		if(!map.Occupied(tmp)){
		  pruned_neighbors.push_back(tmp);
		}
	      }
	    }
	    return pruned_neighbors;
    }

    Node parent = parent_map_[curr];
    // determine direction from parent node to current node
    Node diff = Node(curr_x-parent[0],curr_y-parent[1]);
    
    if(diff[0]>0 && diff[1]==0){ // came from the left
      Node right = Node(curr_x+1,curr_y);
      if(map.ValidNode(right)){
        if(!map.Occupied(right)){pruned_neighbors.push_back(right);}
      }
      Node top = Node(curr_x,curr_y-1);
      Node bot = Node(curr_x,curr_y+1);
      if(map.ValidNode(top)){
        if(map.Occupied(top)){ // forced neighbor
          Node top_right = Node(curr_x+1,curr_y-1);
	  if(map.ValidNode(top_right) && !map.Occupied(top_right)){pruned_neighbors.push_back(top_right);}
        }
      } 
      if(map.ValidNode(bot)){
        if(map.Occupied(bot)){ // forced neighbor
          Node bot_right = Node(curr_x+1,curr_y+1);
	  if(map.ValidNode(bot_right) && !map.Occupied(bot_right)){pruned_neighbors.push_back(bot_right);}
        }
      }           
    }

    else if(diff[0]<0 && diff[1]==0){ // came from the right
      Node left = Node(curr_x-1,curr_y);
      if(map.ValidNode(left)){
        if(!map.Occupied(left)){pruned_neighbors.push_back(left);}
      }
      Node top = Node(curr_x,curr_y-1);
      Node bot = Node(curr_x,curr_y+1);
      if(map.ValidNode(top)){
        if(map.Occupied(top)){ // forced neighbor
          Node top_left = Node(curr_x-1,curr_y-1);
          if(map.ValidNode(top_left) && !map.Occupied(top_left)){pruned_neighbors.push_back(top_left);}
        }
      }
      if(map.ValidNode(bot)){
        if(map.Occupied(bot)){ // forced neighbor
          Node bot_left = Node(curr_x-1,curr_y+1);
          if(map.ValidNode(bot_left) && !map.Occupied(bot_left)){pruned_neighbors.push_back(bot_left);}
        }
      }   
    }

    else if(diff[0]==0 && diff[1]>0){ // came from the top
      Node bot = Node(curr_x,curr_y+1);
      if(map.ValidNode(bot)){
        if(!map.Occupied(bot)){pruned_neighbors.push_back(bot);}
      }
      Node l = Node(curr_x-1,curr_y);
      Node r = Node(curr_x+1,curr_y);
      if(map.ValidNode(l)){
        if(map.Occupied(l)){ // forced neighbor
          Node bot_left = Node(curr_x-1,curr_y+1);
          if(map.ValidNode(bot_left) && !map.Occupied(bot_left)){pruned_neighbors.push_back(bot_left);}
        }
      }
      if(map.ValidNode(r)){
        if(map.Occupied(r)){ // forced neighbor
          Node bot_right = Node(curr_x+1,curr_y+1);
          if(map.ValidNode(bot_right) && !map.Occupied(bot_right)){pruned_neighbors.push_back(bot_right);}
        }
      }
    }

    else if(diff[0]==0 && diff[1]<0){ // came from the bottom
      Node top = Node(curr_x,curr_y-1);
      if(map.ValidNode(top)){
        if(!map.Occupied(top)){pruned_neighbors.push_back(top);}
      }
      Node l = Node(curr_x-1,curr_y);
      Node r = Node(curr_x+1,curr_y);
      if(map.ValidNode(l)){
        if(map.Occupied(l)){ // forced neighbor
          Node top_left = Node(curr_x-1,curr_y-1);
          if(map.ValidNode(top_left) && !map.Occupied(top_left)){pruned_neighbors.push_back(top_left);}
        }
      }
      if(map.ValidNode(r)){
        if(map.Occupied(r)){ // forced neighbor
          Node top_right = Node(curr_x+1,curr_y-1);
          if(map.ValidNode(top_right) && !map.Occupied(top_right)){pruned_neighbors.push_back(top_right);}
        }
      }
    }

    else if(diff[0]<0 && diff[1]<0){ // came from the bottom right
      Node topleft = Node(curr_x-1,curr_y-1);
      Node top = Node(curr_x,curr_y-1);
      Node left = Node(curr_x-1,curr_y);
      if(map.ValidNode(top)){
        if(!map.Occupied(top)){pruned_neighbors.push_back(top);}
      }
      if(map.ValidNode(topleft)){
        if(!map.Occupied(topleft)){pruned_neighbors.push_back(topleft);}
      }
      if(map.ValidNode(left)){
        if(!map.Occupied(left)){pruned_neighbors.push_back(left);}
      }
      Node right = Node(curr_x+1,curr_y);
      Node bot = Node(curr_x,curr_y+1);
      if(map.ValidNode(right)){
        if(map.Occupied(right)){ // forced neighbor
          Node top_right = Node(curr_x+1,curr_y-1);
          if(map.ValidNode(top_right) && !map.Occupied(top_right)){pruned_neighbors.push_back(top_right);}
        }
      }
      if(map.ValidNode(bot)){
        if(map.Occupied(bot)){ // forced neighbor
          Node bot_left = Node(curr_x-1,curr_y+1);
          if(map.ValidNode(bot_left) && !map.Occupied(bot_left)){pruned_neighbors.push_back(bot_left);}
        }
      }
    }


    else if(diff[0]>0 && diff[1]<0){ // came from the bottom left
      Node topright = Node(curr_x+1,curr_y-1);
      Node top = Node(curr_x,curr_y-1);
      Node right = Node(curr_x+1,curr_y);
      if(map.ValidNode(top)){
        if(!map.Occupied(top)){pruned_neighbors.push_back(top);}
      }
      if(map.ValidNode(topright)){
        if(!map.Occupied(topright)){pruned_neighbors.push_back(topright);}
      }
      if(map.ValidNode(right)){
        if(!map.Occupied(right)){pruned_neighbors.push_back(right);}
      }
      Node left = Node(curr_x-1,curr_y);
      Node bot = Node(curr_x,curr_y+1);
      if(map.ValidNode(left)){
        if(map.Occupied(left)){ // forced neighbor
          Node top_left = Node(curr_x-1,curr_y-1);
          if(map.ValidNode(top_left) && !map.Occupied(top_left)){pruned_neighbors.push_back(top_left);}
        }
      }
      if(map.ValidNode(bot)){
        if(map.Occupied(bot)){ // forced neighbor
          Node bot_right = Node(curr_x+1,curr_y+1);
          if(map.ValidNode(bot_right) && !map.Occupied(bot_right)){pruned_neighbors.push_back(bot_right);}
        }
      }
    }


    else if(diff[0]<0 && diff[1]>0){ // came from the top right
      Node botleft = Node(curr_x-1,curr_y+1);
      Node bot = Node(curr_x,curr_y+1);
      Node left = Node(curr_x-1,curr_y);
      if(map.ValidNode(botleft)){
        if(!map.Occupied(botleft)){pruned_neighbors.push_back(botleft);}
      }
      if(map.ValidNode(bot)){
        if(!map.Occupied(bot)){pruned_neighbors.push_back(bot);}
      }
      if(map.ValidNode(left)){
        if(!map.Occupied(left)){pruned_neighbors.push_back(left);}
      }
      Node right = Node(curr_x+1,curr_y);
      Node top = Node(curr_x,curr_y-1);
      if(map.ValidNode(right)){
        if(map.Occupied(right)){ // forced neighbor
          Node bot_right = Node(curr_x+1,curr_y+1);
          if(map.ValidNode(bot_right) && !map.Occupied(bot_right)){pruned_neighbors.push_back(bot_right);}
        }
      }
      if(map.ValidNode(top)){
        if(map.Occupied(top)){ // forced neighbor
          Node top_left = Node(curr_x-1,curr_y-1);
          if(map.ValidNode(top_left) && !map.Occupied(top_left)){pruned_neighbors.push_back(top_left);}
        }
      }
    }


    else if(diff[0]>0 && diff[1]>0){ // came from the top left
      Node botright = Node(curr_x+1,curr_y+1);
      Node right = Node(curr_x+1,curr_y);
      Node bot = Node(curr_x,curr_y+1);
      if(map.ValidNode(botright)){
        if(!map.Occupied(botright)){pruned_neighbors.push_back(botright);}
      }
      if(map.ValidNode(right)){
        if(!map.Occupied(right)){pruned_neighbors.push_back(right);}
      }
      if(map.ValidNode(bot)){
        if(!map.Occupied(bot)){pruned_neighbors.push_back(bot);}
      }
      Node left = Node(curr_x-1,curr_y);
      Node top = Node(curr_x,curr_y-1);
      if(map.ValidNode(left)){
        if(map.Occupied(left)){ // forced neighbor
          Node bot_left = Node(curr_x-1,curr_y+1);
          if(map.ValidNode(bot_left) && !map.Occupied(bot_left)){pruned_neighbors.push_back(bot_left);}
        }
      }
      if(map.ValidNode(top)){
        if(map.Occupied(top)){ // forced neighbor
          Node top_right = Node(curr_x+1,curr_y-1);
          if(map.ValidNode(top_right) && !map.Occupied(top_right)){pruned_neighbors.push_back(top_right);}
        }
      }
    }

    return pruned_neighbors;
}


// helper function
// returns the successors of node x, using JPS pruning and jumping
// Reference: Algorithm 1: Identify Successors, in 
// "Online Graph Pruning for Pathfinding on Grid Maps", by Harabor and Grastien
vector<Node> AStarPlanner::JPSSuccessors(const Map& map, const Node& x, const Node& start, const Node& goal){
  vector<Node> successors;
  vector<Node> neighbors;
  int direction;
  Node diff;
  Node n;
  // prune neighbors of x
  neighbors = Prune(map, x, start);
  // for each neighbor find jump point
  for(size_t i=0; i<neighbors.size(); i++){
    n = neighbors[i];
    // determine direction from x to n
    diff = Node(n[0]-x[0],n[1]-x[1]);
    if(diff[0]>0 && diff[1]==0){ // came from the left
      direction = 0;
    }
    if(diff[0]<0 && diff[1]==0){ // came from the right
      direction = 4;
    }
    if(diff[0]==0 && diff[1]>0){ // came from the top
      direction = 2;
    }
    if(diff[0]==0 && diff[1]<0){ // came from the bottom
      direction = 6;
    }
    if(diff[0]<0 && diff[1]<0){ // came from the bottom right
      direction = 5;
    }
    if(diff[0]<0 && diff[1]>0){ // came from the top right
      direction = 3;
    }
    if(diff[0]>0 && diff[1]<0){ // came from the bottom left
      direction = 7;
    }
    if(diff[0]>0 && diff[1]>0){ // came from the top left
      direction = 1;
    }
    n = Jump(map, x, direction, start, goal);
    successors.push_back(n);
  }

  return successors;
}


bool AStarPlanner::Plan(const Map& map,
                        const Node& start,
                        const Node& goal,
                        Path* path) {
  if (!map.ValidNode(start)) {
    printf("Invalid start location.\n");
    return false;
  }
  if (!map.ValidNode(goal)) {
    printf("Invalid goal location.\n");
    return false;
  }
  if (map.Occupied(start) || map.Occupied(goal)) {
    printf("No path possible. Start unreachable:%d goal unreachable:%d\n",
           1 - static_cast<int>(map.Occupied(start)),
           1 - static_cast<int>(map.Occupied(goal)));
    return false;
  }
  if (!FLAGS_nogui) InitVisualization(map);
  // Initialize parent map.
  parent_map_.clear();
  // Clear all G values.
  g_values_.clear();
  // Initialize an empty priority queue.
  SimpleQueue<Node, AStarPriority> queue;
  // Add start to priority queue.
  const float start_heuristic = Heuristic(start,goal);
  queue.Push(start, AStarPriority(0, start_heuristic));
  g_values_[start] = 0;
  // Clear the closed set.
  closed_set_.clear();

  // ==============================================================
  // Priority queue.
  // ==============================================================
  // Adding a node to the priority queue, with g-value and h-value:
  // queue.Push(node, AStarPriority(g, h));

  // Checking if the queue is empty:
  // if (queue.Empty()) { ... }

  // Get the current node with the highest priority (smallest f = g + h).
  // Node current_node = queue.Pop();

  // ==============================================================
  // Sets.
  // ==============================================================
  // Insert a node into the closed set:
  // closed_set_.insert(node);

  // Check if a node exists in the closed set:
  // if (closed_set_.find(node) == closed_set_.end()) {
  //   // node was not found in the closed set.
  // } else {
  //   // node was found in the closed set.
  // }

  // ==============================================================
  // Parent map and g values are stored using a hash table.
  // ==============================================================
  // If a node does not exist in the hash table, setting its parent or
  // g-value will initialize a new entry in the hash table:
  // g_values_[node] = g;
  // parent_map_[node] = parent_node;

  // If a node already exists in the hash table, you can update its value by
  // setting it to a new value, which looks identical to its creation:
  // g_values_[node] = g;
  // parent_map_[node] = parent_node;

  // Checking if a node exists in the hash table or not:
  // if (g_values_.find(node) == g_values_.end()) {
  //   // Does not exist.
  // } else {
  //   // Exists.
  // }

  // ==============================================================
  // Visualization
  // ==============================================================
  // Visualize the current state of the plan search. Useful to call for every
  // iteration, to visualize the progress of the search.
  // Visualize(map, start, goal, current, parent_map_);

  // starting with Dijkstra (uniform cost search). No heuristics. No jump point search.
  // reference: CS603 Lecture 15 Graph Based Navigation Slide 5
  // TODO: add jump point search. add closed set??

  bool solved = false; 

  // while frontier (queue) is not empty
  while(!queue.Empty()){
    // get node in frontier with smallest cost [f=g+h] 
    Node current = queue.Pop();
    
    // goal test
    if(current == goal){
      // put the solution into path. first node is start. last node is goal.
      /*
      Node itr = goal;
      while(itr != start){
        path->push_back(itr);
	//debugging print
        std::cout<<itr[0]<<","<<itr[1]<<std::endl;
	itr = parent_map_[itr];
      }
      path->push_back(start);
      std::reverse(path->begin(),path->end());
      */
      //debugging print
      std::cout<<"Solution path found!"<<std::endl; 
      solved = true;
      break;
    }

    // go through all the neighbors of current Node
    int curr_x = current[0];
    int curr_y = current[1];
    vector<Node> neighbors;
    neighbors.push_back(Node(curr_x+1,curr_y));
    neighbors.push_back(Node(curr_x+1,curr_y+1));
    neighbors.push_back(Node(curr_x,curr_y+1));
    neighbors.push_back(Node(curr_x-1,curr_y+1));
    neighbors.push_back(Node(curr_x-1,curr_y));
    neighbors.push_back(Node(curr_x-1,curr_y-1));
    neighbors.push_back(Node(curr_x,curr_y-1));
    neighbors.push_back(Node(curr_x+1,curr_y-1));

    for(int i=0;i<8;i++){
      Node next = neighbors[i];
      // make sure neighbor node is valid and is not the parent node of current
      if(map.ValidNode(next)){
	      if(!map.Occupied(next) && next!=parent_map_[current]){
		float new_cost = g_values_[current] + astarplanner::Dist(current, next);
		// if neighbor has not been expanded before or the new cost to neighbor is lower than its previous cost
		if(g_values_.find(next) == g_values_.end() || (new_cost < g_values_[next])){
			  g_values_[next] = new_cost;
		  queue.Push(next, AStarPriority(new_cost, Heuristic(next,goal))); // heuristic used is the straight line distance to goal
		  parent_map_[next] = current; // set parent of neighbor
		}
              }
      }	      
    }

    // visualise search progress
    Visualize(map, start, goal, current, parent_map_);
  }
  

  // To visualize the path found:
  GetPath(parent_map_, goal,start, path);
  if (!FLAGS_nogui) {
    DrawPath(*path);
    display_->display(viz_image_);
    while (!display_->is_closed() && !display_->is_key()) display_->wait();
  }

  if(solved==false){return false;}
  else if(solved==true){return true;}
}

}  // namespace astarplanner

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

#include <cstdint>
#include <unordered_map>
#include <unordered_set>

#include "eigen3/Eigen/Dense"
#include "CImg.h"
#include "imagemap.h"
#ifndef JPS_H
#define JPS_H

namespace astarplanner {

struct NodeHash;

typedef Eigen::Vector2i Node;
typedef ImageMap Map;
typedef std::vector<Node> Path;
typedef std::unordered_map<Node, Node, NodeHash> NodeMap;

// Hashing function to optimize unordered maps with Nodes as keys.
struct NodeHash {
  NodeHash(size_t stride) : kStride(stride) {}
  size_t operator()(const Node& n) const {
    return (n.x() + n.y() * kStride);
  }
  const size_t kStride;
};

// Struct to keep track of node priorities in A*, with tie-breaking to
// prefer expansion of nodes farther along the path.
struct AStarPriority {
  AStarPriority() {}

  AStarPriority(float g, float h) : g(g), h(h) {}

  // Returns true iff this has lower priority than the other.
  // Note that higher cost => lower priority.
  bool operator<(const AStarPriority& other) const {
    const float f = g + h;
    const float f_other = other.g + other.h;
    // This has lower priority if its total cost is higher.
    if (f > f_other + kEpsilon) return true;
    if (f < f_other - kEpsilon) return false;
    // Tie-breaking when costs are the same:
    // This has lower priority if its g-value is lower.
    return (g < other.g);
  }

  // Returns true iff this has higher priority than the other.
  // Note that lower cost => higher priority.
  bool operator>(const AStarPriority& other) const {
    const float f = g + h;
    const float f_other = other.g + other.h;
    // This has higher priority if its total cost is lower.
    if (f < f_other - kEpsilon) return true;
    if (f > f_other + kEpsilon) return false;
    // Tie-breaking when costs are the same:
    // This has higher priority if its g-value is higher.
    return (g > other.g);
  }

  // Epsilon for float comparisons.
  static constexpr float kEpsilon = 1e-3;
  // Cost to go: cost from start to this node.
  float g;
  // Heuristic: estimated cost from this node to the goal.
  float h;
};

class AStarPlanner {
  static constexpr int kMaxNeighbors = 8;
 public:
  explicit AStarPlanner(int max_map_width) :
      kStride(max_map_width),
      kMinLookupSize(max_map_width),
      hash_(kStride),
      parent_map_(kMinLookupSize, hash_),
      g_values_(kMinLookupSize, hash_),
      closed_set_(kMinLookupSize, hash_) {}

  // Note: I added the input param start to indicate the start node.
  void GetPath(const NodeMap& parent_map,
               const Node& goal,
	       const Node& start,
               Path* path_ptr);

  void Visualize(const Map& map,
                 const Node& start,
                 const Node& goal,
                 const Node& current,
                 const NodeMap& parent_map);


  Node Step(const Node& init, const int direction);

  Node Jump(const Map& map, const Node& init, const int direction, const Node& start, const Node& goal);

  std::vector<Node> Prune(const Map& map, const Node& curr, const Node& start);

  std::vector<Node> JPSSuccessors(const Map& map, const Node& curr, const Node& start, const Node& goal);
	  
  // Plan a path from start to goal using A*. Returns true iff a path is found.
  bool Plan(const Map& map,
            const Node& start,
            const Node& goal,
            Path* path,
	    bool verbose,
	    int bb_map[64][64][8][4]);

  void DrawPath(const Path& path);

  void InitVisualization(const Map& map);

  // Stride length of the map, used for hashing.
  const int kStride;
  // Minimum number of entries to allocate memory for, in unordered maps.
  const size_t kMinLookupSize;
  // Node hashing functor.
  NodeHash hash_;
  // Parent map to store naviagtion policy.
  NodeMap parent_map_;
  // G-values of nodes in the open and closed list.
  std::unordered_map<Node, float, NodeHash> g_values_;
  // Closed set (nodes with optimal costs).
  std::unordered_set<Node, NodeHash> closed_set_;
  // Visualization display.
  cimg_library::CImgDisplay* display_ = nullptr;
  // Accumulated visualization image.
  cimg_library::CImg<uint8_t> acc_viz_image_;
  // Current visualization image.
  cimg_library::CImg<uint8_t> viz_image_;
};

}  // namespace astarplanner

#endif  // JPS_H

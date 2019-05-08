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

#include <unordered_set>

#include "eigen3/Eigen/Dense"
#include "CImg.h"

#ifndef IMAGEMAP_H
#define IMAGEMAP_H

namespace astarplanner {

typedef Eigen::Vector2i Node;

// ImageMap wrapper
class ImageMap {
public:
  // Construct an empty map of width w and height h.
  ImageMap(int w, int h);

  // Load a map from an image.
  explicit ImageMap(const char* filename);

  // Returns true iff the specified node is occupied.
  bool Occupied(const Node& n) const;

  // Returns true iff the node at coordinates (x, y) is occupied.
  bool Occupied(const int x, const int y) const;

  // Returns true iff the specified node is within the map dimensions.
  bool ValidNode(const Node& n) const;

  // Returns the width of the map.
  int width() const;

  // Returns the height of the map.
  int height() const;

  // Draws a circular obstacle on the map.
  void DrawCircle(const Node& n, int r);

  // Sets the node (x,y) as occupied.
  void Set(int x, int y);

 private:
  cimg_library::CImg<bool> map_image_;
};

}  // namespace astarplanner

#endif  // IMAGEMAP_H

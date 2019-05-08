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

#include "imagemap.h"
#include "CImg.h"

namespace astarplanner {

int ImageMap::width() const {
  return map_image_.width();
}

int ImageMap::height() const {
  return map_image_.height();
}

ImageMap::ImageMap(int w, int h) : map_image_(w, h, 1, 1, 0) {}

ImageMap::ImageMap(const char* filename) : map_image_(filename) {}

bool ImageMap::Occupied(const int x, const int y) const {
  return (map_image_(x, y));
}

bool ImageMap::Occupied(const Node& n) const {
  return Occupied(n.x(), n.y());
}

void ImageMap::Set(int x, int y) {
  map_image_(x, y) = true;
}

bool ImageMap::ValidNode(const Node& v) const {
  if (v.x() < 0 || v.y() < 0 ||
    v.x() >= map_image_.width() ||
    v.y() >= map_image_.height()) {
    return false;
  }
  return true;
}

void ImageMap::DrawCircle(const Node& n, int r) {
  const bool kObstacleValue = true;
  map_image_.draw_circle(n.x(), n.y(), r, &kObstacleValue);
}

}  // namespace astarplanner

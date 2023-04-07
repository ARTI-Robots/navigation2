#ifndef NAV2_AMCL__KDTREE__POINT_SET_H_
#define NAV2_AMCL__KDTREE__POINT_SET_H_

#include <vector>
#include <cstdlib>
#include <assert.h>

namespace nav2_amcl
{
struct Point
{
  double x;
  double y;
};

typedef std::vector<Point> Points;

// taken from the example pointcloud_kdd_radius
class PointSet
{
public:
  PointSet() = default;

  explicit PointSet(const Points& points)
    : points_(points)
  {
  }

  inline void addPoint(const Point& point)
  {
    points_.push_back(point);
  }

  void clear()
  {
    points_.clear();
  }

  Point operator[](size_t index) const
  {
    return points_[index];
  }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const
  {
    return points_.size();
  }

  // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
  inline double kdtree_distance(const double* p1, const size_t idx_p2, size_t /*size*/) const
  {
    assert(idx_p2 < points_.size());
    const double d0 = p1[0] - points_[idx_p2].x;
    const double d1 = p1[1] - points_[idx_p2].y;
    return d0 * d0 + d1 * d1;
  }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate value, the
  //  "if/else's" are actually solved at compile time.
  inline double kdtree_get_pt(const size_t idx, int dim) const
  {
    assert(idx < points_.size());
    return (dim == 0) ? points_[idx].x : points_[idx].y;
  }

  // Optional bounding-box computation: return false to default to a standard bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template<class BBOX>
  inline bool kdtree_get_bbox(BBOX& /*bb*/) const
  {
    return false;
  }

protected:
  Points points_;
};
}

#endif //NAV2_AMCL__KDTREE__POINT_SET_H_

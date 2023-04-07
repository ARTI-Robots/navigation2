#ifndef NAV2_AMCL__KDTREE__RESULTSET_H_
#define NAV2_AMCL__KDTREE__RESULTSET_H_

#include <limits>
#include <cstdlib>

namespace nav2_amcl
{
class ResultSet
{
public:
  inline ResultSet()
  {
    max_distance = 0.;
    min_distance = std::numeric_limits<double>::max();
    has_point = false;
  }

  inline double worstDist() const
  {
    if (!has_point)
    {
      return std::numeric_limits<double>::max();
    }

    return max_distance;
  }

  inline bool full() const
  {
    return has_point;
  }

  /**
   * Called during search to add an element matching the criteria.
   * @return true if the search should be continued, false if the results are sufficient
   */
  inline bool addPoint(const double dist, size_t index)
  {
    if (dist < min_distance)
    {
      min_distance = dist;
      has_point = true;
      index_of_best_point = index;
    }
    if (dist > max_distance)
    {
      max_distance = dist;
    }

    return true;
  }

  double min_distance;
  double max_distance;
  bool has_point;
  size_t index_of_best_point;
};
}


#endif //NAV2_AMCL__KDTREE__RESULTSET_H_

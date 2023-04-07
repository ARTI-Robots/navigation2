#ifndef NAV2_AMCL__KDTREE__KD_TREE_POINT_2D_VECTOR_H_
#define NAV2_AMCL__KDTREE__KD_TREE_POINT_2D_VECTOR_H_

#include <nav2_amcl/kdtree/nanoflann.hpp>
#include <nav2_amcl/kdtree/point_set.h>
#include <nav2_amcl/kdtree/result_set.h>

namespace nav2_amcl
{
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, nav2_amcl::PointSet>,
  nav2_amcl::PointSet,2> KDTreePoint;

}

#endif //NAV2_AMCL__KDTREE__KD_TREE_POINT_2D_VECTOR_H_

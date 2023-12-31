#include <igl/dijkstra.h>

template <typename IndexType, typename DerivedD, typename DerivedP>
IGL_INLINE int igl::dijkstra_compute_paths(const IndexType &source,
                                           const std::set<IndexType> &targets,
                                           const std::vector<std::vector<IndexType> >& VV,
                                           Eigen::PlainObjectBase<DerivedD> &min_distance,
                                           Eigen::PlainObjectBase<DerivedP> &previous)
{
  int numV = VV.size();
  min_distance.setConstant(numV, 1, std::numeric_limits<typename DerivedD::Scalar>::infinity());
  min_distance[source] = 0;
  previous.setConstant(numV, 1, -1);
  std::set<std::pair<typename DerivedD::Scalar, IndexType> > vertex_queue;
  vertex_queue.insert(std::make_pair(min_distance[source], source));

  while (!vertex_queue.empty())
  {
    typename DerivedD::Scalar dist = vertex_queue.begin()->first;
    IndexType u = vertex_queue.begin()->second;
    vertex_queue.erase(vertex_queue.begin());

    if (targets.find(u)!= targets.end())
      return u;

    // Visit each edge exiting u
    const std::vector<int> &neighbors = VV[u];
    for (std::vector<int>::const_iterator neighbor_iter = neighbors.begin();
         neighbor_iter != neighbors.end();
         neighbor_iter++)
    {
      IndexType v = *neighbor_iter;
      typename DerivedD::Scalar distance_through_u = dist + 1.;
      if (distance_through_u < min_distance[v]) {
        vertex_queue.erase(std::make_pair(min_distance[v], v));

        min_distance[v] = distance_through_u;
        previous[v] = u;
        vertex_queue.insert(std::make_pair(min_distance[v], v));

      }

    }
  }
  //we should never get here
  return -1;
}

template <typename IndexType, typename DerivedP>
IGL_INLINE void igl::dijkstra_get_shortest_path_to(const IndexType &vertex,
                                                   const Eigen::PlainObjectBase<DerivedP> &previous,
                                                   std::vector<IndexType> &path)
{
  IndexType source = vertex;
  path.clear();
  for ( ; source != -1; source = previous[source])
    path.push_back(source);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
template int igl::dijkstra_compute_paths<int, Eigen::VectorXd, Eigen::VectorXi >(int const&, std::set<int, std::less<int>, std::allocator<int> > const&, std::vector<std::vector<int, std::allocator<int> >, std::allocator<std::vector<int, std::allocator<int> > > > const&, Eigen::PlainObjectBase<Eigen::VectorXd >&, Eigen::PlainObjectBase<Eigen::VectorXi >&);
template void igl::dijkstra_get_shortest_path_to<int, Eigen::VectorXi >(int const&, Eigen::PlainObjectBase<Eigen::VectorXi > const&, std::vector<int, std::allocator<int> >&);
#endif

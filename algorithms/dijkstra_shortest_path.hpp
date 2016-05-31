#ifndef DIJKSTRA_SHORTEST_PATH_HPP
#define DIJKSTRA_SHORTEST_PATH_HPP

// Boost
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
  
namespace alg
{  
  typedef boost::adjacency_list < boost::vecS, 
                                  boost::vecS,
                                  boost::undirectedS,
                                  boost::no_property,
                                  boost::property < boost::edge_weight_t, float > > Graph;
                                  
  typedef boost::graph_traits < Graph >::edge_descriptor Edge;
  typedef boost::graph_traits < Graph >::vertex_descriptor Vertex;

  /** \brief Compute the shortest path between two nodes in a graph using
   * Dijkstra's algorithm.
    * \param[in]  edges         edges of the graph
    * \param[in]  edge_weights  weights of the edges
    * \param[in]  start_node_id index of the start node
    * \param[in]  end_node_id   index of the ned node
    * \param[out] path          shortest path from start node to end node represented as a sequence of visited vertices
    * \note edge weights must be non-negative
    * \note uses boost implemntation of dijkstra's algorithm: http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/dijkstra_shortest_paths.html
    * \note vertex indices do not have to be continuous
    * \note if not path between start and end node exists the return path is empty and path cost is equal to -1
    */
  float getShortestPath ( const std::vector<std::pair<int,int> >  &edges, 
                          const std::vector<float>                &edge_weights,
                          const int                               start_node_id,
                          const int                               end_node_id,
                          std::vector<int>                        &path
                        )
  {
    //--------------------------------------------------------------------------
    // Check input
    //--------------------------------------------------------------------------
    
    // Check that edges and edge weights have same size
    if (edges.size() != edge_weights.size())
    {
      std::cout << "[alg::getShortestPath] number of edges and edge weights are different." << std::endl;
      abort();
    }
    
    // Check that start and end vertices are present in the graph
    std::vector<int> vertexIds (edges.size() * 2);
    for (size_t edgeId = 0; edgeId < edges.size(); edgeId++)
    {
      vertexIds[edgeId*2]   = edges[edgeId].first;
      vertexIds[edgeId*2+1] = edges[edgeId].second;
    }
    
    if (  std::find(vertexIds.begin(), vertexIds.end(), start_node_id) == vertexIds.end())
    {
      std::cout << "[alg::getShortestPath] graph does not contain start vertex." << std::endl;
      return -1.0f;
    }

    if (  std::find(vertexIds.begin(), vertexIds.end(), end_node_id) == vertexIds.end())
    {
      std::cout << "[alg::getShortestPath] graph does not contain end vertex." << std::endl;
      return -1.0f;
    }    
    
    //--------------------------------------------------------------------------
    // Convert to Boost grapg
    //--------------------------------------------------------------------------
    
    int numVertices = *std::max_element(vertexIds.begin(), vertexIds.end());
    int numEdges    = edges.size();
        
    Graph g (&edges[0], &edges[0]+numEdges, &edge_weights[0], numVertices);
    
    //--------------------------------------------------------------------------
    // Run Djikstra'sboost 
    //--------------------------------------------------------------------------

    Vertex startVtx = boost::vertex(start_node_id, g);
    Vertex endVtx   = boost::vertex(end_node_id, g);
    std::vector<Vertex> parents(boost::num_vertices(g));
    std::vector<float> distances(boost::num_vertices(g));
 
    // Compute shortest paths from v0 to all vertices, and store the output in parents and distances
    boost::dijkstra_shortest_paths(g, startVtx, boost::predecessor_map(&parents[0]).distance_map(&distances[0]));
    
    //--------------------------------------------------------------------------
    // Get output
    //--------------------------------------------------------------------------
    
    path.resize(0);
    
    if ((distances[end_node_id] == std::numeric_limits<float>::max()))
    {
      return -1.0f;
    }
    else
    {
      Vertex vtxIterator = endVtx;
      while ( vtxIterator !=  startVtx  )
      {
        path.push_back(vtxIterator);
        vtxIterator = parents[vtxIterator];
      }
      path.push_back(startVtx);
      std::reverse(path.begin(), path.end());
      
      return distances[end_node_id];
    }
  }
}

#endif  // DIJKSTRA_SHORTEST_PATH_HPP
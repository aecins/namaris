#ifndef MIN_CUT_HPP
#define MIN_CUT_HPP

// Includes
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

// Typedefs
typedef boost::adjacency_list_traits  < boost::vecS, boost::vecS, boost::directedS > Traits;
typedef boost::adjacency_list         < boost::vecS,                                                                            // Container used for vertices
                                        boost::vecS,                                                                            // Container used for edges
                                        boost::directedS,                                                                       // Directional graph
                                        boost::property < boost::vertex_name_t, std::string,                                    // Vertex properties
                                        boost::property < boost::vertex_index_t, long,
                                        boost::property < boost::vertex_color_t, boost::default_color_type,
                                        boost::property < boost::vertex_distance_t, long,
                                        boost::property < boost::vertex_predecessor_t, Traits::edge_descriptor > > > > >,

                                        boost::property < boost::edge_capacity_t, float,                                       // Edge properties
                                        boost::property < boost::edge_residual_capacity_t, float,
                                        boost::property < boost::edge_reverse_t, Traits::edge_descriptor > > > > Graph;

typedef boost::property_map< Graph, boost::edge_capacity_t >::type CapacityMap;
typedef boost::property_map< Graph, boost::edge_reverse_t>::type ReverseEdgeMap;
typedef boost::property_map< Graph, boost::vertex_color_t, boost::default_color_type>::type VertexColorMap;

////////////////////////////////////////////////////////////////////////////////
bool addEdge (Traits::vertex_descriptor &v1, Traits::vertex_descriptor &v2, Graph &graph, const float weight, CapacityMap &capacity_map, ReverseEdgeMap &reverse_edge_map)
{
  Traits::edge_descriptor edge, reverse_edge;
  bool edge_was_added, reverse_edge_was_added;

  boost::tie (edge, edge_was_added) = boost::add_edge ( v1, v2, graph );
  boost::tie (reverse_edge, reverse_edge_was_added) = boost::add_edge ( v2, v1, graph );
  if ( !edge_was_added || !reverse_edge_was_added )
    return (false);
  
  capacity_map[edge] = weight;
  capacity_map[reverse_edge] = weight;
  reverse_edge_map[edge] = reverse_edge;
  reverse_edge_map[reverse_edge] = edge;
  
  return true;
}
                                    
namespace alg
{
  /** \brief Perform a min cut on a graph
   *  \param[in]  source_potentials   weights between nodes and source node
   *  \param[in]  sink_potentials     weights between nodes and sink node
   *  \param[in]  edges               link structure between nodes
   *  \param[in]  binary_potentials   node link potentials
   *  \param[out] source_points       points belonging to source
   *  \param[out] sink_points         points belonging to sink
   */
  double mincut ( const std::vector<float> &source_potentials, 
                const std::vector<float> &sink_potentials, 
                const std::vector<std::vector<int> > &edges,
                const std::vector <std::vector<float> > &binary_potentials,
                std::vector<int> &source_points,
                std::vector<int> &sink_points
              )
  {
    //////////////////////////////////////////////////////////////////////////////
    // Check input
    if (! (   (source_potentials.size() == sink_potentials.size()) && 
              (source_potentials.size() == edges.size()) && 
              (source_potentials.size() == binary_potentials.size())))
    {
      std::cout << "[alg::minCut] number of input source potentials, sink potentials edges and binary potentials are not equal." << std::endl;
      abort();
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Build graph
    
    int numVertices = source_potentials.size();
    Graph graph;
    std::vector< Traits::vertex_descriptor > vertices;
    Traits::vertex_descriptor source;
    Traits::vertex_descriptor sink;
    CapacityMap capacity          = boost::get (boost::edge_capacity, graph);
    ReverseEdgeMap reverseEdgeMap = boost::get (boost::edge_reverse, graph);
    VertexColorMap vertexColorMap = boost::get (boost::vertex_color, graph);

      // Add vertices
    vertices.resize(numVertices + 2);
    for (size_t i = 0; i < static_cast<size_t>(numVertices + 2); i++)
      vertices[i] = boost::add_vertex(graph);
    
    source  = vertices[source_potentials.size()];
    sink    = vertices[source_potentials.size()+1];
    
    // Add source and sink edges
    for (size_t i = 0; i < static_cast<size_t>(numVertices); i++)
    {
      addEdge(vertices[i], source, graph, source_potentials[i], capacity, reverseEdgeMap);
      addEdge(vertices[i], sink,   graph, sink_potentials[i], capacity, reverseEdgeMap);
    }
    
    // Add binary edges
    for (size_t i = 0; i < edges.size(); i++)
    {
      for (size_t j = 0; j < edges[i].size(); j++)
      {
        Traits::vertex_descriptor v1 = vertices[i];
        Traits::vertex_descriptor v2 = vertices[edges[i][j]];
        float weight = binary_potentials[i][j];
        addEdge(v1, v2, graph, weight, capacity, reverseEdgeMap);
      }
    }
    
    //////////////////////////////////////////////////////////////////////////////  
    // Compute maximim flow
    
    double flow = boost::boykov_kolmogorov_max_flow(graph, source, sink);
    
    //////////////////////////////////////////////////////////////////////////////  
    // Find foreground and background points
    
    source_points.clear();
    sink_points.clear();
    
    for (size_t i = 0; i < static_cast<size_t>(numVertices); i++)
    {    
      if (vertexColorMap(vertices[i]) == 0)
        source_points.push_back(i);
      else
        sink_points.push_back(i);
    }
    
    return flow;
  }
}

# endif // MIN_CUT_HPP
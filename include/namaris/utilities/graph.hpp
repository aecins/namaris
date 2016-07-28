#ifndef GRAPH_UTILITIES_HPP
#define GRAPH_UTILITIES_HPP

// STD includes
#include <queue>

// CPP tools
#include <namaris/utilities/map.hpp>

namespace utl
{
  namespace graph
  {
    /** \brief Data structure representing a graph. Graph is stored as as a
     * vector of vectors. The indices of the outer vector correspond to the
     * indices of the vertices in the graph. The the inner graph correspond to
     * the indices of the vertices that the outter indices of index vertex is
     * connected to.
     */
    typedef std::vector<std::vector<int> > Graph;
    
    /** \brief Data structure representing the weights of the graph edges. */
    typedef std::vector<std::vector<float> > GraphWeights;
    
    /** \brief Data structure representing an edge. Edge is stored as a pair of 
     * integers corresponding to the indices of the vertices that the edge is 
     * connecting.
     */    
    typedef std::pair<int,int> Edge;
    
    /** \brief A vector of edges. */
    typedef std::vector<Edge> Edges;

    /** \brief A vector of edge weights. */
    typedef std::vector<float> EdgeWeights;

    
    /** \brief Add edge to the graph
     *  \param[in] v1    index of first vertex
     *  \param[in] v2    index of second vertex
     *  \param[in] g     graph object
     *  \return    false if any of the vertices is out of bounds, otherwise true
     */
    inline
    bool addEdge(const int &v1, const int &v2, Graph &g)
    {
      // Check that edge is in bounds
      if (static_cast<size_t>(v1) > g.size() || static_cast<size_t>(v2) > g.size() || v1 < 0 || v2 < 0)
      {
        std::cout << "[utl::graph::addEdge] vertex indices are out of bounds." << std::endl;
        return false;
      }
      
      // Check that it is a valid edge
      if (v1 == v2)
      {
        std::cout << "[utl::graph::addEdge] edge must be between two different vertices." << std::endl;
        return false;
      }
      
      // Add edge if it does not exist
      if (std::find(g[v1].begin(), g[v1].end(), v2) == g[v1].end())
        g[v1].push_back(v2);
      
      if (std::find(g[v2].begin(), g[v2].end(), v1) == g[v2].end())
        g[v2].push_back(v1);
      
      return true;
    }

    /** \brief Get number of edges in the graph
     *  \param[in] graph   input graph
     *  \return    number of edges in the graph
     */
    inline
    int getNumEdges (const Graph &g)
    {
      int numEdges = 0;
//       for (size_t vertexId = 0; vertexId < g.size(); vertexId++)
//         numEdges += g[vertexId].size();

      for (size_t vertex1Id = 0; vertex1Id < g.size(); vertex1Id++)
        for (size_t vertex2IdIt = 0; vertex2IdIt < g[vertex1Id].size(); vertex2IdIt++)
          if (vertex1Id > static_cast<size_t>(g[vertex1Id][vertex2IdIt]))
            numEdges++;
      
      return numEdges;
    }
    
    /** \brief Print the edges in the graph
     *  \param[in] graph   input graph
     */
    inline
    void printGraph (const Graph &g)
    {
      for (size_t sourceVtx = 0; sourceVtx < g.size(); sourceVtx++)
      {
        for (size_t targetVtxId = 0; targetVtxId < g[sourceVtx].size(); targetVtxId++)
        {
          int targetVtx = g[sourceVtx][targetVtxId];
          std::cout << "   " << sourceVtx << " -> " << targetVtx << std::endl;
        }
      }    
    }  

    /** \brief Convert a graph to a vector edges.
     *  \param[in]  g   corresponding graph
     *  \return vector of edges
     */
    inline
    Edges graph2Edges (const Graph &g)
    {
      Edges edges;
      
      for (size_t sourceId = 0; sourceId < g.size(); sourceId++)
      {
        for (size_t targetIdIt = 0; targetIdIt < g[sourceId].size(); targetIdIt++)
        {
          int targetId = g[sourceId][targetIdIt];
          if (static_cast<size_t>(targetId) > sourceId)
            edges.push_back(Edge(sourceId, targetId));
        }
      }
      
      return edges;
    }

    /** \brief Given a set of input vertices in a graph find all adjacent
     * vertices and return edges connecting input edges to adjacent edges. Note
     * that none of the input vertices can be in the set of adjacent vertices.
     *  \param[in] g graph
     *  \param[in] v input vertices
     *  \return vector of edges
     */
    inline
    Edges getAdjacentVertexEdges (const Graph &g, const std::vector<int> &v)
    {
      // Find all edges between segment and it's neighbours
      Edges nighbour_edges;
      std::vector<int>::const_iterator v1It = v.begin();
      for ( ; v1It != v.end(); v1It++)
      {
        std::vector<int>::const_iterator v2It = g[*v1It].begin();
        for (; v2It != g[*v1It].end(); v2It++)
        {
          // Check if it is a neighbour segment
          if(std::find(v.begin(), v.end(), *v2It) == v.end())
          {
            nighbour_edges.push_back(Edge(*v1It, *v2It));
          }
        }
      }
      
      return nighbour_edges;
    }

    /** \brief Given a set of vertices in a graph return all edges between
     * the input set of vertices and the rest of the vertices in the graph.
     *  \param[in] g graph
     *  \param[in] v input vertices
     *  \return edges belonging to the cut
     */
    inline
    Edges getCutEdges (const Graph &g, const std::vector<int> &v)
    {
      // Find all edges between segment and it's neighbours
      Edges cut_edges;
      std::vector<int>::const_iterator v1It = v.begin();
      for ( ; v1It != v.end(); v1It++)
      {
        std::vector<int>::const_iterator v2It = g[*v1It].begin();
        for (; v2It != g[*v1It].end(); v2It++)
        {
          // Check if it is a neighbour segment
          if(std::find(v.begin(), v.end(), *v2It) == v.end())
          {
            cut_edges.push_back(Edge (*v1It, *v2It));
          }
        }
      }
      
      return cut_edges;
    }  
    
    /** \brief Given a set of vertices in a graph return the edges of the
     *  subgraph formed by these vertices.
     *  \param[in] g graph
     *  \param[in] v subgraph vertices
     *  \return vector of edges
     */
    inline
    Edges getSubgraphEdges (const Graph &g, const std::vector<int> &v)
    {
      // Find all edges between segment and it's neighbours
      Edges subgraph_edges;
      std::vector<int>::const_iterator v1It = v.begin();
      for ( ; v1It != v.end(); v1It++)
      {
        std::vector<int>::const_iterator v2It = g[*v1It].begin();
        for (; v2It != g[*v1It].end(); v2It++)
        {
          // Check if it is a neighbour segment
          if(std::find(v.begin(), v.end(), *v2It) != v.end())
          {
            subgraph_edges.push_back(Edge (*v1It, *v2It));
          }
        }
      }
      
      return subgraph_edges;
    }  
    
    /** \brief Find leaves in a graph
     *  \param[in]   g input graph
     *  \return indices of leaf vertices
     */
    inline
    std::vector<int> getGraphLeaves ( const Graph  &g)
    {    
      // Leaves are edges with a single outgoing edge
      std::vector<int> leaves;
      for (size_t vertexId = 0; vertexId < g.size(); vertexId++)
      {
        if (g[vertexId].size() == 1)
          leaves.push_back(vertexId);
      }    
      return leaves;
    }  
    
    /** \brief Find connected components in the graph
     *  \param[in] g              graph object
     *  \param[in] min_cc_size    minimum size of a valid connected component (default 0)
     *  \return    a vector of vectors where each inner vector corresponds to a 
     *             connected component and stores the indices of vertices belonging
     *             to it.
     */
    inline
    utl::map::Map getConnectedComponents(const Graph &g, const int min_cc_size = 0)
    {
      std::vector<bool> visited (g.size(), false);
      std::vector<std::vector<int> > CCs;
      
      for (size_t vertexId = 0; vertexId < g.size(); vertexId++)
      {
        // If node has already been visited - skip
        if (visited[vertexId])
          continue;
        
        // Run breadth-first search from current vertex
        std::queue<int> vertexQueue;
        std::vector<int> CC;
        vertexQueue.push(vertexId);
        visited[vertexId] = true;
        
        while (!vertexQueue.empty())
        {
          int curVertex = vertexQueue.front();
          vertexQueue.pop();
          
          CC.push_back(curVertex);
          for (size_t nbrVertexId = 0; nbrVertexId < g[curVertex].size(); nbrVertexId++)
          {
            int nbrVertex = g[curVertex][nbrVertexId];
            
            if (!visited[nbrVertex])
            {
              vertexQueue.push(nbrVertex);
              visited[nbrVertex] = true;
            }
          }
        }
        if (static_cast<int>(CC.size()) > min_cc_size)
          CCs.push_back(CC);
      }
      
      return CCs;
    }
    
    /** \brief Find the longest path in an undirected acyclic unweighted graph starting
     * from a given vertex. Search is performed using recursive depth first 
     * search.
     *  \param[in]   graph             input graph
     *  \param[in]   vertex_start      frst vertex in the path
     *  \param[in]   visited_vertices  vertices that have already been visited
     *  \return a sequence of vertices in the longest path
     */
    inline
    std::vector<int> getLongestPath ( const Graph         &graph,
                                      const int           &vertex_start,
                                      std::vector<bool>   &visited_vertices
                        )
    {
      // Mark start vertex as visited
      visited_vertices[vertex_start] = true;
      
      // Find all adjacent vertices of the start vertex
      std::vector<int> neighbors = graph[vertex_start];
      
      // Remove all neighbours that have already been visited
      std::vector<int>::iterator endIt = std::remove_if(neighbors.begin(), neighbors.end(), [&visited_vertices](int i) { return visited_vertices[i]; });
      neighbors.erase(endIt, neighbors.end());    
      
      std::vector<int> longestPath (0);
      
      // If there are no neighbors that have not been visited this must be a leaf
      // so we return it's index
      if (neighbors.size() == 0)
      {
        longestPath.push_back(vertex_start);
      }
      
      // Otherise call the method on all the neighbors
      else
      {
        // Find all paths from current vertex
        std::vector<std::vector<int> > paths;
        for (std::vector<int>::iterator nbrIdItr = neighbors.begin(); nbrIdItr != neighbors.end(); nbrIdItr++)
        {
          std::vector<int> curPath = getLongestPath(graph, *nbrIdItr, visited_vertices);
          curPath.push_back(vertex_start);
          
          paths.push_back(curPath);
        }
        
        // Return the longest path
        longestPath = *std::max_element(paths.begin(), paths.end(), [](std::vector<int> p1, std::vector<int> p2) {return p1.size() < p2.size();});
      }
        
      return longestPath;
    }
    
    /** \brief Find the longest path in an undirected acyclic unweighted graph.
     *  \param[in]   graph             input graph
     *  \return a sequence of vertices in the longest path
     */
    inline
    std::vector<int> getLongestPath ( const Graph &graph)
    {
      std::vector<int> leaves = getGraphLeaves(graph);
      std::vector<std::vector<int> > paths(leaves.size());
      for (size_t leafId = 0; leafId < leaves.size(); leafId++)
      {
        std::vector<bool> visitedVertices (graph.size(), false);
        paths[leafId] = getLongestPath(graph, leaves[leafId], visitedVertices);
      }
      
      return *std::max_element(paths.begin(), paths.end(), [](std::vector<int> p1, std::vector<int> p2) {return p1.size() < p2.size();});
    }
        
    /** \brief Create an empty graph weight datastructure with the same 
     *  dimensionality as a prototype graph.
     *  \param[in] g     graph
     *  \return graph weights datastructure
     */
    inline
    GraphWeights createGraphWeights (const Graph &g)
    {
      GraphWeights gw (g.size());
      
      for (size_t vertexId = 0; vertexId < g.size(); vertexId++)
        gw[vertexId].resize(g[vertexId].size());
      
      return gw;
    }
    
    /** \brief Get an edge weight from weighted graph.
     *  \param[in]  g         graph
     *  \param[in]  gw        corresponding weights
     *  \param[in]  v1        first vertex of edge
     *  \param[in]  v2        second vertex of edge
     *  \param[out] weight    edge weight
     *  \return false if provided edge does not exist in the graph
     */
    inline
    bool getEdgeWeight (const Graph &g, const GraphWeights &gw, const int &v1, const int &v2, float &weight)
    {
      if (static_cast<size_t>(v1) > g.size() || static_cast<size_t>(v2) > g.size())
      {
        std::cout << "[utl::graph::getEdgeWeight] input edge vertices are out of bounds" << std::cout;
        return false;
      }
      
      // Find iterators to elements
      std::vector<int>::const_iterator v1It = std::find(g[v2].begin(), g[v2].end(), v1);
      std::vector<int>::const_iterator v2It = std::find(g[v1].begin(), g[v1].end(), v2);
      
      // If they are out of bounds - return false
      if ((v1It == g[v2].end()) || (v2It == g[v1].end()))
      {
        std::cout << "[utl::graph::getEdgeWeight] input edge does not exist in the graph or graph is corrupted" << std::cout;
        return false;
      }
      
      // Otherwise update the weights
      int v2Id = v2It - g[v1].begin();
      int v1Id = v1It - g[v2].begin();
      
      if (gw[v1][v2Id] != gw[v2][v1Id])
      {
        std::cout << "[utl::graph::getEdgeWeight] edge weights differ for (v1, v2) and (v2, v1)" << std::cout;
        return false;
      }
      
      weight = gw[v1][v2Id];
          
      return true;
    }
    
    /** \brief Given a weighted graph get all weights for a set of query edges.
     *  \param[in]  g             graph
     *  \param[in]  gw            corresponding weights
     *  \param[in]  edges         query edges
     *  \param[out] edge_weights  weights of query edges
     *  \return false if any of the provided edges do not exist in the graph
     */
    inline
    bool getEdgeWeights (const Graph &g, const GraphWeights &gw, const Edges &edges, EdgeWeights &edge_weights)
    {
      edge_weights.resize(edges.size());
      
      for (size_t edgeId = 0; edgeId < edges.size(); edgeId++)
      {
        bool good = utl::graph::getEdgeWeight(g, gw, edges[edgeId].first, edges[edgeId].second, edge_weights[edgeId]);
        if (!good)
          return false;
      }
      
      return true;
    }    
    
    /** \brief Add an edge weight to the graph weight datastructure
     *  \param[in,out]   gw        weighted graph
     *  \param[in]       g         corresponding graph
     *  \param[in]       v1        first vertex of edge
     *  \param[in]       v2        second vertex of edge
     *  \param[in]       weight    edge weight
     *  \return false if provided edge does not exist in the graph
     *  \note if edge weight already exists it is overwritten
     */
    inline
    bool setEdgeWeight (const Graph &g, GraphWeights &gw, const int &v1, const int &v2, const float &weight)
    {
      if (static_cast<size_t>(v1) > g.size() || static_cast<size_t>(v2) > g.size())
      {
        std::cout << "[utl::graph::setEdgeWeight] input edge vertices are out of bounds\n";
        return false;
      }
      
      // Find iterators to elements
      std::vector<int>::const_iterator v1It = std::find(g[v2].begin(), g[v2].end(), v1);
      std::vector<int>::const_iterator v2It = std::find(g[v1].begin(), g[v1].end(), v2);
      
      // If they are out of bounds - return false
      if ((v1It == g[v2].end()) || (v2It == g[v1].end()))
      {
        std::cout << "[utl::graph::setEdgeWeight] input edge does not exist in the graph or graph is corrupted\n";
        return false;
      }
      
      // Otherwise update the weights
      int v2Id = v2It - g[v1].begin();
      int v1Id = v1It - g[v2].begin();
      gw[v1][v2Id] = weight;
      gw[v2][v1Id] = weight;
          
      return true;
    }
      
    /** \brief Add a weighted edge to the weighted graph datastructure. If edge already
     *  exists it's weight is overwritten.
     *  \param[in,out]   g         graph
     *  \param[in,out]   gw        graph weights
     *  \param[in]       v1        first vertex of edge
     *  \param[in]       v2        second vertex of edge
     *  \param[in]       weight    edge weight
     *  \return false if provided edge does not exist in the graph
     */
    inline  
    bool addWeightedEdge (Graph &g, GraphWeights &gw, const int &v1, const int &v2, const float &weight)
    {
      // Check that vertices are in bounds
      if (static_cast<size_t>(v1) > g.size() || static_cast<size_t>(v2) > g.size() || v1 < 0 || v2 < 0)
      {
        std::cout << "[utl::graph::addWeightedEdge] vertex indices are out of bounds." << std::endl;
        return false;
      }
      
      // Check that it is a valid edge
      if (v1 == v2)
      {
        std::cout << "[utl::graph::addWeightedEdge] edge must be between two different vertices." << std::endl;
        return false;
      }
      
      // Find iterators to elements
      std::vector<int>::const_iterator v1It = std::find(g[v2].begin(), g[v2].end(), v1);
      std::vector<int>::const_iterator v2It = std::find(g[v1].begin(), g[v1].end(), v2);

      // If edge doesn't exist create new one
      if (v1It == g[v2].end() && v2It == g[v1].end())
      {
        g[v1].push_back(v2);
        g[v2].push_back(v1);
        gw[v1].push_back(weight);
        gw[v2].push_back(weight);
      }    
      
      // If edge already exists - just update the weight
      else
      {
        int v2Id = v2It - g[v1].begin();
        int v1Id = v1It - g[v2].begin();
        gw[v1][v2Id] = weight;
        gw[v2][v1Id] = weight;      
      }
      
      return true;
    }  
    
//     /** \brief Convert graph to a vector of pairs representing edges
//      *  \param[in]       g         corresponding graph
//      *  \return vector of pairs where each pair represents an edge
//      */
//     inline
//     void graphWeighted2EdgePairs (const Graph &g, const GraphWeights &w, std::vector<std::pair<int,int> > &edge_pairs, std::vector<float> &edge_pair_weights)
//     {
//       // Get edge pairs
//       edge_pairs = graph2EdgePairs(g);
//       
//       // Get edge pair weights
//       edge_pair_weights.resize(edge_pairs.size());
//       for (size_t edgeId = 0; edgeId < edge_pairs.size(); edgeId++)    
//       {
//         int sourceId = edge_pairs[edgeId].first;
//         int targetId = edge_pairs[edgeId].second;
//         getEdgeWeight(g, w, sourceId, targetId, edge_pair_weights[edgeId]);
//       }
//     }
   
    /** \brief Convert a graph and corresponding graph weights to a vector of 
     * edges and corresponding vector of weights.
     *  \param[in]  g             graph
     *  \param[in]  gw            graph weights
     *  \param[in]  edges         vector of edges
     *  \param[in]  edge_weights  vector of edge weights
     */
    inline
    void graph2EdgesWeighted  ( const Graph &g,
                                const GraphWeights &gw,
                                Edges &edges,
                                EdgeWeights &edge_weights
                               )
    {
      // Get edges
      edges = graph2Edges(g);
      
      // Get edge weights
      edge_weights.resize(edges.size());
      for (size_t edgeId = 0; edgeId < edges.size(); edgeId++)    
      {
        int sourceId = edges[edgeId].first;
        int targetId = edges[edgeId].second;
        getEdgeWeight(g, gw, sourceId, targetId, edge_weights[edgeId]);
      }
    }   
   
    /** \brief Check that a graph data structure and a corresponding weight
     * data structure are in sync i.e. have same dimensions.
     *  \param[in]  g   graph
     *  \param[in]  gw  graph weights
     *  \return TRUE if data structures are in sync
     */
    inline
    bool chechWeightedGraphCorruption ( const Graph &g, const GraphWeights &gw)
    {
      // Check vertex sizes
      if (g.size() != gw.size())
      {
        std::cout << "[utl::graph::chechWeightedGraphCorruption] graph and graph weights have different sizes (" << g.size() << ", " << gw.size() << ")" << std::endl;
        return false;
      }
      
      // Check sizes of edges
      bool good = true;
      for (size_t v1 = 0; v1 < g.size(); v1++)
      {
        if (g[v1].size() != gw[v1].size())
        {
          std::cout << "[utl::graph::chechWeightedGraphCorruption] graph edges and graph weights edges have different sizes (" << v1 << ": " << g[v1].size() << ", " << gw[v1].size() << ")" << std::endl;
        }
      }
      
      return good;    
    }
  }
}

#endif  // GRAPH_UTILITIES_HPP
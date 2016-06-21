#ifndef MAP_UTILITIES_HPP
#define MAP_UTILITIES_HPP


// CPP tools
#include <namaris/utilities/std_vector.hpp>

namespace utl
{  
  namespace map
  {
    /** \brief A one to many map.
     * 
     * Given two sets of unsigned integers a map defines a correspondence
     * between the elements of the first set (domain) and elements of the second
     * set (range). The outer vector corresponds to domain while inner vectors 
     * correspond to range.
     */
    typedef std::vector<std::vector<int> > Map;    // A map from points to segments

    /** \brief Get maximum domain element id
     *  \param[in]  map  input map
     *  \return     domain maximum element id
     */
    int domainMax (const Map &map)
    {
      return map.size();
    }

    /** \brief Get maximum domain element id
     *  \param[in]  map  input map
     *  \return     domain maximum element id
     */
    int rangeMax (const Map &map)
    {
      int maxId = 0;
      for (size_t i = 0; i < map.size(); i++)
        if (map[i].size() > 0)
          maxId = std::max(maxId, stdvec::vectorMax<int>(map[i]));
      
      return maxId;
    }

    /** \brief Get minimum domain element id
     *  \param[in]  map  input map
     *  \return     domain minimum element id
     */
    int rangeMin (const Map &map)
    {
      int minId = 0;
      for (size_t i = 0; i < map.size(); i++)
        if (map[i].size() > 0)
          minId = std::min(minId, stdvec::vectorMin<int>(map[i]));
      
      return minId;
    }
        
    /** \brief Checks if a map is injective i.e. if for each domain element 
     * there is only one corresponding range element
     *  \param[in]  map input map
     *  \return     true if map is injective
     */
    bool isInjective (const Map &map)
    {
      // If any of the mapped values correspond to 
      for (size_t mappedId = 0; mappedId < map.size(); mappedId++)
        if (map[mappedId].size() > 1)
          return false;
        
      return true;
    }    
    
    /** \brief Invert a map.
     *  \param[in]  map input map
     *  \return     domain maximum element id
     */
    Map invertMap (const Map &map)
    {
      // Initialize map
      int maxRangeId = rangeMax(map);
      Map inverseMap (maxRangeId+1);
      if (maxRangeId < 1)
      {
        std::cout << "[utl::map::inverse] maximum range element id is 0. Inverse map is empty." << std::endl;
        abort();
      }
      
      for (size_t domId = 0; domId < map.size(); domId++)
        for (size_t rangeIdIt = 0; rangeIdIt < map[domId].size(); rangeIdIt++)
        {
          int rangeId = map[domId][rangeIdIt];
          if (rangeId < 0)
          {
            std::cout << "[utl::map::inverse] range element value is negative. Cannot invert." << std::endl;
            abort();
          }
          inverseMap[rangeId].push_back(domId);
        }
      
      return inverseMap;
    }
    
    /** \brief Remap values in a vector using a map
     *  \param[in]  v input vector
     *  \param[in]  map input map
     *  \param[in]  check_injective bool indicating whether to check if input mapping is injective
     *  \return     mapped vector
     */
    std::vector<int> remapVector (const std::vector<int> &v, const Map &map, bool check_injective = true)
    { 
      // First check that remapping is possible
      if (check_injective && !isInjective(map))
      {
        std::cout << "[utl::map::mapVector] mapping is not injective, mapping is not possible." << std::endl;
        abort();
      }
      
      int vectorMax     = utl::stdvec::vectorMax<int>(v);
      int vectorMin     = utl::stdvec::vectorMin<int>(v);
      int mapDomainMax  = domainMax(map);
      
      if (vectorMin < 0)
      {
        std::cout << "[utl::map::mapVector] vector has negative values, mapping is not possible." << std::endl;
        abort();
      }
      
      if (vectorMax > mapDomainMax)
      {
        std::cout << "[utl::map::mapVector] vector maximum value is greater than map domain maximum value, mapping is not possible." << std::endl;
        abort();
      }
      
      // Map!
      std::vector<int> v_mapped (v.size());
      for (size_t vIt = 0; vIt < v.size(); vIt++)
      {
        int domainVal = v[vIt];
        if (map[domainVal].size() == 0)
        {
          std::cout << "[utl::map::mapVector] map has no range value for domain value, mapping not possible " << domainVal << std::endl;
          abort();
        }
        v_mapped[vIt] = map[domainVal][0];
      }
      
      return v_mapped;
    }
    
    /** \brief Remap range values of a map using another map
     *  \param[in]  map_src map to be remapped
     *  \param[in]  map map used for remapping
     *  \return     remapped source map
     */
    std::vector<std::vector<int> > remapMap (const Map &map_src, const Map &map)
    { 
      // First check that remapping is possible
      if (!isInjective(map))
      {
        std::cout << "[utl::map::mapMap] mapping is not injective, mapping is not possible." << std::endl;
        abort();
      }
      
      // Map!
      Map mas_src_remapped (map_src.size());
      for (size_t vId = 0; vId < map_src.size(); vId++)
        mas_src_remapped[vId] = remapVector(map_src[vId], map, false);
      
      return mas_src_remapped;
    }    
  }
}

#endif  // SEGMENTATION_UTILITIES_HPP
#ifndef PCL_VISUALIZATION_COLOR_H
#define PCL_VISUALIZATION_COLOR_H

#include <vector>

namespace utl
{
  
  namespace pclvis
  {
    struct Color
    {
      double r;
      double g;
      double b;
      
      Color ()
        : r (-1)
        , g (-1)
        , b (-1)
      {};
      
      Color (const double r, const double g, const double b)
        : r (r)
        , g (g)
        , b (b)
      {};
      
      std::vector<double> toStdVec ()
      {
        std::vector<double> c (3);
        c[0] = r; c[1] = g; c[2] = b;
        return c;
      }
      
      bool operator==(const Color& rhs) const
      {
        return (r == rhs.r && b == rhs.b && g == rhs.b);
      }
      
      bool operator!=(const Color& rhs) const
      {
        return (r != rhs.r && b != rhs.b && g != rhs.b);
      }
    };
        
    typedef std::vector<Color> Colors;
  }
}

#endif  // PCL_VISUALIZATION_COLOR_H
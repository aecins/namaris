#ifndef STRING_UTILITIES_HPP
#define STRING_UTILITIES_HPP

// STD includes
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace utl
{
  namespace str
  {
    /** \brief Convert a decimal number to a string of fixed length with zero padding on the left.
     *  \param[in] decimal_number number to convert to string
     *  \param[in] decimal_places number of decimal places for a floating point number
     */
    template <typename T> inline
    std::string to_padded_string (const  T decimal_number, const unsigned int padded_width = 6)
    {
      std::ostringstream out;
      out << std::setfill('0') << std::setw(padded_width);
      out << decimal_number;
      return out.str();
    }
    /** \brief Convert a number to a string
     *  \param[in] number string which is modified
     *  \param[in] decimal_places number of decimal places for a floating point number
     */
    template <typename T> inline
    std::string to_string (const  T number, const unsigned int decimal_places = 6)
    {
        std::ostringstream out;
        out << std::fixed;
        out << std::setprecision(decimal_places) << number;
        return out.str();
    }    
    
    /** \brief Replace all occurences of a substring with a new substring
     *  \param[in,out] string_in string which is modified
     *  \param[in] substring_orig substring that needs to be replaced
     *  \param[in] substring_new new substring
     */
    inline
    void replaceSubstring(std::string &string_in, const std::string &substring_orig, const std::string &substring_new)
    {
      std::string::size_type n = 0;
      while ( ( n = string_in.find( substring_orig, n ) ) != std::string::npos )
      {
          string_in.replace( n, substring_orig.size(), substring_new );
          n += substring_new.size();
      }  
    }

    /** \brief Split a string by a delimeter
     *  \param[in] path string to be separated
     *  \param[in] delimiter delimiter
     *  \return a vector of delimited substrings
     *  \NOTE: http://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
     */
    inline
    std::vector<std::string> splitString (const std::string &line, const std::string delimiter = " ")
    {
      std::vector<std::string> substrings;
      
      auto start = 0U;
      auto end = line.find(delimiter);
      while (end != std::string::npos)
      {
        substrings.push_back(line.substr(start, end - start));
        start = end + delimiter.length();
        end = line.find(delimiter, start);
      }
      
      end = line.length();
      substrings.push_back(line.substr(start, end - start));
      
      return substrings;
    }  
  }
}

#endif    // STRING_UTILITIES_HPP
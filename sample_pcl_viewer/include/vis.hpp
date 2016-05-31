#ifndef VIS_HPP_
#define VIS_HPP_

#include <utilities/pcl_visualization.hpp>

// State
struct VisState
{
  VisState ()
    : cloudDisplayState_ (CLOUD_NO_COLOR)
    , showNormals_(false)
    , updateDisplay_(true)
    , pointSize_ (4.0)
  { };
  
  enum CloudDisplay { CLOUD_NO_COLOR, CLOUD_COLOR, CLOUD_DISTANCE_TO_ORIGIN };
  
  CloudDisplay cloudDisplayState_;
  bool showNormals_;
  bool updateDisplay_;
  float pointSize_;
};

// Callback
void keyboard_callback (const pcl::visualization::KeyboardEvent &event, void *cookie)
{
  VisState* visState = reinterpret_cast<VisState*> (cookie);
  
  if (event.keyUp ())
  {    
    std::string key = event.getKeySym ();
//     cout << key << " key pressed!\n";
    
    visState->updateDisplay_ = true;
    
    if (key == "KP_1")
      visState->cloudDisplayState_ = VisState::CLOUD_NO_COLOR;
    else if (key == "KP_2")
      visState->cloudDisplayState_ = VisState::CLOUD_COLOR;
    else if (key == "KP_3")
      visState->cloudDisplayState_ = VisState::CLOUD_DISTANCE_TO_ORIGIN;

    // Point size
    else if (key == "KP_Add")
      visState->pointSize_ += 1.0;
    else if (key == "KP_Subtract")
      visState->pointSize_ = std::max(0.0, visState->pointSize_ - 1.0);    
        
    // More stuff
    else if ((key == "n") || (key == "N"))
      visState->showNormals_ = !visState->showNormals_;
    
    else
      visState->updateDisplay_ = false;
  }
}

#endif // VIS_HPP_
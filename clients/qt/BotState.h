#ifndef __BOTSTATE__
#define __BOTSTATE__
#include <eigen3/Eigen/Core>

class BotState {
public:
  Eigen::Vector3d _position;
  bool _isYellow;
  
  BotState(bool isYellow, Eigen::Vector3d pos = Eigen::Vector3d(0,0,0)) {
    _position = pos;
    _isYellow = isYellow;
  }
  
};
#endif
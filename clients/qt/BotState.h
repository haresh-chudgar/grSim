#ifndef __BOTSTATE__
#define __BOTSTATE__
#include <eigen3/Eigen/Core>
#include <math.h>

class BotState {
public:
  Eigen::Vector3d _position;
  bool _isYellow;
  
  BotState(bool isYellow, Eigen::Vector3d pos = Eigen::Vector3d(0,0,0)) {
    _position = pos;
    _isYellow = isYellow;
  }
  
  double distanceToLocation(Eigen::Vector3d loc) {
    return sqrt(pow(_position[0] - loc[0],2) + pow(_position[1] - loc[1],2));
  }
};
#endif
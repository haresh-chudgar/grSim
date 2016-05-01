#ifndef __BOTSTATE__
#define __BOTSTATE__
#include <eigen3/Eigen/Core>
#include <math.h>

class BotState {
public:
  // x, y, theta
  Eigen::Vector3d _position;
  int _id;
 
  // vX, vY, theta
  Eigen::Vector3d _velocity;
  bool _isYellow;
  
  BotState(bool isYellow, Eigen::Vector3d pos = Eigen::Vector3d(0,0,0), int id = 0) {
    _position = pos;
    _isYellow = isYellow;
    _id = id;
  }
  
  double distanceToLocationFromSpinner(Eigen::Vector3d loc) {
    double x = loc[0] - 100*cos(_position[2]);
    double y = loc[1] - 100*sin(_position[2]);
    Eigen::Vector3d locMinusSpinnerPos(x, y, loc[0]); 
    return distanceToLocation(locMinusSpinnerPos);
  }

  double distanceToLocation(Eigen::Vector3d loc) {
    return sqrt(pow(_position[0] - loc[0],2) + pow(_position[1] - loc[1],2));
  }
};
#endif

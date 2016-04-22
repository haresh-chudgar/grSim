#ifndef GamePlan
#define GamePlan

#include "communicator.h"
#include "robot.h"
#include "trajectoryplanner.h"
#include <eigen3/Eigen/Core>

using namespace std; 
using namespace Eigen;

class GamePlan{
public:
    GamePlan(const bool team, 
               Communicator* communicator, 
               TrajectoryPlanner* planner,
               const int num_robots);
    ~GamePlan();
    void SimCallback(Vector3d ball, vector<Vector3d> robots);
    vector<Robot*> StartRobots(int num_robots);
private:
  const char* _team;
  Communicator* _communicator;
  TrajectoryPlanner* _planner;
  const int _num_robots;
  vector<Robot*> _robots;
};

#endif // GamePlan
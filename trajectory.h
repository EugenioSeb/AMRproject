#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "robtimer.h"
#include "obstacle.h"
#include "utils.h"
#include <math.h>
#include <vector>
#include <list>

#define DIST 0.5f
#define DEF_LENGHT 1.0f

using namespace std;


class Trajectory
{
public:
  //create a trajectory and fix the constant velocity
  Trajectory(float velocity, vec start, vec nextCheckpoint);

  //ad a checkpoint at the end
  void pushBackCheckpoint(vec checkpoint);

  //change the trajectory adding a checkpoint near the collisionPont.
  void addCheckpoint(vec checkpoint);

  //given the time, position and velocity of the robot are provided as outputs.
  void getPositioVelocity(float time, vec &position, vec &velocity);

  //return the point of the first collision NULL otherwise
  bool isCollision(obstacle& obs);

  //make a chekpointr after dist metes from the robot position
  void makeForwardCheckpoint(float distance);
  void manageCollision();

private:
  float _velocity;
  list<vec> _checkpoints;
  list<vec>::iterator _currentInitCheckpoint;
  float _currentInitTime;
  float _currentEndTime;
  vec _robotPosition;
  vec _collision;
  list<vec>::iterator _collisionInitCheckpoint;

};

#endif // TRAJECTORY_H

#include "trajectory.h"
#include "utils.h"
Trajectory::Trajectory(float velocity, vec start) :
  _velocity(velocity),
  _checkpoints()
{
  pushBackCheckpoint(start);
  _currentInitCheckpoint = _checkpoints.begin();
}

void Trajectory::pushBackCheckpoint(vec checkpoint)
{
  _checkpoints.push_back(checkpoint);
}

void Trajectory::addCheckpoint(vec checkpoint, vec collisionPoint)
{
  float currentDist;
  vec currentCheckpoint;
  for(list<vec>::iterator it = _currentInitCheckpoint; it != prev(_checkpoints.end(),1) ; it++)
    {
      list<vec>::iterator nextIt = next(it,1);
      if (it->at(0) < collisionPoint.at(0) && it->at(1) < collisionPoint.at(1)  &&
          nextIt->at(0) > collisionPoint.at(0) && nextIt->at(1) > collisionPoint.at(1))
        {
          _checkpoints.insert(nextIt,checkpoint);
        }
    }
  throw "CollisionPoint is not in the trajectory";
}

void Trajectory::getPositioVelocity(float time, vec &position, vec &velocity)
{

  if (time > _currentEndTime)
    {
      _currentInitCheckpoint++;
      _currentInitTime = _currentEndTime;
    }
  time -= _currentInitTime;
  vec start = *_currentInitCheckpoint;
  vec end = *next(_currentInitCheckpoint,1);
  float mod = stocazzo(start, end);
  float m = (end[1]-start[1])/(end[0]-start[0]);
  float theta_d = atan(m);
  //Traiettoria tra due punti
  position[0] = start[0] +  (_velocity * time)/ mod * (end[0]- start[0]);
  position[1] = start[1] +  (_velocity * time)/ mod * (end[1]- start[1]);

  //X and y velocity beetwen the two point
  velocity[0] = _velocity * cos(theta_d);
  velocity[1] = _velocity * sin(theta_d);

  if (time > _currentEndTime)
    _currentEndTime = _velocity * mod;
}

vec* Trajectory::isCollision(obstacle obs)
{

}

void Trajectory::makeForwardCheckpoint(float dist)
{

}


float Trajectory::dist(vec a,vec b){
  return sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2));
}


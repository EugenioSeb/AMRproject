#include "trajectory.h"
#include "utils.h"

bool intersection( vec p1seg1, vec p2seg1, vec p1seg2,  vec p2seg2, vec& intersect) {
    float d = (p1seg1[0]-p2seg1[0])*(p1seg2[1]-p2seg2[1]) - (p1seg1[1]-p2seg1[1])*(p1seg2[0]-p2seg2[0]);
    if (d == 0) return false;

    intersect[0] = ((p1seg2[0]-p2seg2[0])*(p1seg1[0]*p2seg1[1]-p1seg1[1]*p2seg1[0])-(p1seg1[0]-p2seg1[0])*(p1seg2[0]*p2seg2[1]-p1seg2[1]*p2seg2[0]))/d;
    intersect[1] = ((p1seg2[1]-p2seg2[1])*(p1seg1[0]*p2seg1[1]-p1seg1[1]*p2seg1[0])-(p1seg1[1]-p2seg1[1])*(p1seg2[0]*p2seg2[1]-p1seg2[1]*p2seg2[0]))/d;

    if (intersect[0] < min(p1seg1[0],p2seg1[0]) || intersect[0] > max(p1seg1[0],p2seg1[0]))
      return false;
    if (intersect[0] < min(p1seg2[0],p2seg2[0]) || intersect[0] > max(p1seg2[0],p2seg2[0]))
      return false;
    if (intersect[1] < min(p1seg1[1],p2seg1[1]) || intersect[1] > max(p1seg1[1],p2seg1[1]))
      return false;
    if (intersect[1] < min(p1seg2[1],p2seg2[1]) || intersect[0] > max(p1seg2[1],p2seg2[1]))
      return false;
    return true;
  }


Trajectory::Trajectory(float velocity, vec start, vec nextCheckpoint) :
  _velocity(velocity),
  _currentInitTime(0.0f),
  _checkpoints()
{
  pushBackCheckpoint(start);
  pushBackCheckpoint(nextCheckpoint);
  _currentEndTime = _velocity / module(start,nextCheckpoint);
  _currentInitCheckpoint = _checkpoints.begin();
}

void Trajectory::pushBackCheckpoint(vec checkpoint)
{
  _checkpoints.push_back(checkpoint);
}

void Trajectory::addCheckpoint(vec checkpoint, vec collisionPoint)
{

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
  list<vec>::iterator temp = next(_currentInitCheckpoint,1);
  vec end = *temp;
  float mod = module(start, end);
  float m = (end[1]-start[1])/(end[0]-start[0]);
  float theta_d = atan(m);
  //Traiettoria tra due punti
  position[0] = start[0] +  (_velocity * time) / mod * (end[0]- start[0]);
  position[1] = start[1] +  (_velocity * time) / mod * (end[1]- start[1]);
  _robotPosition = position;
  //X and y velocity beetwen the two point
  velocity[0] = _velocity * cos(theta_d);
  velocity[1] = _velocity * sin(theta_d);

  if (time > _currentEndTime)
    _currentEndTime = _velocity * mod;
}

//return true if there is a collision and do side effect on argument collision (a vec of 2 elements), if not return false
bool Trajectory::isCollision(obstacle obs, vec& collision)
{
  vector<vec> obsPoints = obs.getBoundingBox();
  for(list<vec>::iterator it = _currentInitCheckpoint; it != prev(_checkpoints.end(),1) ; it++)
    {
      list<vec>::iterator nextIt = next(it,1);
      if(intersection(obsPoints[0],obsPoints[1],*it, *nextIt,collision) ||
         intersection(obsPoints[1],obsPoints[2],*it, *nextIt,collision) ||
         intersection(obsPoints[2],obsPoints[3],*it, *nextIt,collision) ||
         intersection(obsPoints[3],obsPoints[0],*it, *nextIt,collision) )
        if(it != _currentInitCheckpoint || dist(*it,collision) > dist(*it,_robotPosition))
          return true;
    }
  return false;
}

void Trajectory::makeForwardCheckpoint(float distance)
{
  //Compute the theta_d (angle between the segment and the x axis)
  vec start = *_currentInitCheckpoint;
  vec end = *next(_currentInitCheckpoint,1);
  float m = (end[1]-start[1])/(end[0]-start[0]);
  float theta_d = atan(m);

  //Compute the new checkpoint
  vec newCheckpoint(2);
  newCheckpoint[0] =  _robotPosition[0] + distance * cos(theta_d);
  newCheckpoint[1] =  _robotPosition[1] + distance * sin(theta_d);

  //Add the new checkpoint
  _checkpoints.insert(next(_currentInitCheckpoint,1), newCheckpoint);

}


float Trajectory::dist(vec a,vec b){
  return sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2));
}




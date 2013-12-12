#include "obstacle.h"

#define B_BOX_DIST 0.5
#define VELOCITY 0.1
using namespace std ;

obstacle::obstacle(const char* name)
{
  //Connect VREP through remote API
  _clientId = simxStart("127.0.0.1",19699,true,true,2000,5);
  if(_clientId == -1){
      cout << "Error simxStart";
    }
  // Get the handle of the "trajectory ball"
  cout<<"Errore obstacle Handle:"<<simxGetObjectHandle(_clientId, name, &_obsHandle, simx_opmode_oneshot_wait)<<endl;
}

void obstacle::move(vec start, vec end)
{
  //Set the variable for the time
  //Init of other parameters
  double delta_t = 0;

  //The current position of the obstacle
  vec p_obs(2);

  //Set the obstacle to the initial position
  _position[0] = start[0];
  _position[1] = start[1];
  _position[2] = 0.25f;
  //Start the time
  RobTimer time;
  //The while loop for update the position of the obstacle
  while( dist(start, p_obs ) < _currentModule )
    {
      //calcola elapsedTime
      delta_t = time.getTime();
      obsTrajectory(p_obs, delta_t, start, end);
      _position[0] = p_obs[0];
      _position[1] = p_obs[1];
      simxSetObjectPosition(_clientId, _obsHandle, -1, _position, simx_opmode_oneshot_wait);
      sleep(0.1);
    }
  cout << "MoveThread ob1 position:" << _position[0] << " " << _position[1] << endl;
}

float simxFloatDist(simxFloat a[], simxFloat b[]){
  return sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2));
}


simxFloat obstacle::obsTrajectory(vec &p_obs, double &time, vec &start, vec &end)
{
  //Traiettoria tra due punti
  p_obs[0] = start[0] +  (VELOCITY * time)/ _currentModule * (end[0]- start[0]);
  p_obs[1] = start[1] +  (VELOCITY * time)/ _currentModule * (end[1]- start[1]);

}

vec obstacle::getDirection()
{
  return create2dvec(cos(_directionTheta),sin(_directionTheta));
}

void obstacle::startMove(vec start, vec end)
{
  float m = (end[1]-start[1])/(end[0]-start[0]);
  float _directionTheta = atan(m);
  _currentModule = module(start, end);
  _motion = thread(&obstacle::move, this, start, end);
}

vector<vec> obstacle::getBoundingBox()
{
  //Initizlize the vertex of the bounding box
  vec p1(2);
  vec p2(2);
  vec p3(2);
  vec p4(2);

  //Compute the vertex
  p1[0] = _position[0] - B_BOX_DIST;
  p1[1] = _position[1] + B_BOX_DIST;

  p2[0] = _position[0] + B_BOX_DIST;
  p2[1] = _position[1] + B_BOX_DIST;

  p3[0] = _position[0] + B_BOX_DIST;
  p3[1] = _position[1] - B_BOX_DIST;

  p4[0] = _position[0] - B_BOX_DIST;
  p4[1] = _position[1] - B_BOX_DIST;

  //Initialize the return vector<vec>
  vector<vec> box(4);
  box[0] = p1;
  box[1] = p2;
  box[2] = p3;
  box[3] = p4;

  return box;

}


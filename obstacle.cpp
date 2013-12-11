#include "obstacle.h"

#define D_B 0.5

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
  timeval start_t;
  timeval stop_t;

  //Set the initial and final point of the desidered trajectory of the obstacle
  start[0] = 1;
  start[1] = -1;
  end[0] = 1;
  end[1] = 1;

  //The current position of the obstacle
  vec p_obs(2);

  //Set the obstacle to the initial position
  simxFloat obsPosition[] = {start[0], start[1], 0.25f};
  simxSetObjectPosition(_clientId, _obsHandle, -1, obsPosition, simx_opmode_oneshot_wait);

  //Start the time
  gettimeofday(&start_t, NULL);

  //The while loop for update the position of the obstacle
  while(true)
    {
      //Stop the time interval
      gettimeofday(&stop_t, NULL);

      //calcola elapsedTime
      delta_t  = (stop_t.tv_sec - start_t.tv_sec) * 1000.0;               // sec to ms
      delta_t += (stop_t.tv_usec - start_t.tv_usec) / 1000.0;            // us to ms
      delta_t  = delta_t / 1000.0;
      obsTrajectory(p_obs, delta_t, start, end);
      cout << p_obs[1]<<endl;
      _position[0] = p_obs[0];
      _position[1] = p_obs[1];
      _position[2] = 0;
      simxSetObjectPosition(_clientId, _obsHandle, -1, _position, simx_opmode_oneshot_wait);
      sleep(0.1);
    }
}


simxFloat obstacle::obsTrajectory(vec &p_obs, double &time, vec &start, vec &end)
{

  float mod = module(start, end);
  const float velocity = 0.1;

  //Traiettoria tra due punti
  p_obs[0] = start[0] +  (velocity * time)/ mod * (end[0]- start[0]);
  p_obs[1] = start[1] +  (velocity * time)/ mod * (end[1]- start[1]);

}

void obstacle::startMove(vec start, vec end)
{
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
  p1[0] = _position[0] - D_B;
  p1[1] = _position[1] + D_B;

  p2[0] = _position[0] + D_B;
  p2[1] = _position[1] + D_B;

  p3[0] = _position[0] + D_B;
  p3[1] = _position[1] - D_B;

  p4[0] = _position[0] - D_B;
  p4[1] = _position[1] - D_B;

  //Initialize the return vector<vec>
  vector<vec> box(4);
  box[0] = p1;
  box[1] = p2;
  box[2] = p3;
  box[3] = p4;

  return box;

}


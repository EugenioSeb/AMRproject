#include "obstacle.h"

using namespace std ;

obstacle::obstacle(const char* name)
{
    //Connect VREP through remote API
    _clientId = simxStart("127.0.0.1",19699,true,true,2000,5);
    if(_clientId == -1){
    cout << "Error simxStart";
    // Get the handle of the "trajectory ball"
    cout<<"Errore ballHandle:"<<simxGetObjectHandle(_clientId, name, &_obsHandle, simx_opmode_oneshot_wait)<<endl;

      }
}


void obstacle::move(){

    //Set the variable for the time
    //Init of other parameters
    double delta_t = 0;
    timeval start_t;
    timeval stop_t;

    //Set the initial and final point of the desidered trajectory of the obstacle
    vector<float> start(2);
    vector<float> end(2);
    start[0] = 1;
    start[1] = -1;
    end[0] = 1;
    end[1] = 1;

    //The current position of the obstacle
    vector<float> p_obs(2);

    //Set the obstacle to the initial position
    simxFloat obsPosition[] = {start[0], start[1], 0};
    simxSetObjectPosition(_clientId, _obsHandle, -1, obsPosition, simx_opmode_oneshot_wait);

    //Start the time
    gettimeofday(&start_t, NULL);

    //The while loop for update the position of the obstacle
    while(true){

          //Stop the time interval
          gettimeofday(&stop_t, NULL);

          //calcola elapsedTime
          delta_t  = (stop_t.tv_sec - start_t.tv_sec) * 1000.0;               // sec to ms
          delta_t += (stop_t.tv_usec - start_t.tv_usec) / 1000.0;            // us to ms
          delta_t  = delta_t / 1000.0;
          obsTrajectory(p_obs, delta_t, start, end);
          simxFloat obsPosition[] = {p_obs[0], p_obs[1], 0};
          simxSetObjectPosition(_clientId, _obsHandle, -1, obsPosition, simx_opmode_oneshot_wait);


    }
}

float modul(vector<float> &init, vector<float> &end){
    int size= init.size();
    float mod = 0;
    for(int i=0; i<size; i++){
    mod += pow(end[i] - init[i], 2);
    }
    return mod;
}

simxFloat obstacle::obsTrajectory(vector<float> &p_obs, double &time, vector<float> &start, vector<float> &end){

    float mod = modul(start, end);
    float m = (end[1]-start[1])/(end[0]-start[0]);
    float theta_d = atan(m);
    const float velocity = 0.1;

    cout<<"Theta:"<<theta_d<<endl;

    //Traiettoria tra due punti
    p_obs[0] = start[0] +  (velocity * time)/ mod * (end[0]- start[0]);
    p_obs[1] = start[1] +  (velocity * time)/ mod * (end[1]- start[1]);

    //X and y velocity beetwen the two point
    //vel_des[0] = (end[0]- init[0]) / mod * LIN_V * cos(theta_d);
    //vel_des[1] = (end[1]- init[1]) / mod * LIN_V * sin(theta_d);
    //vel_des[0] = LIN_V * cos(theta_d);
    //vel_des[1] = LIN_V * sin(theta_d);
}

void obstacle::startMove(){
    _motion = new thread(&obstacle::move, this);
}



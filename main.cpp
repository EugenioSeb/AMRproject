
#include <iostream>
#include <alproxies/alnavigationproxy.h>
#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>
#include <alproxies/alrobotpostureproxy.h>
#include <qi/os.hpp>
#include <sys/time.h>
#include <ctime>
#include <vector>
#include <math.h>
#include <thread>
#include "obstacle.h"

extern "C" {
#include "remoteApi/extApi.h"
#include "remoteApi/v_repConst.h"
/*	#include "extApiCustom.h" if you wanna use custom remote API functions! */
}
//controller constants
#define K1 2.0
#define POINT_B 0.25

//trajectory constants
#define OMEGA_DES 2 * M_PI / 100
#define CENTER_X 0
#define CENTER_Y 1
#define RAY 1
#define LIN_V 0.1

using namespace std ;


//Compute the module of two vectors
float module(vector<float> &init, vector<float> &end){
    int size= init.size();
    float mod = 0;
    for(int i=0; i<size; i++){
    mod += pow(end[i] - init[i], 2);
    }
    return mod;
}

vector<float> inOutController(const vector<float> &x_y_d,  const vector<float> &robot_pos,  const vector<float> &v_x_y_d){

  float x_d = x_y_d[0];
  float y_d = x_y_d[1];

  float x = robot_pos[0];
  float y = robot_pos[1];
  float theta = robot_pos[2];

  float v_x_d = v_x_y_d[0];
  float v_y_d = v_x_y_d[1];

  float x_b = x + POINT_B* cos(theta);
  float y_b = y + POINT_B * sin(theta);

  float v_x_b = v_x_d + (K1 * (x_d - x_b));
  float v_y_b = v_y_d + (K1 * (y_d - y_b));
  //cout<<"v_x_b"<<v_x_b<<"v_y_b"<<v_y_b<<endl;

  float v = cos(theta) * v_x_b + sin(theta) * v_y_b;
  float omega = (cos(theta) * v_y_b - (sin(theta) * v_x_b )) / POINT_B;

  vector<float> res(2);
  res.at(0) = v;
  res.at(1) = omega;
  return res;
}



// TODO Documentation
void getTrajectory(vector<float> &pos_des, vector<float> &vel_des, double time){
  float theta_d = OMEGA_DES * time - (M_PI/2) + 0.15;
  pos_des[0] = CENTER_X + (RAY * cos(theta_d));
  pos_des[1] = CENTER_Y + (RAY * sin(theta_d));

  vel_des[0] = -RAY * sin(theta_d) * OMEGA_DES;
  vel_des[1] =  RAY * cos(theta_d) * OMEGA_DES;
}


//Get the straight trajectory between two points
void getTrajectoryPo2Po(vector<float> &pos_des, vector<float> &vel_des, double time, vector<float> &init, vector<float> &end){

    float mod = module(init, end);
    float m = (end[1]-init[1])/(end[0]-init[0]);
    float theta_d = atan(m);

    //Traiettoria tra due punti
    pos_des[0] = init[0] +  (LIN_V * time)/ mod * (end[0]- init[0]);
    pos_des[1] = init[1] +  (LIN_V * time)/ mod * (end[1]- init[1]);

    //X and y velocity beetwen the two point
    //vel_des[0] = (end[0]- init[0]) / mod * LIN_V * cos(theta_d);
    //vel_des[1] = (end[1]- init[1]) / mod * LIN_V * sin(theta_d);
    vel_des[0] = LIN_V * cos(theta_d);
    vel_des[1] = LIN_V * sin(theta_d);
}

void obsTrajectory(vector<float> &pos_des, vector<float> &vel_des, double time, vector<float> &ObInit, vector<float> &ObEnd){

}


int main() {
  //Connect VREP through remote API
  int remApiClientID = simxStart("127.0.0.1",19698,true,true,2000,5);
  if(remApiClientID == -1){
    cout << "Error simxStart";
  }

  // Get the handle of the "trajectory ball"
  simxInt ballHandle;
  cout<<"Errore ballHandle:"<<simxGetObjectHandle(remApiClientID, "Sphere", &ballHandle, simx_opmode_oneshot_wait)<<endl;

  //Initialize the position
  simxInt robotHandle;
  cout<<"Errore robotHandle:"<<simxGetObjectHandle(remApiClientID, "torso_respondable1cyl0", &robotHandle, simx_opmode_oneshot_wait)<<endl;

  //Initialize the position of the "trajectory ball"
  simxFloat ballPosition[] = {0, 0, 0.25};

  //Set the IP and the Port for comunication
  string IpNao = "127.0.0.1";
  int PortNao = 9559 ;

  //Create an AlMotionProxy object
  AL::ALMotionProxy motion(IpNao, PortNao);

  //Give a stiffness at the robot
  motion.stiffnessInterpolation("Body", 1, 1);
  sleep(1);
  motion.moveInit();

  //Init of other parameters
  double delta_t = 0;

cout<<"Porco"<<endl;
  timeval start;
  timeval stop;

  // x e y desiderati
  std::vector<float> x_y_d(2);

  // velocità su x e y desiderate
  std::vector<float> v_x_y_d(2);
  float v_x;
  float omega;
  float theta_d;
  simxFloat robot_pos[3]; //x y z vector
  simxFloat robot_orient[3]; // angles along x y z
  vector<float> robot_pos_orient(3); // x y and z angle

  //The initial and the final point for a straight trajectory
  vector<float> init(2);
  vector<float> end(2);
  //Set the two point
  init[0] = 0;
  init[1] = 0;
  end[0]  = -1;
  end[1]  = -1;

  //The initial and the final point of the obstacle trajectory
  vector<float> ObInit(2);
  vector<float> ObEnd(2);
  //Set the two point
  ObInit[0] = 0;
  ObInit[1] = 0;
  ObEnd[0]  = 2.5;
  ObEnd[1]  = 0;

  //Create the obstacle

  obstacle ob1("Cuboid");
  sleep(1);
  ob1.startMove();

  //wait the robot start mooving
  sleep(0.8);

  gettimeofday(&start, NULL);
  //------------------------------------------- main cycle ----------------------------------------//
  while(true){

      //motion.waitUntilWalkIsFinished( );
      gettimeofday(&stop, NULL);

      //calcola elapsedTime
      delta_t  = (stop.tv_sec - start.tv_sec) * 1000.0;               // sec to ms
      delta_t += (stop.tv_usec - start.tv_usec) / 1000.0;            // us to ms
      delta_t  = delta_t / 1000.0;

      //Get the trajectory desi
      //getTrajectory(x_y_d,v_x_y_d,delta_t);
      getTrajectoryPo2Po(x_y_d, v_x_y_d, delta_t, init, end);

      //std::vector<float> robot_pos = motion.getRobotPosition(false);
      simxGetObjectPosition(remApiClientID,   robotHandle, -1,robot_pos, simx_opmode_oneshot_wait);
      simxGetObjectOrientation(remApiClientID, robotHandle, -1, robot_orient, simx_opmode_oneshot_wait);
      robot_pos_orient[0] = robot_pos[0];
      robot_pos_orient[1] = robot_pos[1];
      robot_pos_orient[2] = robot_orient[2];

      vector<float> v_x_y = inOutController(x_y_d, robot_pos_orient, v_x_y_d);

      //update ball position
      ballPosition[0] = x_y_d[0];
      ballPosition[1] = x_y_d[1];
      simxSetObjectPosition(remApiClientID, ballHandle, -1, ballPosition, simx_opmode_oneshot_wait);
      v_x = 0.2 *v_x_y[0];
      omega = 0.2* v_x_y[1];

      cout <<"At time:" << delta_t << "\tv:" << v_x << "...w:" << omega << endl;
      motion.move(v_x * cos(omega), v_x * sin(omega), omega);
      sleep(0.4);
    }
  motion.stopMove();
  simxFinish(remApiClientID);

}


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

using namespace std ;
std::vector<float> inOutController(const std::vector<float> &x_y_d,  const std::vector<float> &robot_pos,  const std::vector<float> &v_x_y_d){

  float x_d = x_y_d[0];
  float y_d = x_y_d[1];

  float x = robot_pos[0];
  float y = robot_pos[1];
  float theta = robot_pos[2];

  float v_x_d = v_x_y_d[0];
  float v_y_d = v_x_y_d[1];

  float x_b = x + 2.0 * cos(theta);
  float y_b = y + POINT_B * sin(theta);

  float v_x_b = v_x_d + (K1 * (x_d - x_b));
  float v_y_b = v_y_d + (K1 * (y_d - y_b));
  //cout<<"v_x_b"<<v_x_b<<"v_y_b"<<v_y_b<<endl;

  float v = cos(theta) * v_x_b + sin(theta) * v_y_b;
  float omega = (cos(theta) * v_y_b - (sin(theta) * v_x_b )) / POINT_B;

  vector<float> res(2);
  res[0] = v;
  res[1] = omega;
  return res;
}



// TODO Documentation
void getTrajectory(vector<float> &position, vector<float> &velocity, double time){
  const float theta_d = OMEGA_DES * time - (M_PI/2) + 0.15;
  position[0] = CENTER_X + (RAY * cos(theta_d));
  position[1] = CENTER_Y + (RAY * sin(theta_d));

  velocity[0] = -RAY * sin(theta_d) * OMEGA_DES;
  velocity[1] =  RAY * cos(theta_d) * OMEGA_DES;

  //Traiettoria "otto"
  //x_c and y_c center of the "eight"
//  position[0] = CENTER_X + (RAY * sin(2 * OMEGA_DES));
//  position[1] = CENTER_Y + (RAY * sin(OMEGA_DES));

//  velocity[0] = RAY * cos(2 * theta_d)* 2 * OMEGA_DES;
//  velocity[1] =  RAY * cos(theta_d) * OMEGA_DES;
}


int main() {
  //Connect VREP through remote API
  int remApiClientID = simxStart("127.0.0.1",19698,true,true,2000,5);
  if(remApiClientID == -1){
      std::cout << "Error simxStart";
    }
  // Get the handle of the ball
  simxInt ballHandle;
  cout<<"ErroreObjectHandle:"<<simxGetObjectHandle(remApiClientID, "Sphere", &ballHandle, simx_opmode_oneshot_wait)<<endl;
  //Initialize the position
  simxInt robotHandle;
  cout<<"Errore Handle:"<<simxGetObjectHandle(remApiClientID, "torso_respondable1cyl0", &robotHandle, simx_opmode_oneshot_wait)<<endl;
  //Initialize the position
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

  //wait the robot start mooving
  sleep(0.8);

  gettimeofday(&start, NULL);
  //------------------------------------------- main cycle ----------------------------------------//
  while(true){

      //motion.waitUntilWalkIsFinished( );
      gettimeofday(&stop, NULL);
      //delta_t = delta_t + 0.5;
      //theta = omega_d * delta_t;
      //cout <<theta <<endl;

      //calcola elapsedTime
      delta_t  = (stop.tv_sec - start.tv_sec) * 1000.0;               // sec to ms
      delta_t += (stop.tv_usec - start.tv_usec) / 1000.0;            // us to ms
      delta_t  = delta_t / 1000.0;


      //x_c and y_c center of the circumference
      getTrajectory(x_y_d,v_x_y_d,delta_t);

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

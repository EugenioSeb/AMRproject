
#include <iostream>
#include <alproxies/alnavigationproxy.h>
#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>
#include <alproxies/alrobotpostureproxy.h>
#include <qi/os.hpp>
#include <time.h>
#include <vector>

std::vector<float> set_v(const vector<int> x_y_d, const vector<int> robot_pos, const vector<int> v_x_y_d, float theta){

   float x_d, y_d;
   float x, y;
   float v_x_d, v_y_d;
   float x_b, y_b;
   const float b;
   const float K1=2;
   const float K2=2;

   x_d = x_y_d.at(0);
   y_d = x_y_d.at(1);
   x = robot_pos.at(0);
   y = robot_pos.at(1);
   v_x_d = v_x_y_d.at(0);
   v_y_d = v_x_y_d.at(1);
   x_b = x + b * cos(theta);
   y_b = y + b * sin(theta);

   float v_x_b = v_x_d + K1 * (x_d - x_b);
   float v_y_b = v_y_d + K2 * (y_d - y_b);
   float v = cos(theta) * v_x_b + sin(theta) * v_y_b
   float omega = sin(theta) * v_x_b + cos(theta) * v_y_b;
   std::vector<float> res;
   res.push_back(v);
   res.push_back(omega);

   return res;

}

int main() {

  std::string IpNao = "127.0.0.1";
  int PortNao = 9559 ;

  //Create an AlMotionProxy object
  AL::ALMotionProxy motion(IpNao, PortNao);

  //Give a stiffness at the robot
  const AL::ALValue jointName = "Body";
  const AL::ALValue stiffness = 1.0;
  motion.setStiffnesses(jointName, stiffness);

  //Inizialize the variable for the computation
  time_t start_t;
  time_t real_t;


  const float R=1;
  float omega_d= 0.1;
  float v_x;
  float v_y ;
  float theta;


    std::string name = "CameraTop";
    int space = 1;
    bool useSensorValues = true;

    motion.moveInit();

    std::vector<float> x_y_d;
    std::vector<float> v_x_y_d;
    std::vector<float> robot_pos; = motion.getPosition(name, space, useSensorValues);
    x_y_d.push_back(R * cos(omega_d * real_t));
    x_y_d.push_back(R * sin(omega_d * real_t));
    v_x_y_d.push_back(-R * sin(omega_d * real_t) * omega_d);
    v_x_y_d.push_back(R * cos(omega_d * real_t) * omega_d);


    std::vector<float> v_x_y = set_v(x_y_d, robot_pos, v_x_y_d, theta);




    start_t = time(&start_t);

    while(true){
        //qi::os::msleep(100);
        motion.move(x, y, omega);

       // motion.moveToward(x,y,theta);
        std::vector<float> result = motion.getPosition(name, space, useSensorValues);
        std::cout <<"Current position:" <<result << std::endl;

       }






}

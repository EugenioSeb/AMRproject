
#include <iostream>
#include <alproxies/alnavigationproxy.h>
#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>
#include <alproxies/alrobotpostureproxy.h>
#include <qi/os.hpp>
#include <sys/time.h>
#include <ctime>
#include <vector>
#include<math.h>


   std::vector<float> set_v(const std::vector<float> &x_y_d,  const std::vector<float> &robot_pos,  const std::vector<float> &v_x_y_d, float theta){

   float x_d, y_d;
   float x, y;
   float v_x_d, v_y_d;
   float x_b, y_b;
   const float b = 0.5;
   const float K1 = 2;
   const float K2 = 2;

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
   float v = cos(theta) * v_x_b + sin(theta) * v_y_b;
   float omega = -sin(theta/b) * v_x_b + cos(theta/b) * v_y_b;
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

  const float R=1;
  float omega_d= 0.25;
  float omega;
  float v_x;
  float v_y ;
  float theta;
  timeval start, stop;
  double elapsedTime;




    motion.moveInit();

     // x e y desiderati
     std::vector<float> x_y_d;
     // velocità su x e y desiderate
     std::vector<float> v_x_y_d;
     //posizione del robot nel world frame
     std::vector<float> robot_pos = motion.getRobotPosition(false);

     x_y_d.push_back(R * cos(omega_d * elapsedTime));
     x_y_d.push_back(R * sin(omega_d * elapsedTime));
     v_x_y_d.push_back(-R * sin(omega_d * elapsedTime) * omega_d);
     v_x_y_d.push_back(R * cos(omega_d * elapsedTime) * omega_d);


     std::vector<float> v_x_y = set_v(x_y_d, robot_pos, v_x_y_d, theta);

     v_x = v_x_y.at(0) * cos(theta);
     v_y = v_x_y.at(0) * sin(theta);
     omega = v_x_y.at(1);



     gettimeofday(&start, NULL);

     while(true){
        //qi::os::msleep(100);

        motion.move(v_x, v_y, omega);

        gettimeofday(&stop, NULL);

        std::vector<float> robot_pos = motion.getRobotPosition(false);

        //calcola elapsedTime
        elapsedTime = (stop.tv_sec - start.tv_sec) * 1000.0;               // sec to ms
        elapsedTime += (stop.tv_usec - start.tv_usec) / 1000.0;            // us to ms



        x_y_d.at(0) = R * cos(omega_d * elapsedTime);
        x_y_d.at(1) = R * sin(omega_d * elapsedTime);
        v_x_y_d.at(0) = -R * sin(omega_d * elapsedTime) * omega_d;
        v_x_y_d.at(1)=R * cos(omega_d * elapsedTime) * omega_d;

        std::vector<float> v_x_y = set_v(x_y_d, robot_pos, v_x_y_d, theta);

        v_x = v_x_y.at(0) * cos(theta);
        v_y = v_x_y.at(0) * sin(theta);
        omega = v_x_y.at(1);



       }






}

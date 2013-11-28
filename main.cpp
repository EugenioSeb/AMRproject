
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

using namespace std ;

   std::vector<float> set_v(const std::vector<float> &x_y_d,  const std::vector<float> &robot_pos,  const std::vector<float> &v_x_y_d, float theta){

   float x_d;
   float y_d;

   float x;
   float y;

   float v_x_d;
   float v_y_d;

   float x_b;
   float y_b;

   const float b = 1.5;
   const float K1 = 0.5;
   const float K2 = 0.5;

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

      //cout<<"v_x_b"<<v_x_b<<"v_y_b"<<v_y_b<<endl;

   float v = cos(theta) * v_x_b + sin(theta) * v_y_b;
   float omega = (cos(theta) * v_y_b - sin(theta) * v_x_b )/b;

   vector<float> res;

   res.push_back(v);
   res.push_back(omega);

   return res;

}

int main() {



  //Set the IP and the Port for comunication
  string IpNao = "127.0.0.1";
  int PortNao = 9559 ;

  //Create an AlMotionProxy object
  AL::ALMotionProxy motion(IpNao, PortNao);
  cout<<"cazzo"<<endl;

  //Give a stiffness at the robot
  const AL::ALValue jointName = "Body";
  const AL::ALValue stiffness = 1.0f;

  motion.setStiffnesses(jointName, stiffness);
  motion.moveInit();

  //Inizialize the variable for the computation
  const float R = 4;
  float omega_d = 2 * M_PI / 30;
  float omega = 0;
  float v_x = 0;
  float v_y = 0;
  float theta;
  double delta_t = 0;

  double x_c = 0;
  double y_c = 0;

  timeval start;
  timeval stop;




  // x e y desiderati
  std::vector<float> x_y_d;

  // velocità su x e y desiderate
  std::vector<float> v_x_y_d;

  //posizione del robot nel world frame
  std::vector<float> robot_pos = motion.getRobotPosition(false);
  //std::cout << "Robot position is: " << robot_pos << std::endl;


     //x_c and y_c center of the circumference
     x_y_d.push_back(x_c + (R * cos(omega_d * delta_t)));
     x_y_d.push_back(y_c + (R * sin(omega_d * delta_t)));

     v_x_y_d.push_back(-R * sin(omega_d * delta_t) * omega_d);
     v_x_y_d.push_back(R * cos(omega_d * delta_t) * omega_d);


     theta = omega_d * delta_t;

     std::vector<float> v_x_y = set_v(x_y_d, robot_pos, v_x_y_d, theta);

     v_x = v_x_y.at(0) ;
     omega = v_x_y.at(1);



     gettimeofday(&start, NULL);

     bool i=true;
     while(i){


        motion.move(v_x, 0, omega);
       sleep(1);
        //motion.waitUntilWalkIsFinished();
        gettimeofday(&stop, NULL);
        //delta_t = delta_t + 0.5;
       // theta = omega_d * delta_t;
        //cout <<theta <<endl;




        //calcola elapsedTime
        delta_t  = (stop.tv_sec - start.tv_sec) * 1000.0;               // sec to ms
        delta_t += (stop.tv_usec - start.tv_usec) / 1000.0;            // us to ms
        delta_t  = delta_t / 1000.0;


        x_y_d.at(0) = x_c + R * cos(omega_d * delta_t);
        x_y_d.at(1) = y_c + R * sin(omega_d * delta_t);

        v_x_y_d.at(0) = -R * sin(omega_d * delta_t) * omega_d;
        v_x_y_d.at(1)=R * cos(omega_d * delta_t) * omega_d;

        std::vector<float> robot_pos = motion.getRobotPosition(false);
        //std::cout << "Robot position is: " << robot_pos << std::endl;
        vector<float> v_x_y = set_v(x_y_d, robot_pos, v_x_y_d, theta);

        v_x = v_x_y.at(0) ;
        omega = v_x_y.at(1);

        cout<<"v"<<v_x<<"...w"<<omega<<"...t"<<delta_t<<endl;


       }




}










































/*
/*
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <utility>
#include <cmath>

using namespace std ;


#include <alproxies/alnavigationproxy.h>
#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>
#include <alproxies/alrobotpostureproxy.h>
#include <qi/os.hpp>
#include "robot.hpp"

#include <sys/time.h>



class Move_t {
public :

  static AL::ALMotionProxy *Robot ;
  static double b ;
  static double K1 ;
  static double K2 ;


  double x ;
  double y ;
  double theta ;

  double y1d ;
  double y2d ;
  double y1d_p ;
  double y2d_p ;

  double y1 ;
  double y2 ;

  double u1 ;
  double u2 ;

  double y1_p ;
  double y2_p ;

  double v ;
  double w ;

  void Next(const vector< double >& rvd) {

    const vector< float > Pos(Robot->getRobotPosition(false)) ;

    x = (double)(Pos[0]) ;
    y = (double)(Pos[1]) ;
    theta = (double)(Pos[2]) ;

    y1d   = rvd[0] ;
    y2d   = rvd[1] ;
    y1d_p = rvd[2] ;
    y2d_p = rvd[3] ;

    y1 = x + b * cos(theta) ;
    y2 = y + b * sin(theta) ;

    u1 = y1d_p + K1 * (y1d - y1) ;
    u2 = y2d_p + K2 * (y2d - y2) ;

    y1_p = u1 ;
    y2_p = u2 ;

    v = cos(theta) * u1  + sin(theta) * u2 ;
    w = (-sin(theta) * u1 + cos(theta) * u2) / b ;

    Robot->move(y1_p,y2_p,w) ;
  }

  void Print() {
    cout << left
     << "x = "     << setw(10) << x
     << "y = "     << setw(10) << y
     << "theta = " << setw(10) << theta

     << "y1d = "   << setw(10) << y1d
     << "y2d = "   << setw(10) << y2d
     << "y1d_p = " << setw(10) << y1d_p
     << "y2d_p = " << setw(10) << y2d_p

     << "y1 = "    << setw(10) << y1
     << "y2 = "    << setw(10) << y2
     << "u1 = "    << setw(10) << u1
     << "u2 = "    << setw(10) << u2

     << "y1_p = "  << setw(10) << y1_p
     << "y2_p = "  << setw(10) << y2_p
     << "v = "     << setw(10) << v
     << "w = "     << setw(10) << w
     << endl ;
  }
} ;


double Move_t::b = 1.5 ;
double Move_t::K1 = 2 ;
double Move_t::K2 = 2 ;

AL::ALMotionProxy *Move_t::Robot = 0 ;


#include "robot.hpp"

typedef vector< vector< double > > Trajectory_t ;

Trajectory_t GetCircle(const double& xc,const double yc,
               const double& R,const double& wd,
               const double& dt,
               const double& tEnd) {
  Trajectory_t Trajectory ;
  Trajectory.clear() ;
  vector< double > rv ; rv.resize(4) ;
  for(double t = 0 ; t < tEnd ; t += dt) {
    rv[0] = xc + R * sin(wd * t) ;
    rv[1] = yc + R * cos(wd * t) ;
    rv[2] = + R * wd * cos(wd * t) ;
    rv[3] = - R * wd * sin(wd * t) ;
    Trajectory.push_back(rv) ;
  }
  return Trajectory ;
}


Trajectory_t RandomWalk(const double& dt,const double tEnd) {
  Trajectory_t Trajectory ;
  Trajectory.clear() ;
  srand48(time(0)) ;
  for(double t = 0 ; t < tEnd ; t += dt) {
    vector< double > rv ; rv.resize(4) ;
    rv[0] = 10*drand48() ;
    rv[1] = 10*drand48() ;
    rv[2] = 10*drand48() ;
    rv[3] = 10*drand48() ;
    Trajectory.push_back(rv) ;
  }
  return Trajectory ;
}

int main(int argc,char *argv[]) {

  std::string IpNao = "127.0.0.1";
  int PortNao = 9559 ;
  Robot_t Robot(IpNao,PortNao) ;

  //cout<<FRAME_ROBOT <<endl;

  //Create an AlMotionProxy object

  //Inizialize the variable for the computation
  double xc = 0 ;
  double yc = 0 ;
  double R = 3 ;
  double wd = 1.0 / 3.0 ;
  double dt = 0.5 ;
  double tEnd = 120 ;
  Trajectory_t Trajectory(GetCircle(xc,yc,R,wd,dt,tEnd)) ;

  //Trajectory_t Trajectory(RandomWalk(dt,tEnd)) ;
//  Move_t Move ;

  double delta_t = 0;

  sleep(3);
  while(true){
    for(Trajectory_t::const_iterator rvd = Trajectory.begin() ; rvd != Trajectory.end() ; rvd++) {
        Robot.Follow((*rvd)[0],(*rvd)[1] ,(*rvd)[2] ,(*rvd)[3], wd, delta_t)  ;
        delta_t += 0.5;
        //      Move.Print() ;
    }
  }
  return 0 ;
} ;




 */






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

using namespace std ;



int main(){

    //Connect VREP through remote API
    int remApiClientID = simxStart("127.0.0.1",19698,true,true,2000,5);
    if(remApiClientID == -1){
        std::cout << "Error simxStart";
      }
    // Get the handle of the ball
    simxInt handle;
    cout<<"ErroreObjectHandle:"<<simxGetObjectHandle(remApiClientID, "Sphere", &handle, simx_opmode_oneshot_wait)<<endl;
    //Initialize the position
    simxFloat position[] = {0, 0, 0.25};

    //Set the IP and the Port for comunication
    string IpNao = "127.0.0.1";
    int PortNao = 9559 ;

    //Create an AlMotionProxy object
    AL::ALMotionProxy motion(IpNao, PortNao);

    //Give a stiffness at the robot
    motion.stiffnessInterpolation("Body", 1, 1);
    sleep(1);
    motion.moveInit();

    // parametri della traiettoria
    float R = 1;
    float x_c = 0;
    float y_c = 1;

    // intervallo di tempo
    float delta_t = 0;

    // variabili per controllo tempo
    timeval start;
    timeval stop;

    // theta desiderato
    float theta_d;

    // omega desiderato (to set)
    const float omega_d = 2 * M_PI / 100;

    // velocità desiverata (to set)
    float v_d;
    //float v_d = 0.1;

    // parametri di controllo
    const float zeta = 0.5;
    const float a = 2;
    float K1 = 2 * zeta * a;
    float K2 = (pow(a, 2) - pow(omega_d, 2))/v_d;
    float K3 = 2 * zeta * a;

    // x e y desiderati
    vector<float> x_y_d(2);

    // v_x e v_y desiderati
    vector<float> v_x_y_d(2);

    // tracking error
    vector<float> e(3);

    // linear feedback
    vector<float> u(2);

    // the robot position
    vector<float> robot_pos;

    // imput velocities
    float v_x;
    float omega;

    // inizio intervallo di tempo
    gettimeofday(&start, NULL);

    while(true){
        // set theta desiderato
        theta_d = omega_d * delta_t - (M_PI/2) + 0.15;

        // set x e y desiderati
        x_y_d[0] = x_c + (R * cos(theta_d));
        x_y_d[1] = y_c + (R * sin(theta_d));


        // real NAO position
        robot_pos = motion.getRobotPosition(false);

        // set the error tracking (remember e1=e[0], e2=e[1], e3=e[2] )
        e[0] = cos(robot_pos[2]) * (x_y_d[0] - robot_pos[0]) + sin(robot_pos[2]) * (x_y_d[1] - robot_pos[1]);
        e[1] = -sin(robot_pos[2]) * (x_y_d[0] - robot_pos[0]) + cos(robot_pos[2]) * (x_y_d[1] - robot_pos[1]);
        e[2] = theta_d - robot_pos[2];

        //update k2
        K2 = (pow(a, 2) - pow(omega_d, 2))/fabs(v_d);

        // set the linear feedback (remember u1=u1[0], u2=u2[1])
        u[0] =-K1 * e[0];
        u[1] = -e[1] * K2 -K3 * e[2];

        //update ball position
        position[0] = x_y_d[0];
        position[1] = x_y_d[1];
        simxSetObjectPosition(remApiClientID, handle, -1, position, simx_opmode_oneshot_wait);

        // set v_d
        v_x_y_d[0] = -R * sin(theta_d) * omega_d;
        v_x_y_d[1] =  R * cos(theta_d) * omega_d;
        v_d = sqrt(pow(v_x_y_d[0], 2) + pow(v_x_y_d[1], 2));

        // set v_x and omega
        v_x = v_d * cos(e[2]) - u[0];
        omega = omega_d - u[1];

        cout<<"Linear velocity: "<<v_x<<" Angular velocity:"<<omega<<"Error e1:"<<e[0]<<endl;

        // give the imput to the NAO
        motion.move(v_x, 0, omega);

        // fine intervallo di tempo
        gettimeofday(&stop, NULL);

        // calcola intervallo di tempo
        delta_t  = (stop.tv_sec - start.tv_sec) * 1000.0;               // sec to ms
        delta_t += (stop.tv_usec - start.tv_usec) / 1000.0;            // us to ms
        delta_t  = delta_t / 1000.0;


    }
    motion.stopMove();
    simxFinish(remApiClientID);


}

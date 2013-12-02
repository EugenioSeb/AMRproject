
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include<math.h>


using namespace std ;

extern "C" {
  #include "remoteApi/extApi.h"
  #include "remoteApi/v_repConst.h"
/*	#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

int main(){
  int clientID = simxStart("127.0.0.1",19698,true,true,2000,5);
  if(clientID == -1){
      std::cout << "Error simxStart";
    }
//      cout << " connction id: " << simxGetConnectionId(clientID) << std::endl;
//      cout << " simxLoadScene return:  "
//            << simxLoadScene(clientID,"/home/eugenio/VREP_ROOT/scenes/naoConvexShapesMod21bis.ttt",0x000000,simx_opmode_oneshot_wait)
//            << endl;


  simxInt handle;
  cout<<"ErroreObjectHandle:"<<simxGetObjectHandle(clientID, "Sphere", &handle, simx_opmode_oneshot_wait)<<endl;

  // cout<<"ErrorStart:"<<simxStartSimulation(clientID, simx_opmode_oneshot_wait)<<endl;

   const float R = 1;
   const float x_c = 0;
   const float y_c = 1;
   double theta_d = 0;
   simxFloat position[] = {0,0,0.25};

   while(true){
      theta_d += 0.01 - (M_PI/2);
      position[0] = x_c + (R * cos(theta_d));
      position[1] = y_c + (R * sin(theta_d));
      simxSetObjectPosition(clientID, handle, -1, position, simx_opmode_oneshot_wait);
   }
   simxFinish(clientID);

}



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
  int clientID = simxStart("127.0.0.1",19998,true,true,2000,5);
  if(clientID == -1){
      std::cout << "errore";
    }
//      cout << " connction id: " << simxGetConnectionId(clientID) << std::endl;
//      cout << " simxLoadScene return:  "
//            << simxLoadScene(clientID,"/home/eugenio/VREP_ROOT/scenes/naoConvexShapesMod21bis.ttt",0x000000,simx_opmode_oneshot_wait)
//            << endl;


  simxInt handle;
  cout<<"ErroreObjectHandle:"<<simxGetObjectHandle(clientID, "Sphere", &handle, simx_opmode_oneshot_wait)<<endl;

  cout<<"ErrorStart:"<<simxStartSimulation(clientID, simx_opmode_oneshot_wait)<<endl;

   const float R = 2;
   const int x_c = 0;
   const int y_c = 1;
   double theta_d=0;
   simxFloat position[] = {R,0,0};

   while(true){
      theta_d += 0.01;
      position[0] = x_c + R * cos(theta_d);
      position[1] = y_c + R * sin(theta_d);
   cout<<"ErrorePosition:"<<simxSetObjectPosition(clientID, handle, -1, position, simx_opmode_oneshot_wait)<<endl;
   }


   simxFinish(clientID);

}


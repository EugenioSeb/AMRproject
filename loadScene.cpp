
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

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
  cout << " connction id: " << simxGetConnectionId(clientID) << std::endl;
  cout << " simxLoadScene return:  "
            << simxLoadScene(clientID,"/home/eugenio/VREP_ROOT/scenes/naoConvexShapesMod21bis.ttt",0x000000,simx_opmode_oneshot_wait)
            << endl;

  simxInt** objectH;
  simxInt* objectCount;
  simxInt  object = simxGetObjects(clientID, sim_object_shape_type,objectCount, objectH, simx_opmode_oneshot_wait );

  const simxChar* Obstacle;
  simxInt* handle;
  simxInt objectHandles = simxGetObjectHandle(clientID, Obstacle, handle, simx_opmode_oneshot_wait);




  //cout<<"Result: "<<object<<endl;
  simxInt start = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
  simxFinish(clientID);

}



#include <stdio.h>
#include <stdlib.h>
#include <iostream>

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
  std::cout << " connction id: " << simxGetConnectionId(clientID) << std::endl;
  std::cout << " simxLoadScene return;  "
            << simxLoadScene(clientID,"/opt/VREP_ROOT/scenes/naoConvexShapesMod21bis.ttt",0x000000,simx_opmode_oneshot_wait)
            << std::endl;
  simxFinish(clientID);
}


#include "utils.h"

float module(vec &init, vec &end){
    int size= init.size();
    float mod = 0;
    for(int i=0; i<size; i++){
    mod += pow(end[i] - init[i], 2);
    }
    return mod;
}

vec create2dvec(float x, float y)
{
  vec vector(2);
  //Set the two point
  vector[0] = x;
  vector[1] = y;
  return vector;
}

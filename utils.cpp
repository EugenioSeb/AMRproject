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

float dist(vec a, vec b){
  return sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2));
}

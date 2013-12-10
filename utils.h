#ifndef UTILS_H
#define UTILS_H

float module(vec &init, vec &end){
    int size= init.size();
    float mod = 0;
    for(int i=0; i<size; i++){
    mod += pow(end[i] - init[i], 2);
    }
    return mod;
}
#endif // UTILS_H

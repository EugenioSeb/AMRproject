#ifndef UTILS_HPP
#define UTILS_HPP
#include <vector>
#include "math.h"

typedef std::vector<float> vec;

float module(vec &init, vec &end);

float dist(vec a, vec b);

vec create2dvec(float x, float y);

#endif // UTILS_HPP

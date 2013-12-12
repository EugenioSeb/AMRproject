#include <iostream>
#include "utils.h"
#include <math.h>
using namespace std;

#define PI 3.14159
#define NEWLINE '\n'
vec create2dvec(float x, float y)
{
  vec vector(2);
  //Set the two point
  vector[0] = x;
  vector[1] = y;
  return vector;
}
bool intersection(vec p1seg1, vec p2seg1, vec p1seg2,  vec p2seg2, vec& intersect) {
    float d = (p1seg1[0]-p2seg1[0])*(p1seg2[1]-p2seg2[1]) - (p1seg1[1]-p2seg1[1])*(p1seg2[0]-p2seg2[0]);
    if (d == 0) return false;

    intersect[0] = ((p1seg2[0]-p2seg2[0])*(p1seg1[0]*p2seg1[1]-p1seg1[1]*p2seg1[0])-(p1seg1[0]-p2seg1[0])*(p1seg2[0]*p2seg2[1]-p1seg2[1]*p2seg2[0]))/d;
    intersect[1] = ((p1seg2[1]-p2seg2[1])*(p1seg1[0]*p2seg1[1]-p1seg1[1]*p2seg1[0])-(p1seg1[1]-p2seg1[1])*(p1seg2[0]*p2seg2[1]-p1seg2[1]*p2seg2[0]))/d;

    if (intersect[0] < min(p1seg1[0],p2seg1[0]) || intersect[0] > max(p1seg1[0],p2seg1[0]))
      return false;
    if (intersect[0] < min(p1seg2[0],p2seg2[0]) || intersect[0] > max(p1seg2[0],p2seg2[0]))
      return false;
    if (intersect[1] < min(p1seg1[1],p2seg1[1]) || intersect[1] > max(p1seg1[1],p2seg1[1]))
      return false;
    if (intersect[1] < min(p1seg2[1],p2seg2[1]) || intersect[1] > max(p1seg2[1],p2seg2[1]))
      return false;
    return true;
  }

int main()
{
  vec coll = create2dvec(0,0);
  bool isCol = intersection(create2dvec(0.25,-0.49),create2dvec(0.25,0.51),create2dvec(2,0),create2dvec(-1.5,0),coll);
  cout << isCol;
}

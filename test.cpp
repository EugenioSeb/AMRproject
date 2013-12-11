

#include <iostream>
#include "utils.h"
#include <math.h>
using namespace std;

#define PI 3.14159
#define NEWLINE '\n'

bool c(int& a, int& b){
  a=b;
  return true;
}

int main()
{
  int a=1;
  int b=0;
  if(false || c(a,b))
    cout << a;
}

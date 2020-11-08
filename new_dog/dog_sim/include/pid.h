 #ifndef _PID_H_ 
#define _PID_H_ 
#include<iostream>
using namespace std;
#include <stdio.h>
#include<cmath>
class pid
{
private:
    float p,d,max_output;
public:
    double tar_pos, tar_vel,cur_pos,cur_vel, output;
    pid();
    ~pid();
    pid(float _p,float _d, float _maxoutput);
    void cur_update(double _cur_pos, double _cur_vel);
    void calculate(double _tar_pos, double _tar_vel);
}; 

#endif 

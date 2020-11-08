#include "pid.h"
#define sign(a) ( (a) > 0 ? (1) : (-1) )
pid::pid(){}
pid::~pid(){}

pid::pid(float _p,float _d, float _maxoutput)
{
    p = _p;
    d = _d;
    max_output = _maxoutput;
}
void  pid::cur_update(double _cur_pos, double _cur_vel)
{
    cur_pos  = _cur_pos;
    cur_vel = _cur_vel;
}

void pid::calculate(double _tar_pos, double _tar_vel)
{
    double _out_com = p*(tar_pos - cur_pos) + d *(tar_vel - cur_vel);
    if(fabs(_out_com)>= max_output)
    {
        output = sign(_out_com) * max_output;
    }
    else
    {
        output = _out_com;
    }
    
}

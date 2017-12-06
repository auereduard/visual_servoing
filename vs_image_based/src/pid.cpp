/*
 * pid.cpp
 *
 *  Created on: 17.11.2017
 *      Author: edau
 */
#ifndef PID_CPP
#define PID_CPP

#include<pid.h>
#include<iostream>
using namespace std;

PID::PID(){
    Kp = 0;
    Ki = 0;
    Kd = 0;
    min = 0;
    max = 0;
    Ta = 0;
    ealt = 0;
    esum = 0;
}

PID::PID(double _Kp,double _Ki,double _Kd,double _min,double _max, double _Ta){

    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    min = _min;
    max = _max;
    Ta = _Ta;
    ealt = 0;
    esum = 0;

}

double PID::calc(const double &e){

    esum += e;
    double u;
    //Ansteuerwerte bestimmen
    u = Ki * Ta * esum + Kp * e + Kd * (e - ealt) / Ta;

    //Ansteuerwerte begrenzen
    if(u > max){u = max;}
    else if(u < min){u = min;}

    //Sicherung der Abweichung
    ealt = e;

    return u;
}

void PID::get_param(){
    cout<<Kp<<" "<<Ki<<" "<<Kd<<" "<<min<<" "<<max<<" "<<Ta<<endl;
}

#endif

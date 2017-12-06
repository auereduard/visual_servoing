#ifndef PID_H
#define PID_H

//#include <iostream>

class PID{

private:
        double Kp;
        double Ki;
        double Kd;
        double min;
        double max;
        double Ta;
        double ealt;
        double esum;

public:
        PID(double _Kp, double _Ki, double _Kd, double _min, double _max, double _Ta);
        PID();
        double calc(const double &e);
        void get_param();
};
#endif

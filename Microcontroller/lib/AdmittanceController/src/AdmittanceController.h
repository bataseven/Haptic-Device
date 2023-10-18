#ifndef AdmittanceController_H
#define AdmittanceController_H

#include "Arduino.h"


class AdmittanceController{
public:
AdmittanceController(double mass = 1, double damping = 1, double timeStep = 1, double K = 1);

double solve(double);

double setm(double);
double setb(double);
double setdt(double);
double setk(double);

double getm();
double getb();
double getdt();
double getk();

private:
double _mass;
double _damping;
double _dt;
double _K = 1;
double _vel = 0;
double _velPrev = 0;
};


#endif
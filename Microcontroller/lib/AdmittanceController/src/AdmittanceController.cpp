/********************************************************************
 
  - AdmittanceController.cpp
  - Implementation of admittance controller. 
  - Calculates a reference velocity for a given force input.
  - Equation to solve for the time domain:

    v(s)      K
    ---- = --------
    F(s)    ms + b

  - Created by Berke Ataseven, 2021.

*********************************************************************/

#include "AdmittanceController.h"

#include "Arduino.h"

AdmittanceController::AdmittanceController(double mass, double damping, double timeStep, double K) : _mass(mass), _damping(damping), _dt(timeStep), _K(K) {
  _mass = _mass > 0 ? _mass : 0.01;
  _damping = _damping > 0 ? _damping : 0.01;
  _dt = _dt > 0 ? _dt : 0.0001;
  _K = _K > 0 ? _K : 0.01;
}

// Calculate the output using euler integration
double AdmittanceController::solve(double forceInput) {
  _vel = (_velPrev + ((forceInput * _K - _damping * _velPrev) / _mass) * _dt);
  _velPrev = _vel;
  return _vel;
}

// Set a new time step for the solver
// Returns the old time step
double AdmittanceController::setm(double mass) {
  double tmp = _mass;
  _mass = mass > 0 ? mass : 0.01;
  return tmp;
}

// Set a new time step for the solver
// Returns the old time step
double AdmittanceController::setb(double damping) {
  double tmp = _damping;
  _damping = damping > 0 ? damping : 0.01;
  return tmp;
}

// Set a new time step for the solver
// Returns the old time step
double AdmittanceController::setdt(double dt) {
  double tmp = _dt;
  _dt = dt > 0 ? dt : 0.0001;
  return tmp;
}

// Set a new gain for the solver
// Returns the old gain
double AdmittanceController::setk(double K) {
  double tmp = _K;
  _K = K > 0 ? K : 0.01;
  return tmp;
}

// Returns the parameters of the controller.
double AdmittanceController::getm() { return _mass; }
double AdmittanceController::getb() { return _damping; }
double AdmittanceController::getdt() { return _dt; }
double AdmittanceController::getk() { return _K; }
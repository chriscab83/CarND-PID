#include "vehicle.h"

using namespace std;

Vehicle::Vehicle() 
  : speed(0.0), steering_angle(0.0), cte(0.0), throttle(0.0),
    timestep(0), set_speed(30.0), set_steering(0.0)
{ 
  steering_pid.Init(0.2, 0.0004, 2.0);
  throttle_pid.Init(0.3, 0.001, 2.0);
}

Vehicle::~Vehicle() {

}

void Vehicle::Update(double speed, double steering_angle, double cte) {
  this->speed = speed;
  this->steering_angle = steering_angle;
  this->cte = cte;

  ++timestep;

  UpdateSteering();
  UpdateThrottle();
}

void Vehicle::UpdateSteering() {
  steering_pid.UpdateError(cte);

  double steer_value = steering_pid.TotalError();
  if (steer_value > 1) {
    steer_value = 1;
  }
  else if (steer_value < -1) {
    steer_value = -1;
  }

  this->set_steering = steer_value;
}

void Vehicle::UpdateThrottle() {
  throttle_pid.UpdateError(speed - set_speed);

  double throttle_value = throttle_pid.TotalError();
  if (throttle_value > 1) {
    throttle_value = 1;
  }
  else if (throttle_value < -1) {
    throttle_value = -1;
  }

  this->throttle = throttle_value;
}
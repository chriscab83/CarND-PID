#ifndef VEHICLE_H
#define VEHICLE_H

#include "PID.h"

class Vehicle {
public:
  /*
  * Vehicle state
  */
  double speed;
  double steering_angle;
  double cte;
  double throttle;
  int timestep;
  
  double set_speed;
  double set_steering;

  /*
  * PID definitions
  */
  PID steering_pid;
  PID throttle_pid;

  /*
  * Constructor
  */
  Vehicle();

  /*
  * Destructor
  */
  virtual ~Vehicle();

  /*
  * Update vehicle state.
  */
  void Update(double speed, double steering_angle, double cte);

  void SetSpeed(double speed);
  
  /*
  * Update steering angle with PID
  */
  void UpdateSteering();

  /*
  * Update throttle setting with PID
  */
  void UpdateThrottle();

  // reset vehicle for next run
  void reset();
};

#endif  // VEHICLE_H
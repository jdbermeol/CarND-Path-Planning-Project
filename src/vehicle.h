#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
public:
  double id;
  double s;
  double d;
  double x;
  double y;
  double v;
  double yaw;
  int lane;

  /**
  * Constructor
  */
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();

};

#endif
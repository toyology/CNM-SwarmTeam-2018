#ifndef LOCATIONCONTROLLER_H
#define LOCATIONCONTROLLER_H

#include "Controller.h"
#include "LogicController.h"

class LocationController : virtual Controller
{
public:
  LocationController();
  ~LocationController();



  void setCurrentLocation(Point currentLocation);

  bool CNMCurrentLocationAVG();

  //Resets internal state to defaults
  void Reset(){};

  //Determines what action should be taken based on current
  //internal state and data
  Result DoWork(){};

  //Returns whether or not an interrupt must be thrown
  bool ShouldInterrupt(){};

  //Returns whether or not a controller should be polled for a Result
  bool HasWork(){};

private:

  // Numeric Variables for rover positioning
  Point currentLocation;
  Point currentLocationMap;
  Point currentLocationAverage;

  Point centerLocation;
  Point centerLocationMap;
  Point centerLocationOdom;

//Create Logic controller
  //LogicController logicController;

protected:

  //Looks at external data and determines if an interrupt must be thrown
  //or if the controller should be polled
  void ProcessData(){};
};

#endif // LOCATIONCONTROLLER_H

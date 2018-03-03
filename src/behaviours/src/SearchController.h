#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <geometry_msgs/Pose2D.h> //CNM added 2/15/18
#include <random_numbers/random_numbers.h>
#include "Controller.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();

  void cnmSetCenterLocation(Point cnmCenterLocation);
  void cnmSetAvgCurrentLocation(Point cnmAVGCurrentLocation);

  //sets the value for initial point in search pattern
  int SquareSearchStartPosition();

protected:

  void ProcessData();

private:

  //Averaged GPS center location
  Point cnmCenterLocation;

  Point cnmCurrentLocation;

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;
};

#endif /* SEARCH_CONTROLLER */

#include "SearchController.h"
#include <angles/angles.h>

int cnmSearchLoop = 0;

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
  result.reset = false;
}

/*
 * This code implements a basic square walk search for 12.8.17 checkin.
 */
Result SearchController::DoWork() {

  // Print info everytime the search loop is used
cout << "SEARCH - SearchController is doing work"  << endl;

//if for some reason searchLoop goes out of bounds, reset
//---------------------------------------------
if (cnmSearchLoop < 0 || cnmSearchLoop > 3) {
  cnmSearchLoop = 0;
  cout << "SEARCH - search loop out of bounds, reset"  << endl;
 }

   //clear intitial waypoints if any
   if (!result.wpts.waypoints.empty()) {
      if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.10) {
        //attemptCount = 0;
      }
    }
    result.type = waypoint;
    Point  searchLocation;

    //find which corner of the square to goto first based on 180deg of current heading
    if (first_waypoint)
    {
      cout << "SEARCH - finding where to go first"  << endl;
      first_waypoint = false;
      cnmSearchLoop = SearchController::SearchStartPosition();
    }
    else if (cnmSearchLoop == 0) //corner in quaderant 1
    {
      cout << "SEARCH - going to first corner of square"  << endl;
      searchLocation.theta = atan2((searchLocation.y - currentLocation.y), (searchLocation.x - currentLocation.x));
      searchLocation.x = centerLocation.x + 2.5;
      searchLocation.y = centerLocation.y + 2.5;
      cnmSearchLoop = 1;
    }
    else if (cnmSearchLoop == 1) //corner in quaderant 2
    {
      cout << "SEARCH - going to second corner of square"  << endl;
      searchLocation.theta = atan2((searchLocation.y - currentLocation.y), (searchLocation.x - currentLocation.x));
      searchLocation.x = centerLocation.x - 2.5;
      searchLocation.y = centerLocation.y + 2.5;
      cnmSearchLoop = 2;

    }
    else if (cnmSearchLoop == 2) //corner in quaderant 3
    {
      cout << "SEARCH - going to third corner of square"  << endl;
      searchLocation.theta = atan2((searchLocation.y - currentLocation.y), (searchLocation.x - currentLocation.x));
      searchLocation.x = centerLocation.x - 2.5;
      searchLocation.y = centerLocation.y - 2.5;
      cnmSearchLoop = 3;

    }
    else if (cnmSearchLoop == 3) //corner in quaderant 4
    {
      cout << "SEARCH - going to forth corner of square"  << endl;
      searchLocation.theta = atan2((searchLocation.y - currentLocation.y), (searchLocation.x - currentLocation.x));
      searchLocation.x = centerLocation.x + 2.5;
      searchLocation.y = centerLocation.y - 2.5;
      cnmSearchLoop = 0;

    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;

 }


/*
// This code implements the UNM basic random walk search.

Result SearchController::DoWork() {

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
      attemptCount = 0;
    }
  }

  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0)
  {
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

    //select new position 50 cm from current location
    if (first_waypoint)
    {
      first_waypoint = false;
      searchLocation.theta = currentLocation.theta + M_PI;
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }
    else
    {
      //select new heading from Gaussian distribution around current heading
      searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
  }

}

*/

void SearchController::SetCenterLocation(Point centerLocation) {

  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;

  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }

}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}

//decides which quaderant to send the rover based on it current heading + 180deg
int SearchController::SearchStartPosition()
{
    int searchLoop =0;

    if (currentLocation.theta + M_PI <= angles::from_degrees(80))
    {
        searchLoop = 0;
    }
    else if (currentLocation.theta  + M_PI  <= angles::from_degrees(170))
    {
        searchLoop = 1;
    }
    else if (currentLocation.theta  + M_PI  <= angles::from_degrees(260))
    {
        searchLoop = 2;
    }
    else if (currentLocation.theta  + M_PI <= angles::from_degrees(350))
    {
        searchLoop = 3;
    }

    return searchLoop;
  }

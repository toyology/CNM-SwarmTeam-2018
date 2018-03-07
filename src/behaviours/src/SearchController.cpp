#include "SearchController.h"
#include <angles/angles.h>

int cnmSquareSearchLoop = 0;

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();

  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  cnmCurrentLocation.x = 0;
  cnmCurrentLocation.y = 0;
  cnmCurrentLocation.theta = 0;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;

  delete rng; //managing memory leak from 2017 #neveragain

}

void SearchController::Reset() {
  result.reset = false;



}



/*
 * This code implements a basic square walk search for 12.8.17 checkin.
 */

Result SearchController::DoWork() {

  // Print info everytime the search loop is used
cout << "SEARCH - SquareSearchStartPosition is doing work"  << endl;

//Setup get cnm current location avgerage
static bool averaged = false;
averaged = SearchController::CNMCurrentLocationAVG();

//if for some reason searchLoop goes out of bounds, reset
//---------------------------------------------
if (cnmSquareSearchLoop < 0 || cnmSquareSearchLoop > 3) {
  cnmSquareSearchLoop = 0;
  cout << "SEARCH - search loop out of bounds, reset"  << endl;
 }

 if(averaged)
 {
   averaged = false;
   cout << "AVERAGED - Averaged Current location complete" << endl;
 }

   //clear intitial waypoints if any
   if (!result.wpts.waypoints.empty()) {
      if (hypot(result.wpts.waypoints[0].x-cnmCurrentLocation.x, result.wpts.waypoints[0].y-cnmCurrentLocation.y) < 0.10) {
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
      cnmSquareSearchLoop = SearchController::SquareSearchStartPosition();
    }
    else if (cnmSquareSearchLoop == 0) //corner in quadrant 1
    {
      cout << "SEARCH - going to first corner of square"  << endl;
      searchLocation.theta = atan2((searchLocation.y - cnmCurrentLocation.y), (searchLocation.x - cnmCurrentLocation.x));
      searchLocation.x = cnmCenterLocation.x + 2.5;
      searchLocation.y = cnmCenterLocation.y + 2.5;
      cnmSquareSearchLoop = 1;
    }
    else if (cnmSquareSearchLoop == 1) //corner in quadrant 2
    {
      cout << "SEARCH - going to second corner of square"  << endl;
      searchLocation.theta = atan2((searchLocation.y - cnmCurrentLocation.y), (searchLocation.x - cnmCurrentLocation.x));
      searchLocation.x = cnmCenterLocation.x - 2.5;
      searchLocation.y = cnmCenterLocation.y + 2.5;
      cnmSquareSearchLoop = 2;
    }
    else if (cnmSquareSearchLoop == 2) //corner in quadrant 3
    {
      cout << "SEARCH - going to third corner of square"  << endl;
      searchLocation.theta = atan2((searchLocation.y - cnmCurrentLocation.y), (searchLocation.x - cnmCurrentLocation.x));
      searchLocation.x = cnmCenterLocation.x - 2.5;
      searchLocation.y = cnmCenterLocation.y - 2.5;
      cnmSquareSearchLoop = 3;

    }
    else if (cnmSquareSearchLoop == 3) //corner in quadrant 4
    {
      cout << "SEARCH - going to forth corner of square"  << endl;
      searchLocation.theta = atan2((searchLocation.y - cnmCurrentLocation.y), (searchLocation.x - cnmCurrentLocation.x));
      searchLocation.x = cnmCenterLocation.x + 2.5;
      searchLocation.y = cnmCenterLocation.y - 2.5;
      cnmSquareSearchLoop = 0;
    }


    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;

 }



// This code implements the UNM basic random walk search.
/*
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


void SearchController::cnmSetCenterLocation(Point newLocation)
{
    cnmCenterLocation = newLocation;

}

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

void SearchController::cnmSetAvgCurrentLocation(Point cnmAVGCurrentLocation)
{
  cnmCurrentLocation = cnmAVGCurrentLocation;
}

//decides which quaderant to send the rover based on it current heading + 180deg
int SearchController::SquareSearchStartPosition()
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


  bool SearchController::CNMCurrentLocationAVG()
  {

    const int CASIZE = 30;

    float avgCurrentCoordsX[CASIZE];
    float avgCurrentCoordsY[CASIZE];

      static int index = 0;

      if(index < CASIZE)
      {

  	     avgCurrentCoordsX[index] = currentLocation.x;
      	 avgCurrentCoordsY[index] = currentLocation.y;

  	index++;

  	return false;
      }
      else
      {
  	float x = 0, y = 0;
  	for(int i = 0; i < CASIZE; i++)
  	{
  	    x += avgCurrentCoordsX[i];
  	    y += avgCurrentCoordsY[i];
  	}

  	x = x/CASIZE;
  	y = y/CASIZE;

    Point cnmAVGCurrentLocation;
  	cnmAVGCurrentLocation.x = x;
  	cnmAVGCurrentLocation.y = y;
    cnmAVGCurrentLocation.theta = currentLocation.theta;

    SearchController::cnmSetAvgCurrentLocation(cnmAVGCurrentLocation);

  	index = 0;
  	return true;
      }
  }

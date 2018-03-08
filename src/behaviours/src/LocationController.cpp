#include "LocationController.h"
#include "LogicController.h"


LocationController::LocationController()
{

}

LocationController::~LocationController(){ /*Destructor*/  }

bool LocationController::CNMCurrentLocationAVG()
{

  const int CASIZE = 30;

  float avgCurrentCoordsX[CASIZE];
  float avgCurrentCoordsY[CASIZE];

    static int index = 0;

    if(index < CASIZE)
    {

	     avgCurrentCoordsX[index] = currentLocationMap.x;
    	 avgCurrentCoordsY[index] = currentLocationMap.y;

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

  //logicController.cnmSetAvgCurrentLocation(cnmAVGCurrentLocation);

  //staticTest();

	index = 0;
	return true;
    }
}

void LocationController::setCurrentLocation(Point current) {
  currentLocationMap = current;
}

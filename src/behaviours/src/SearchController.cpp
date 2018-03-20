//3-3-18 Clean Search Controller
//3-7-18 Averaged Localization

#include "SearchController.h"
#include <angles/angles.h>

int cnmSearchLoop = 0;
bool cnmObstacleAvoided = false;

SearchController::SearchController() {
    rng = new random_numbers::RandomNumberGenerator();
    searchCounter = .5;
    //searchCounter = 4;
    searchDist = .35;
    //searchDist = 2;
    currentLocation.x = 0;
    currentLocation.y = 0;
    currentLocation.theta = 0;
    
    centerLocation.x = 0;
    centerLocation.y = 0;
    centerLocation.theta = 0;
    
    //Added 3-7-2018, cnmCurrentLocation for averaging
    cnmCurrentLocation.x = 0;
    cnmCurrentLocation.y = 0;
    cnmCurrentLocation.theta = 0;
    result.PIDMode = FAST_PID;
    
    result.fingerAngle = M_PI/2;
    result.wristAngle = M_PI/4;
    
    //Added 3-5-2018
    searchState = OCTAGON;
    
    //Added 3-6-2018
    sectorRadius = 2.5;
    
    //Added 3-6-2018
    squareHeight = 5;
    
    //Added 3-7-2018
    delete rng; //??
    
    //Added 3-10-2018 For obstacle handling
    //cnmObstacleAvoided = false;
    obstacleAvoidanceCount = 0;
    totalObstacleAvoidanceCount = 0;
    hasIncremented = true;
    //cnmObstacleAvoided = false;
    
    //Added 3-10-2018
    searchStep = 0;
}

void SearchController::Reset() 
{
    result.reset = false;
}
Result SearchController::DoWork() 
{
    
    float doneOneFullRotation = false;
    
    // Print info everytime the search loop is used
    cout << "SEARCH SquareSearchStartPositionler is doing work"  << endl;
    
    //Added 3-7-2018
    //Setup get cnm current location avgerage
    static bool averaged = false;
    averaged = SearchController::CNMCurrentLocationAVG();
    
    if(averaged)
    {
        averaged = false;
        cout << "AVERAGED - Averaged Current location complete" << endl;
    }
    
    //clear intitial waypoints if any
    if (!result.wpts.waypoints.empty()) 
    {                                      //changed from CL to cnmCL 3-7-2018
        if (hypot(result.wpts.waypoints[0].x-cnmCurrentLocation.x, 
                  result.wpts.waypoints[0].y-cnmCurrentLocation.y) < 0.10) 
        {
            //attemptCount = 0;
        }
    }
    result.type = waypoint;
    Point searchLocation;
    
    //Added 3-10-2018 for obstacle handling
    if(!cnmObstacleAvoided)
    {
        cnmSearchLoop++;
        searchStep++;
    }
    else
    {
        //right now, we always want our gather swarmies to 
        //stick with the octagon pattern
        //all other search patterns should switch to random, if 
        //they encounter enough obstacles
        if(!searchState == OCTAGON){
            if(++obstacleAvoidanceCount > 3)
            {
                cnmSearchLoop++;
                searchStep++;
                obstacleAvoidanceCount = 0;
                if(++totalObstacleAvoidanceCount > 10)
                    searchState = RANDOM;
            }
        }

    }
    
    
    
    //find which corner of the square to go 
    //to first based on 180deg of current heading
    if (first_waypoint)
    {
        cout << "SEARCH - finding where to go first"  << endl;
        first_waypoint = false;
        switch(searchState)
        {
            case SQUARE: 
            {
                cnmSearchLoop 
                = SearchController::SquareSearchStartPosition();
                break;
            }
            case OCTAGON:
            {
                cnmSearchLoop 
                = SearchController::OctagonSearchStartPosition();
                break;
            }
            case STAR:
            {
                cnmSearchLoop 
                = SearchController::StarSearchStartPosition();
                break;
            }
            case SECTOR:
            {
                cnmSearchLoop 
                = SearchController::SectorSearchStartPosition();
                break;
            }
            case RANDOM:
            {
                searchLocation.theta = currentLocation.theta + M_PI;
                searchLocation.x = currentLocation.x 
                + (0.5 * cos(searchLocation.theta));
                searchLocation.y = currentLocation.y 
                + (0.5 * sin(searchLocation.theta)); 
            }
        }
    }
    
    switch(searchState)
    {
        case SQUARE:
        {
            if (cnmSearchLoop < 0 || cnmSearchLoop > 3) 
            {
                cnmSearchLoop = 0;
                cout << "SEARCH - search loop out of bounds, reset"  << endl;
            }
            if (cnmSearchLoop == 0) //corner in quaderant 1
            {
                cout << "SEARCH - going to first corner of square"  << endl;
                searchLocation = SetDestination(cnmCurrentLocation,
                                                currentLocation, 
                                                squareHeight/2,
                                                squareHeight/2);
                //cnmSearchLoop = 1;
            }
            else if (cnmSearchLoop == 1) //corner in quaderant 2
            {
                cout << "SEARCH - going to second corner of square"  << endl;
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -squareHeight/2,
                                                squareHeight/2);
                //cnmSearchLoop = 2;
                
            }
            else if (cnmSearchLoop == 2) //corner in quaderant 3
            {
                cout << "SEARCH - going to third corner of square"  << endl;
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -squareHeight/2,
                                                -squareHeight/2);
                //cnmSearchLoop = 3;
                
            }
            else if (cnmSearchLoop == 3) //corner in quaderant 4
            {
                cout << "SEARCH - going to forth corner of square"  << endl;
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                +squareHeight/2,
                                                -squareHeight/2);
                //cnmSearchLoop = 0;
                
            }
            
            if (searchStep > 2)
            {
                squareHeight+=searchDist;
                totalObstacleAvoidanceCount = 0;
                searchStep = 0;
            }  
            break;  
        }//END SQUARE CASE
        
        case OCTAGON: //Counter Clockwise Octagon
        {
            if (cnmSearchLoop < 0 || cnmSearchLoop > 8) 
            {
                cnmSearchLoop = 0;
                cout << "SEARCH - search loop out of bounds, reset"  << endl;
            }
            if (cnmSearchLoop == 0)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                searchCounter/2,
                                                searchCounter);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 1)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -searchCounter/2,
                                                searchCounter);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 2)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -searchCounter,
                                                searchCounter/2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 3)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -searchCounter,
                                                -searchCounter/2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 4)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -searchCounter/2,
                                                -searchCounter);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 5)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                searchCounter/2,
                                                -searchCounter);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 6)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                searchCounter,
                                                -searchCounter/2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 7)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                searchCounter,
                                                searchCounter/2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 8)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                searchCounter/2,
                                                searchCounter);
                //cnmSearchLoop++;
                //searchCounter += searchDist;
            }
            if (searchStep > 7)
            {
                searchCounter+=searchDist;
                totalObstacleAvoidanceCount = 0;
                searchStep = 0;
            } 
            break;
        }//END OCTAGON CASE
        
        case STAR: //6 point star consisting of two triangles
        {
            if (cnmSearchLoop < 0 || cnmSearchLoop > 7)
            {
                cnmSearchLoop = 0;
                cout << "SEARCH - search loop out of bounds, reset"  << endl;
            }
            if (cnmSearchLoop == 0)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                0,
                                                searchCounter);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 1)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                searchCounter,
                                                -searchCounter/2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 2)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -searchCounter,
                                                -searchCounter/2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 3)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                0,
                                                searchCounter);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 4)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                searchCounter,
                                                searchCounter/2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 5)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                0,
                                                -searchCounter);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 6)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -searchCounter,
                                                searchCounter/2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 7)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                searchCounter,
                                                searchCounter/2);
                //cnmSearchLoop++;
                //searchCounter += searchDist;
            }
            if (searchStep > 6)
            {
                searchCounter+=searchDist;
                totalObstacleAvoidanceCount = 0;
                searchStep = 0;
            } 
            break;
        }//END STAR CASE
        
        case SECTOR:
        {
            if (cnmSearchLoop < 0 || cnmSearchLoop > 5)
            {
                cnmSearchLoop = 0;
                cout << "SEARCH - search loop out of bounds, reset"  << endl;
            }
            if (cnmSearchLoop == 0)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                sectorRadius,
                                                sectorRadius*2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 1)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -sectorRadius,
                                                sectorRadius*2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 2)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                sectorRadius,
                                                -sectorRadius*2);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 3)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                sectorRadius*2,
                                                0);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 4)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -sectorRadius*2,
                                                0);
                //cnmSearchLoop++;
            }
            else if (cnmSearchLoop == 5)
            {
                searchLocation = SetDestination(centerLocation,
                                                cnmCurrentLocation, 
                                                -sectorRadius,
                                                -sectorRadius*2);
                //cnmSearchLoop++;
            }
            break;
        }//END SECTOR CASE
        
        case RANDOM:
        {
            searchLocation.theta //45 degrees in radians
            = rng->gaussian(currentLocation.theta, 0.785398); 
            searchLocation.x 
            = currentLocation.x + (0.5 * cos(searchLocation.theta));
            searchLocation.y 
            = currentLocation.y + (0.5 * sin(searchLocation.theta));
        }//END RANDOM CASE
    }//END SWITCH 
    
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
    
}

Point SearchController::SetDestination(Point centerLocation,
                                       Point currentLocation, 
                                       double xDelta,
                                       double yDelta)
{
    searchLocation.x = centerLocation.x + xDelta;
    searchLocation.y = centerLocation.y + yDelta;
    searchLocation.theta = atan2((searchLocation.y - currentLocation.y),
                                 (searchLocation.x - currentLocation.x));
                                 
    //Added 3-10-2018 Succseful setting of new wayppoint
    //means obstacle can be reset.
    cnmObstacleAvoided = false;  
    return searchLocation;  
}

//Added 3-10-18 For Obstacle Avoidance Tracking
void SearchController::cnmSetObstacleAvoidanceState()
{
    cnmObstacleAvoided = true;
}

//Added 3-7-2018
void SearchController::cnmSetCenterLocation(Point newLocation)
{
    cnmCenterLocation = newLocation;
}

void SearchController::SetCenterLocation(Point centerLocation) 
{
    
    float diffX = this->cnmCenterLocation.x - centerLocation.x;
    float diffY = this->cnmCenterLocation.y - centerLocation.y;
    this->cnmCenterLocation = centerLocation;
    
    if (!result.wpts.waypoints.empty())
    {
        result.wpts.waypoints.back().x -= diffX;
        result.wpts.waypoints.back().y -= diffY;
    }
    
}

void SearchController::SetCurrentLocation(Point currentLocation) 
{
    this->currentLocation = currentLocation;
}

//Added 3-6-2018
void SearchController::SetSearchState(SearchState searchState)
{
    this->searchState = searchState;
}

//Added 3-6-2018
void SearchController::SetSectorRadius(double sectorRadius)
{
    this->sectorRadius = sectorRadius;
}

//Added 3-6-2018
void SearchController::SetSquareHeight(double squareHeight)
{
    this->squareHeight = squareHeight;
}

//Added 3-6-2018
void SearchController::SetSearchCounter(double searchCounter)
{
    this->searchCounter = searchCounter;
}

void SearchController::ProcessData() 
{
    //AJH do something here????
}

bool SearchController::ShouldInterrupt()
{
    ProcessData();
    
    return false;
}

bool SearchController::HasWork() 
{
    return true;
}

void SearchController::SetSuccesfullPickup() 
{
    succesfullPickup = true;
}

//decides which quaderant to send the rover based on it current heading + 180deg
int SearchController::SquareSearchStartPosition()
{
    int searchLoop =0;
    
    if (cnmCurrentLocation.theta + M_PI <= angles::from_degrees(80))
    {
        searchLoop = 0;
    }
    else if (cnmCurrentLocation.theta  + M_PI  <= angles::from_degrees(170))
    {
        searchLoop = 1;
    }
    else if (cnmCurrentLocation.theta  + M_PI  <= angles::from_degrees(260))
    {
        searchLoop = 2;
    }
    else if (cnmCurrentLocation.theta  + M_PI <= angles::from_degrees(350))
    {
        searchLoop = 3;
    }
    
    return searchLoop;
}


//decides which sector to send the rover based on its opposite heading
int SearchController::OctagonSearchStartPosition()
{
    int searchLoop = 0;
    
    if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(45))
    {
        searchLoop = 7;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(90))
    {
        searchLoop = 0;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(135))
    {
        searchLoop = 1;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(180))
    {
        searchLoop = 2;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(225))
    {
        searchLoop = 3;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(275))
    {
        searchLoop = 4;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(315))
    {
        searchLoop = 5;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(360))
    {
        searchLoop = 6;
    }
    
    return searchLoop;
}


//decides which sector to send the rover based on its opposite heading
int SearchController::StarSearchStartPosition()
{
    int searchLoop = 0;
    
    if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(45))
    {
        searchLoop = 1;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(135))
    {
        searchLoop = 0;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(275))
    {
        searchLoop = 2;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(360))
    {
        searchLoop = 1;
    }
    
    return searchLoop;
}

//decides which sector to send the rover based on its opposite heading
int SearchController::SectorSearchStartPosition()
{
    int searchLoop = 0;
    
    if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(45))
    {
        searchLoop = 3;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(90))
    {
        searchLoop = 0;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(135))
    {
        searchLoop = 1;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(225))
    {
        searchLoop = 4;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(270))
    {
        searchLoop = 5;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(315))
    {
        searchLoop = 2;
    }
    else if (cnmCurrentLocation.theta + M_PI<= angles::from_degrees(360))
    {
        searchLoop = 3;
    }
    
    return searchLoop;
}
    
//Added 3-7-2018
void SearchController::cnmSetAvgCurrentLocation(Point cnmAVGCurrentLocation)
{
    cnmCurrentLocation = cnmAVGCurrentLocation;
}

//Added 3-7-2018
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

bool SearchController::updateSearch(){
    //first, check if stash is empty
    //if it is not, return to our previous search state, if possible
    if(stash.wpts.waypoints.size()!=0)
    {
        result = stash;
        searchState = stashState;
        cnmSearchLoop = stashLoop;
        searchCounter = stashCounter;
        //since we still have work on our previous search, return false
        return false;
    }
    //if stash is empty, then we need to go to our next gridpoint, I guess?
    //so return true to indicate we can update our search location
    return true;
}

void SearchController::setStartingPoint(Point p, double radius){
    //set our initial starting point for a search area
    //first, tell our search controller that this is the first waypoint
    first_waypoint = true;
    //then, clear our current results.waypoints vector
    result.wpts.waypoints.clear();
    //then, add our grid area's center point to our waypoints list  
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), p);
    //reset our search state to a wagon wheel? yes?
    //although ideally we would be able to switch among different types of search pattern, 
    //right now our options are to switch between one & random
    searchState = SearchState::SECTOR;
    SetSectorRadius(radius);
    cnmSetCenterLocation(p);
    cnmSearchLoop = 0;
    searchCounter = radius;
}

void SearchController::stashCurrentSearch()
{
    //if there are current seach waypoints
    if(!result.wpts.waypoints.size()==0)
    {
        //copy current waypoints into our stash
        stash = result;
        stashState = searchState;
        stashLoop = cnmSearchLoop;
        stashCounter = searchCounter;
        cnmSearchLoop = 0;
    }
}

void SearchController::clearStash()
{
    stash.wpts.waypoints.clear();
    stashLoop = 0;
    stashCounter = 0;
}

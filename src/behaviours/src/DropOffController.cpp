#include "DropOffController.h"



DropOffController::DropOffController() {


reachedCollectionPoint = false;

result.type = behavior;
result.b = wait;
result.wristAngle = 0.7;
result.reset = false;
interrupt = false;

circularCenterSearching = false;
spinner = 0;
centerApproach = false;
seenEnoughCenterTags = false;
prevCount = 0;

countLeft = 0;
countRight = 0;

negLeft = 0;
negRight = 0;

isPrecisionDriving = false;
startWaypoint = false;
timerTimeElapsed = -1;

readyToDrop = false;
cTagcount = 0;
cnmReverse = false;
isDroppingOff = false;
CNMCentered = false;
backUp = false;
firstReverse = true;
dropTimerStatered = false;
reverseTimerStatered = false;
cnmDropoff180 = false;
cnm180Finished = false;
the180TimerStatered = false;


}

DropOffController::~DropOffController() {

}

Result DropOffController::DoWork() {

bool left = (countLeft > 0);
bool right = (countRight > 0);
centerSeen = (right || left);
int count = countLeft + countRight;

cout << "DROPOFF - DoWork    elapsed time: " << timerTimeElapsed << endl;

//start timer
if(timerTimeElapsed > -1)
{
  long int elapsed = current_time - returnTimer;
  timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
}

double distanceToCenter = hypot(cnmCenterLocation.x - this->currentLocation.x, cnmCenterLocation.y - this->currentLocation.y);

//check to see if we are driving to the center location or if we need to drive in a circle and look.
cout << "DROPOFF - distCenter > collectPts" << (distanceToCenter > collectionPointVisualDistance)<< endl;
cout << "DROPOFF - circSearch " << !circularCenterSearching<< endl;
cout << "DROPOFF - count == 0" << (count == 0)<< endl;
cout << "DROPOFF - CNMCentered " << CNMCentered<<endl;
if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && (count == 0))
{
  cout << "DROPOFF - Checked Distance and driving to waypoint" << endl;
  result.type = waypoint;
  result.wpts.waypoints.clear();
  result.wpts.waypoints.push_back(cnmCenterLocation);
  startWaypoint = false;
  isPrecisionDriving = false;

  timerTimeElapsed = 0;

  return result;
}

//if we seen center start precisionDiving and align with tags
if(centerSeen)
{

  if (first_center && isPrecisionDriving)
  {

    centerApproach = true;

    cout << "DROPOFF - First time seeing center setting precisionDriving" << endl;
    first_center = false;

    result.type = behavior;
    result.reset = false;
    result.b = nextProcess;

    //result.type = precisionDriving;
    //result.pd.cmdVel = 0;
    //result.pd.cmdAngularError= 0;
  //  result.reset = false;
    //result.PIDMode = SLOW_PID;
    return result;

  }
  isPrecisionDriving = true;
  //if we have not centered up with home
  if(!CNMCentered)
  {
    result.type = precisionDriving;
    result.PIDMode = SLOW_PID;
    DropOffController::cnmCenteringNow();
  }
  //once centered drive forward and stop after dropoff timer complete
  if(CNMCentered)
  {
    cout << "8" << endl;
    if(!dropTimerStatered)
    {
      cout << "DROPOFF - CNM Centered: " << CNMCentered << "  dropoff timer started!" << endl;
      cnmDropoffTimerStart = current_time;
      dropTimerStatered = true;

      result.pd.cmdVel = 0.25; // was 1.5 from CNM 2017
      result.pd.cmdAngularError  = 0.0;
    }

    //start timer for dropoff drive
    long int  elapsedDropoff = current_time - cnmDropoffTimerStart;
    float cnmDropoffTimerElapsed = elapsedDropoff/1e3; // Convert from milliseconds to seconds

    if(cnmDropoffTimerElapsed >= cnmDropoffTimer && dropTimerStatered && !readyToDrop)
    {
      cout << "DROPOFF - dropoff timer: " << cnmDropoffTimerElapsed << endl;
      cout << "DROPOFF - Dropoff Drive Timer complete stop here and drop" << endl;
      result.pd.cmdVel = 0.00;
      result.pd.cmdAngularError  = 0.0;
      reachedCollectionPoint =true;
      readyToDrop = true;

    }
    //addition adjustment while driving forward on timer (not used)
    if(readyToDrop)
    {
      //cout << "DROPOFF - Now readyToDrop calling fianl adjustment" << endl;
      //DropOffController::finalAdjustmentBeforeDrop();
    }
  }
}

//if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
//to resart our search.


//once dropoff drvie timer is complete open gripper and start reverse
if(reachedCollectionPoint)
{
  cout << "DROPOFF - reachedCollectionPoint" << endl;
  cout << "2" << endl;

  if(!reverseTimerStatered)
  {
    cout << "DROPOFF - opening fingers now" << endl;
    result.fingerAngle = M_PI_2; //open fingers
    result.wristAngle = 0; //raise wrist
    result.pd.cmdVel = -0.2;
    result.pd.cmdAngularError = 0.0;
    cnmReverse = true;
  }
  if(cnmReverse && firstReverse)
  {
    firstReverse = false;
    cnmReverseTimerStart = current_time;
    cout << "DROPOFF - reverse timer started" << endl;
    reverseTimerStatered = true;
  }

  long int  elapsedReverse = current_time - cnmReverseTimerStart;
  float cnmReverseTimerElapsed = elapsedReverse/1e3; // Convert from milliseconds to seconds

  //stop after reversing timer complete and call for final interupt
  if(cnmReverseTimerElapsed >= cnmReverseTimer && cnmReverse)
  {
    cout << "DROPOFF - Reverse timer: " << cnmReverseTimerElapsed << endl;
    result.pd.cmdVel = 0.0;
    result.pd.cmdAngularError = 0.0;
    cnmReverse = false;
    cnmDropoff180 = true;
    cnm180Finished = true;
    finalInterrupt = true;
  }

/*
  if(cnmDropoff180 && !the180TimerStatered)
  {
    cout << "DROPOFF - turning 180 meow" << endl;
    cout << "DROPOFF - current theta: " << currentLocation.theta << " + PI = " << ( currentLocation.theta + M_PI) << endl;
    result.pd.cmdAngularError = M_PI;
    cnm180Finished = true;
    finalInterrupt = true;
    //cnm180TimerStart = current_time;
    //the180TimerStatered = true;
  }

  long int  elapsed180 = current_time - cnm180TimerStart;
  float cnm180TimerElapsed = elapsed180/1e3; // Convert from milliseconds to seconds

  if(cnm180TimerElapsed >= cnm180Timer && !cnm180Finished)
  {
    cout << "DROPOFF - Dropoff 180 Timer complete calling final interupt" << endl;
    cnm180Finished = true;
    finalInterrupt = true;
      //targetHeld =false;
  }
*/

}


if (finalInterrupt)
{
  cout << "DROPOFF - finalInterrupt";
  result.type = behavior;
  result.b = nextProcess;
  result.reset = true;
  return result;
}
//  return result;
//  }


/*
//was on approach to center and did not seenEnoughCenterTags
//for lostCenterCutoff seconds so reset.
else if (centerApproach)
{

long int elapsed = current_time - lastCenterTagThresholdTime;
float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds
if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff)
{
  cout << "Switching to drive to center" << endl;
  //go back to drive to center base location instead of drop off attempt
  reachedCollectionPoint = false;
  seenEnoughCenterTags = false;
  centerApproach = false;

  result.type = waypoint;
  result.wpts.waypoints.push_back(cnmCenterLocation);
  if (isPrecisionDriving) {
    result.type = behavior;
    result.b = prevProcess;
    result.reset = false;
  }
  isPrecisionDriving = false;
  interrupt = false;
  precisionInterrupt = false;
}
else
{
  result.pd.cmdVel = searchVelocity;
  result.pd.cmdAngularError = 0.0;
}

return result;

}



else if (timerTimeElapsed >= 2)//spin search for center
{
  Point nextSpinPoint;
  cout << "DROPOFF - spin search" << endl;

  //sets a goal that is 60cm from the centerLocation and spinner
  //radians counterclockwise from being purly along the x-axis.
  nextSpinPoint.x = cnmCenterLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
  nextSpinPoint.y = cnmCenterLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
  nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);

  result.type = waypoint;
  result.wpts.waypoints.clear();
  result.wpts.waypoints.push_back(nextSpinPoint);

  spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
  if (spinner > 2*M_PI) {
    spinner -= 2*M_PI;
  }
  spinSizeIncrease += spinSizeIncrement/8;
  circularCenterSearching = true;
  //safety flag to prevent us trying to drive back to the
  //center since we have a block with us and the above point is
  //greater than collectionPointVisualDistance from the center.

  returnTimer = current_time;
  timerTimeElapsed = 0;

}
*/
//reset lastCenterTagThresholdTime timout timer to current time
if ((!CNMCentered && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {

  lastCenterTagThresholdTime = current_time;

}

/*
if (count > 0 || seenEnoughCenterTags || prevCount > 0) //if we have a target and the center is located drive towards it.
{
  cout << "DROPOFF - seen enough tags" << endl;
  cout << "9" << endl;
  centerSeen = true;
  isPrecisionDriving = true;



  {
    cout << "DROPOFF - CNM Centered on home" << endl;
    if (first_center && isPrecisionDriving)
    {
      cout << "DROPOFF - Setting result type once CNM Centered" << endl;
      first_center = false;
      result.type = behavior;
      result.reset = false;
      result.b = nextProcess;
      result.PIDMode = SLOW_PID;
      return result;

      cnmDropoffTimerStart = current_time;
      cout << "DROPOFF - dropoff timer started" << endl;
    }

    //Once Squared Up, trigger in position
    result.pd.cmdVel = 0.25; // was 1.5 from CNM 2017
    result.pd.cmdAngularError  = 0.0;
    //start timer for dropoff drive


    long int  elapsedDropoff = current_time - cnmDropoffTimerStart;
    float cnmDropoffTimerElapsed = elapsedDropoff/1e3; // Convert from milliseconds to seconds


    if(cnmDropoffTimerElapsed >= cnmDropoffTimer)
    {cout << "DROPOFF - dropoff timer: " << cnmDropoffTimerElapsed << endl;
      cout << "DROPOFF - Dropoff Dirve Timer complete stop here and drop" << endl;
      result.pd.cmdVel = 0.00;
      result.pd.cmdAngularError  = 0.0;
      readyToDrop = true;
    }

    if(readyToDrop)
    {
      cout << "DROPOFF - readyToDrop" << endl;
      DropOffController::finalAdjustmentBeforeDrop();
    }

  //must see greater than this many tags before assuming we are driving into the center and not along an edge.
  if (count > centerTagThreshold)
  {
    cout << "DROPOFF - seenEnoughCenterTags = true" << endl;
    seenEnoughCenterTags = true; //we have driven far enough forward to be in and aligned with the circle.
    lastCenterTagThresholdTime = current_time;
  }
  if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
  {
    lastCenterTagThresholdTime = current_time;
  }
  //time since we dropped below countGuard tags
  long int elapsed = current_time - lastCenterTagThresholdTime;
  float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds

  //we have driven far enough forward to have passed over the circle.
  if (count < 1 && seenEnoughCenterTags && timeSinceSeeingEnoughCenterTags > dropDelay) {
    centerSeen = false;
  }
  centerApproach = true;
  prevCount = count;
  count = 0;
  countLeft = 0;
  countRight = 0;
  negLeft = 0;
  negRight = 0;
}
}

//was on approach to center and did not seenEnoughCenterTags
//for lostCenterCutoff seconds so reset.
else if (centerApproach) {

  cout << "DROPOFF - centerApproach" << endl;
  long int elapsed = current_time - lastCenterTagThresholdTime;
  float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds
  if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff)
  {
    cout << "4" << endl;
    //go back to drive to center base location instead of drop off attempt
    reachedCollectionPoint = false;
    seenEnoughCenterTags = false;
    centerApproach = false;

    result.type = waypoint;
    result.wpts.waypoints.push_back(cnmCenterLocation);
    if (isPrecisionDriving) {
      result.type = behavior;
      result.b = prevProcess;
      result.reset = false;
    }
    isPrecisionDriving = false;
    interrupt = false;
    precisionInterrupt = false;
  }
  else
  {
    result.pd.cmdVel = searchVelocity;
    result.pd.cmdAngularError = 0.0;
  }

  return result;

}

/*
if (!centerSeen && seenEnoughCenterTags)
{
  reachedCollectionPoint = true;
  centerApproach = false;
  returnTimer = current_time;
}

else if(backUp)
{
  result.pd.cmdVel = -0.15;
  result.pd.cmdAngularError = 0.0;
  cnmReverseTimerStart = current_time;
}
*/
return result;

}


void DropOffController::finalAdjustmentBeforeDrop()
{

cout << "DROPOFF - Aligned and making final adjustment" << endl;

int count = countLeft + countRight;

float turnDirection = 1;
//reverse tag rejection when we have seen enough tags that we are on a
//trajectory in to the square we dont want to follow an edge.
//if (seenEnoughCenterTags) turnDirection = -3;

float turnThisWay = 0;

result.type = precisionDriving;

if (count > 8)
{
cout << "DROPOFF - Tags > 8 dropping now!" << endl;
reachedCollectionPoint =true;
//result.pd.cmdVel = -0.2;
result.pd.cmdAngularError  = 0.0;
}
else if (count > 3 && count <= 8)
{

cout << "DROPOFF - Tags between 3 and 8" << endl;

//otherwise turn till tags on both sides of image then drive straight
if ((countLeft - 6) <= 0 && (countRight - 6) <=0) { //CNM 2017 values Go straight in
  cout << "DROPOFF - Tag Count is even" << countLeft << " = " << countRight << endl;
  //result.pd.cmdVel = 0.0;
}
else if (countLeft < (countRight - 6)){ //CNM 2017 values Turn Left
//  result.pd.cmdVel = 0.0;
result.pd.cmdAngularError  = centeringTurnRate*turnDirection;
}
else if (countLeft > (countRight - 6)) { //CNM 2017 values Turn Right
//  result.pd.cmdVel = 0.0;
  result.pd.cmdAngularError  = -centeringTurnRate*turnDirection;
}
/*
else if(negLeft)
{
    cout << "DROPOFF - YAW - Negative Left count: " << negLeft << " we should be turning right!" << endl;
    result.pd.cmdVel = 0.0;
    result.pd.cmdAngularError  = -centeringTurnRate*turnDirection;
}
else if(negRight)
{
    cout << "DROPOFF - YAW - Negative Right count: " << negLeft << " we should be turning left!" << endl;
    result.pd.cmdVel = 0.0;
    result.pd.cmdAngularError  = centeringTurnRate*turnDirection;
}
*/

}
  else
  {
    cout << "DROPOFF - Tags less than 3; Dropping off!" << endl;
    //result.pd.cmdVel = -0.2;
    result.pd.cmdAngularError = 0.0;

    reachedCollectionPoint = true;
  }
}

void DropOffController::cnmCenteringNow()
{

cout << "DROPOFF - cnmCenteringNow called" << endl;

float linearSpeed, angularSpeed;

//VARIABLES
//-----------------------------------
const int amountOfTagsToSee = 5;
bool enoughTagsSeen = false;
bool right, left;
int count = countLeft + countRight;

if(!cnmReverse && !CNMCentered)
{
    if (countRight > 0) { right = true; }
    else { right = false; }

    if (countLeft > 0) { left = true; }
    else { left = false; }

    if(count > amountOfTagsToSee) { enoughTagsSeen = true;}
    else { enoughTagsSeen = false; }

    float turnDirection = 1;

    if (enoughTagsSeen) //if we have seen enough tags
    {
      if ((countLeft - 5) > countRight) //and there are too many on the left
      {
        right = false; //then we say none on the right to cause us to turn right
      }
      else if ((countRight - 5) > countLeft)
      {
        left = false; //or left in this case
      }

        //otherwise turn till tags on both sides of image then drive straight
      if(left && right) //2017 included confirmedCenter &&
      {
        //confirmedCenter = false;
        cout << "DROPOFF - CNM Centered = true" << endl;
        CNMCentered = true;
        //cnmFinishedCenteringTimer.start();
        //return true;
      }
      /*
      else if(left && right)
        {
          if(isDroppingOff)
          {
            cnmConfirmCentered.start();

            cnmGoHomeAgain.stop();

            goalLocation = currentLocationMap;
            sendDriveCommand(0.0, 0.0);
            return false;
          }
          else
          {
            cnmFinishedCenteringTimer.start();
            return true;
          }

          cnmGoHomeAgain.stop();

        }
        */
    }

//cnmConfirmCentered.stop();
//cnmFinishedCenteringTimer.stop();

    if (right)
    {
      cout << "DROPOFF - Centering right" << endl;
        linearSpeed = 0.15;
        angularSpeed = -0.15;
        //cnmGoHomeAgain.stop();
        //CNMCentered = false;
    }
    else if (left)
    {
      cout << "DROPOFF - Centering left" << endl;
        linearSpeed = -0.15;
        angularSpeed = 0.15;
        //cnmGoHomeAgain.stop();
        //CNMCentered = false;
    }
    else
    {
      cout << "DROPOFF - Centering -set drive speed" << endl;
      if(isDroppingOff) { linearSpeed = 0.25; }
      else { linearSpeed = 0.15; }

      angularSpeed = 0.0;
      //CNMCentered = false;

      //cnmGoHomeAgain.start();

    }

    result.pd.cmdVel = linearSpeed;
    result.pd.cmdAngularError = angularSpeed;
}
}



void DropOffController::Reset() {
cout << "DROPOFF - DropoffController Reset" << endl;
result.type = behavior;
result.b = wait;
result.pd.cmdVel = 0;
result.pd.cmdAngularError = 0;
result.fingerAngle = -1;
result.wristAngle = 0.7;
result.reset = false;
result.wpts.waypoints.clear();
result.PIDMode = FAST_PID;
spinner = 0;
spinSizeIncrease = 0;
prevCount = 0;
timerTimeElapsed = -1;

countLeft = 0;
countRight = 0;


//reset flags
reachedCollectionPoint = false;
seenEnoughCenterTags = false;
circularCenterSearching = false;
isPrecisionDriving = false;
finalInterrupt = false;
precisionInterrupt = false;
targetHeld = false;
startWaypoint = false;
first_center = true;
cout << "6" << endl;

CNMCentered = false;
backUp = false;
readyToDrop = false;
cTagcount = 0;
cnmReverse = false;
isDroppingOff = false;
firstReverse = true;
centerSeen = false;
dropTimerStatered = false;
reverseTimerStatered = false;
cnmDropoff180 = false;
cnm180Finished = false;
the180TimerStatered = false;


}

void DropOffController::SetTargetData(vector<Tag> tags) {
countRight = 0;
countLeft = 0;

if(targetHeld) {
  // if a target is detected and we are looking for center tags
  if (tags.size() > 0 && !reachedCollectionPoint) {

    // this loop is to get the number of center tags
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].getID() == 256) {

        //cout << "TAG - Right Tag Orientation X: "<< tags[i].getOrientationX() << "  Y: " << tags[i].getOrientationY() << "  Z: " << tags[i].getOrientationZ() << endl;

        /*
        if(tags[i].getPositionX() + cameraOffsetCorrection > 0 && tags[i].getOrientationY() < 0)
        {

        }
        */

        // checks if tag is on the right or left side of the image
        if (tags[i].getPositionX() + cameraOffsetCorrection > 0) {
          //cout << "YAW - Tag Right: " << tags[i].getOrientationY() << endl;
          countRight++;

          if(tags[i].getOrientationY() < 0)
          {
            negRight++;
          }

        } else {
          //cout << "YAW - Tag Left: " << tags[i].getOrientationY() << endl;
          countLeft++;

          if(tags[i].getOrientationY() < 0)
          {
            negLeft++;
          }

        }
      }
    }
  }
}

}

void DropOffController::ProcessData() {
if((countLeft + countRight) > 0) {
  isPrecisionDriving = true;
} else {
  startWaypoint = true;
}
}

bool DropOffController::ShouldInterrupt() {
ProcessData();
if (startWaypoint && !interrupt) {
  interrupt = true;
  precisionInterrupt = false;
  return true;
}
else if (isPrecisionDriving && !precisionInterrupt) {
  precisionInterrupt = true;
  return true;
}
if (finalInterrupt) {
  return true;
}
}

bool DropOffController::HasWork() {

if(timerTimeElapsed > -1) {
  long int elapsed = current_time - returnTimer;
  timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
}

if (circularCenterSearching && timerTimeElapsed < 2 && !isPrecisionDriving) {
  return false;
}

return ((startWaypoint || isPrecisionDriving));
}

bool DropOffController::IsChangingMode() {
return isPrecisionDriving;
}


void DropOffController::SetCenterLocation(Point center) {
centerLocation = center;
}


void DropOffController::cnmSetCenterLocation(Point center) {
cnmCenterLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
targetHeld = true;
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
targetHeld = targetHeld || blockBlock;
}

void DropOffController::SetCurrentTimeInMilliSecs( long int time )
{
current_time = time;
}

void DropOffController::cnmSetAvgCurrentLocation(Point cnmAVGCurrentLocation)
{
cnmCurrentLocation = cnmAVGCurrentLocation;
}

bool DropOffController::CNMCurrentLocationAVG()
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

DropOffController::cnmSetAvgCurrentLocation(cnmAVGCurrentLocation);

index = 0;
return true;
  }
}

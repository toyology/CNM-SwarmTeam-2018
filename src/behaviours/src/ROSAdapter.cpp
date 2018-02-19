#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "swarmie_msgs/Waypoint.h"

// Include Controllers
#include "LogicController.h"
#include <vector>
#include <iterator>

#include "Point.h"
#include "Tag.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include <exception> // For exception handling

#include <fstream>
#include <iostream>
#include <math.h>

using namespace std;

// Define Exceptions
// Define an exception to be thrown if the user tries to create
// a RangeShape using invalid dimensions
class ROSAdapterRangeShapeInvalidTypeException : public std::exception {
public:
  ROSAdapterRangeShapeInvalidTypeException(std::string msg) {
    this->msg = msg;
  }

  virtual const char* what() const throw()
  {
    std::string message = "Invalid RangeShape type provided: " + msg;
    return message.c_str();
  }

private:
  std::string msg;
  //std_msgs::String Msg;     //sortOrder
};

// Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create logic controller
LogicController logicController;

void humanTime();

// Behaviours Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees
void resultHandler();
void CNMFirstBoot();        //StartOrder
void sortOrder();     //SortOrder

Point updateCenterLocation();
void transformMapCentertoOdom();

// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocation;          //current location of robot
geometry_msgs::Pose2D currentLocationMap;       //current location on MAP
geometry_msgs::Pose2D currentLocationAverage;   //???

geometry_msgs::Pose2D centerLocation;           //location of center location
geometry_msgs::Pose2D centerLocationMap;        //location of center on map
geometry_msgs::Pose2D centerLocationOdom;       //location of center ODOM
geometry_msgs::Pose2D centerLocationMapRef;

int currentMode = 0;
const float behaviourLoopTimeStep = 0.1; // time between the behaviour loop calls
const float status_publish_interval = 1;
const float heartbeat_publish_interval = 2;
const float waypointTolerance = 0.1; //10 cm tolerance.

// used for calling code once but not in main
bool initilized = false;

float linearVelocity = 0;
float angularVelocity = 0;

float prevWrist = 0;
float prevFinger = 0;
long int startTime = 0;
float minutesTime = 0;
float hoursTime = 0;


float drift_tolerance = 0.5; // meters

Result result;

geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

// Publishers
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher driveControlPublish;
ros::Publisher heartbeatPublisher;
// Publishes swarmie_msgs::Waypoint messages on "/<robot>/waypoints"
// to indicate when waypoints have been reached.
ros::Publisher waypointFeedbackPublisher;
//AJH added publisher declaration for manual waypoint publisher
ros::Publisher manualWaypointPublisher;

//CNM publishers
ros::Publisher startOrderPub;		//startOrder
ros::Publisher sortOrderPub;		//SortOrder
ros::Publisher broadcastPub;
//ros::Publisher obstacleWaypointPub;
//ros::Publisher miscWaypointPub;
//TODO: testing functionality of reusing variable name
ros::Publisher namedSwarmiePub;

// Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;
ros::Subscriber virtualFenceSubscriber;
// manualWaypointSubscriber listens on "/<robot>/waypoints/cmd" for

ros::Subscriber manualWaypointSubscriber;
ros::Subscriber startOrderSub;			//startOrder
ros::Subscriber sortOrderSub;			//SortOrder
ros::Subscriber myNameSub;
ros::Subscriber broadcastSub;

// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer publish_heartbeat_timer;

// records time for delays in sequenced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to
// average its location.
unsigned int startDelayInSeconds = 30;
float timerTimeElapsed = 0;

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void virtualFenceHandler(const std_msgs::Float32MultiArray& message);
void manualWaypointHandler(const swarmie_msgs::Waypoint& message);
void behaviourStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);

//CNM handlers
void startOrderHandler(const std_msgs::String& msg);			//startOrder
void sortOrderHandler(const std_msgs::String& msg);	//SortOrder

void myMessageHandler(const swarmie_msgs::Waypoint& my_msg);

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int getROSTimeInMilliSecs();

/* CNM added code --------------------------------------------------------------
------------------------------------------------------------------------------*/
Point cnmCenterLocation;
bool resetMap = false;
void CNMAVGCenter();       //Averages derived center locations
bool purgeMap = false;

//AJH added variables:
int switchSwarmie = 0;
bool roleReady = true;
bool gpsAverageSent = false;
int gpsCount = 0;
int searchStart = 0;
int update = 0;
enum class Role{
    //teamsize == 3
    gather1, //searches close to center, gathers drop offs from searchers, helps searchers
    searcher1, //searches assigned areas, drops off for gatherer
    searcher2, //searches assigned areas, drops off for gatherer
    //teamsize == 6 (or > 3)
    gather2,  //hybrid searches & gathers based on time
    searcher3, //searches assigned areas, drops off for gatherer
    searcher4 //hybrid searches & gathers based on time
  };
//variable to hold my role 
Role myRole;
int myStartTime;
const int* myAreasBig;
const int* myAreasSmall;
vector<ros::Publisher> comms;
//my integer id, which prompts swarmie role selection/assignment
int myID;
//set this to something big initially so we don't update tasks every timer tick
float taskTime = 100;
float nextTask = 100;
ros::NodeHandle *cnm_NH;

swarmie_msgs::Waypoint wmsg;
swarmie_msgs::Waypoint my_msg;
std_msgs::String msg;
std_msgs::String Msg;    //sortOrder

bool hasTested = false;
bool gpsAveraged = false;
bool mapBuilt = false;
vector<std::string> swarmieNames;
void assignSwarmieRoles(int startTime);
void updateBehavior(int currentTime, int update);
void buildMap();
//build & store our 25 grid areas as points, which 
//will become fence locations
Point mapAreas[25];
//create an array to store up to 30 obstacle locations 
//as 'fences' built with range controller
RangeController foundObstacles[30]; 
//AJH for testing, obvs. Hoping that was obvious to everyone involved
void testStuff();
//Kaily's Outlier Code
Point coord[50];
Point removeOutliers(int data_types, Point gps_points[], int data_points);
double fireTimer = 3.25;
int indexTest = 0;


//ARRAYS FOR CENTER
const int ASIZE = 200;
int centerIndex = 0;

//TODO AJH possibly we're no longer using those???
bool maxedCenterArray = false;

const int CASIZE = 30;

float avgCurrentCoordsX[CASIZE];
float avgCurrentCoordsY[CASIZE];

//Point cnmCurrentLocation;
void CNMCurrentLocationAVG();      //Averages current location on map
void CNMProjectCenter();

//Actual Center Array
Point CenterCoords[ASIZE];
float CenterXCoordinates[ASIZE];
float CenterYCoordinates[ASIZE];

//INITIAL NEST SEARCH
bool cnmFirstBootProtocol = true;
bool cnmHasWaitedInitialAmount = false;
bool cnmInitialPositioningComplete = false;
bool cnmHasMovedForward = false;
bool cnmHasTurned180 = false;

//VAIRABLES FOR //startOrder
bool testCount = true;		//bool for window to increment
float cnmStartOrder = 0;           //startCheck
bool sortTrigger = true;
bool sortTrigger1 = true;
bool sortTrigger2 = true;

// IP address test @@@@
#include <arpa/inet.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <stdio.h>

/* @@@@ */

int main(int argc, char **argv) {
  /* @@@ */
    
  gethostname(host, sizeof (host));
  string hostname(host);

  if (argc >= 2) {
    publishedName = argv[1];
    cout << "Welcome to the world of tomorrow " << publishedName
         << "!  Behaviour turnDirectionule started." << endl;
  } else {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }

  // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (publishedName + "_BEHAVIOUR"), ros::init_options::NoSigintHandler);
  ros::NodeHandle mNH;
  cnm_NH = &mNH;
  //stash.stashComms(&mNH);

  // Register the SIGINT event handler so the node can shutdown properly
  signal(SIGINT, sigintEventHandler);

  joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
  modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
  targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
  odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
  mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);
  virtualFenceSubscriber = mNH.subscribe(("/virtualFence"), 10, virtualFenceHandler);
  manualWaypointSubscriber = mNH.subscribe((publishedName + "/waypoints/cmd"), 10, manualWaypointHandler);
  message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (publishedName + "/sonarLeft"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (publishedName + "/sonarCenter"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (publishedName + "/sonarRight"), 10);

  //CNM CODE
  startOrderSub = mNH.subscribe("startOrder", 1000, &startOrderHandler);			//startOrder
  sortOrderSub = mNH.subscribe("sortOrder", 1000, &sortOrderHandler);				//sortOrder
  //AJH: each swarmie has an individual and broadcast subscriber, because some messages are targeted
  //but some messages will need to be sent out to all swarmies at once
  myNameSub =  mNH.subscribe(("dear"+publishedName), 10, &myMessageHandler);
  broadcastSub = mNH.subscribe("broadcast", 10, &myMessageHandler); 

  //CNM CODE
  startOrderPub = mNH.advertise<std_msgs::String>("startOrder", 1000);			//startOrder
  sortOrderPub = mNH.advertise<std_msgs::String>("sortOrder", 1000);			//sortOrder
  //all swarmies subscribe to the broadcast message, and all swarmies hail to the hypnotoad
  broadcastPub = mNH.advertise<swarmie_msgs::Waypoint>("broadcast", 100, true);

  status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
  stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
  fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
  wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
  driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);
  heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/behaviour/heartbeat"), 1, true);
  manualWaypointPublisher = mNH.advertise<swarmie_msgs::Waypoint>((publishedName + "/waypoints/cmd"), 10, true);
  waypointFeedbackPublisher = mNH.advertise<swarmie_msgs::Waypoint>((publishedName + "/waypoints"), 1, true);

  publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
  stateMachineTimer = mNH.createTimer(ros::Duration(behaviourLoopTimeStep), behaviourStateMachine);

  publish_heartbeat_timer = mNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

  message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
  sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

  tfListener = new tf::TransformListener();
  std_msgs::String msg;
  msg.data = "Log Started";
  infoLogPublisher.publish(msg);

  stringstream ss;
  ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
  msg.data = ss.str();
  infoLogPublisher.publish(msg);

  if(currentMode != 2 && currentMode != 3)
  {
    // ensure the logic controller starts in the correct mode.
    logicController.SetModeManual();
  }

  timerStartTime = time(0);


//ss << "IP Address running"<< ip<< "Identity";
//        msg.data = ss.str();
//        infoLogPublisher.publish(msg);


  ros::spin();

  return EXIT_SUCCESS;
}


// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.

void behaviourStateMachine(const ros::TimerEvent&)
{

//const int ASIZE = 200;
//int centerIndex = 0;
//bool fireTimer = ((lrint(timerTimeElapsed)*10)%3 == 0);
if(timerTimeElapsed > fireTimer && !gpsAveraged)
{
  if(indexTest < 50)
  {
    coord[indexTest].x = currentLocationMap.x + (1.3 * cos(currentLocation.theta));
    coord[indexTest].y = currentLocationMap.y + (1.3 * sin(currentLocation.theta));
    //cout << "OUTLIER - current time is "<< fireTimer <<", reading gps pt # " << indexTest <<": " << coord[indexTest].x << ", " << coord[indexTest].y << endl;
    coord[indexTest].theta = currentLocationMap.theta;
    indexTest++;
  }  
  if(indexTest == 50 && !gpsAveraged)
  {
    cout << "OUTLIER - finished collecting points" << endl;
    gpsAveraged = true;
    cnmCenterLocation = removeOutliers(3,coord,50);
    //logicController.SetCenterLocationMap(cnmCenterLocation);
    cout << "OUTLIER - " << publishedName << " my pts to average: " << cnmCenterLocation.x << ", " << cnmCenterLocation.y << endl;
  }
  fireTimer += .25;
}


if (timerTimeElapsed > 31)
{
    CNMFirstBoot();               //StartOrder
}

if (timerTimeElapsed > 33)
{
    sortOrder();
}

//TODO: AJH if a swarmie crashes & reboots, we want them to have a copy of their role & 
//the time that they were working 
//also, this is almost a minute after boot, but any earlier and we can't 
//guarantee the efficacy of our role call messages
if(gpsAveraged && !mapBuilt && timerTimeElapsed > 40)
{
  mapBuilt = true;
  buildMap();
  if(roleReady)
  {
    roleReady = false;
    assignSwarmieRoles(timerTimeElapsed);
  }
}

/*
//TODO: AJH this is just for testing! I would never in good conscience hard code 
//a comparison to a published name. That would be silly.
if(timerTimeElapsed > 53 && publishedName == "artemis" && firstUpdate) 
{
  firstUpdate = false;
  testStuff();
}*/

//at each taskTime elapsed (timerTimeElapsed is in seconds), 
//update our roles
//TODO AJH update this timer to actually fucking work, please
/*
if((timerTimeElapsed/60) >= nextTask){
  updateBehavior(timerTimeElapsed, update);
  cout << " MAPPING - updating our behavior at time " << timerTimeElapsed << endl;
  //set our next task timer deadline
  nextTask += taskTime;
  update++;
}
*/

/*if (sortTrigger1 == false)
{
  if (sortTrigger2)
  {
     sortTrigger2 = false;
        std_msgs::String msg;
        msg.data = "sortTriggerHandler running ";
        infoLogPublisher.publish(msg);
  }
}*/
  std_msgs::String stateMachineMsg;

  // time since timerStartTime was set to current time
  timerTimeElapsed = time(0) - timerStartTime;

  // init code goes here. (code that runs only once at start of
  // auto mode but wont work in main goes here)
  if (!initilized)
  {
    
    if (timerTimeElapsed > startDelayInSeconds)
    {
      // initialization has run
      initilized = true;
      //TODO: this just sets center to 0 over and over and needs to change
      Point centerOdom;
      centerOdom.x = 1.3 * cos(currentLocation.theta);
      centerOdom.y = 1.3 * sin(currentLocation.theta);
      centerOdom.theta = centerLocation.theta;
      logicController.SetCenterLocationOdom(centerOdom);

      Point& centerMap = cnmCenterLocation;
      //centerMap.x = currentLocationMap.x + (1.3 * cos(currentLocation.theta));
      //centerMap.y = currentLocationMap.y + (1.3 * sin(currentLocation.theta));
      //centerMap.theta = centerLocationMap.theta;
      //just use our cnm calculated version of this
      //&& this should mean centerMap actually points to cnmCenterMap
      logicController.SetCenterLocationMap(centerMap);

      /*Point cnmCenterMap;
      cnmCenterMap.x = currentLocationMap.x + (1.3 * cos(currentLocation.theta));
      cnmCenterMap.y = currentLocationMap.y + (1.3 * sin(currentLocation.theta));
      cnmCenterMap.theta = centerLocationMap.theta;
      */

      /*for(int i = 0; i < ASIZE; i++)
	    {
		  //flood x and y coordinates
      //problem: these are all the same point, because they're being read really, reeeaaally quickly
	        CenterXCoordinates[i] = currentLocationMap.x;
    	    CenterYCoordinates[i] = currentLocationMap.y;

          myFile << " x:"<< currentLocationMap.x << " y:" << currentLocationMap.y << " index:" << i;

          stringstream ss;
          ss << "reading position X: " <<   currentLocationMap.x << " Y: " << currentLocationMap.y << "  i: " << i << endl;
          msg.data = ss.str();
          infoLogPublisher.publish(msg);
	    }
      CNMProjectCenter();
      */

      centerLocationMap.x = centerMap.x;
      centerLocationMap.y = centerMap.y;

      centerLocationOdom.x = centerOdom.x;
      centerLocationOdom.y = centerOdom.y;

      startTime = getROSTimeInMilliSecs();
    }

    else
    {
      return;
    }

  }

  // Robot is in automode
  if (currentMode == 2 || currentMode == 3)
  {

    humanTime();

    //update the time used by all the controllers
    logicController.SetCurrentTimeInMilliSecs( getROSTimeInMilliSecs() );

    //update center location
    logicController.SetCenterLocationOdom( updateCenterLocation() );

    //ask logic controller for the next set of actuator commands
    result = logicController.DoWork();

    bool wait = false;

    //if a wait behaviour is thrown sit and do nothing untill logicController is ready
    if (result.type == behavior)
    {
      if (result.b == wait)
      {
        wait = true;
      }
    }

    //do this when wait behaviour happens
    if (wait)
    {
      sendDriveCommand(0.0,0.0);
      std_msgs::Float32 angle;

      angle.data = prevFinger;
      fingerAnglePublish.publish(angle);
      angle.data = prevWrist;
      wristAnglePublish.publish(angle);
    }

    //normally interpret logic controllers actuator commands and deceminate them over the appropriate ROS topics
    else
    {

      sendDriveCommand(result.pd.left,result.pd.right);

      //Alter finger and wrist angle is told to reset with last stored value if currently has -1 value
      std_msgs::Float32 angle;
      if (result.fingerAngle != -1)
      {
        angle.data = result.fingerAngle;
        fingerAnglePublish.publish(angle);
        prevFinger = result.fingerAngle;
      }

      if (result.wristAngle != -1)
      {
        angle.data = result.wristAngle;
        wristAnglePublish.publish(angle);
        prevWrist = result.wristAngle;
      }
    }

    //publishHandeling here
    //logicController.getPublishData(); suggested
    //adds a blank space between sets of debugging data to easily tell one tick from the next
    cout << endl;

  }

  // mode is NOT auto
  else
  {
    humanTime();

    logicController.SetCurrentTimeInMilliSecs( getROSTimeInMilliSecs() );

    // publish current state for the operator to see
    stateMachineMsg.data = "WAITING";

    // ask the logicController to get the waypoints that have been
    // reached.
    std::vector<int> cleared_waypoints = logicController.GetClearedWaypoints();

    for(std::vector<int>::iterator it = cleared_waypoints.begin();
        it != cleared_waypoints.end(); it++)
    {
      swarmie_msgs::Waypoint wpt;
      wpt.action = swarmie_msgs::Waypoint::ACTION_REACHED;
      wpt.id = *it;
      waypointFeedbackPublisher.publish(wpt);
    }
    result = logicController.DoWork();	//ask logic controller to run
    if(result.type != behavior || result.b != wait)
    {
      // if the logic controller requested that the robot drive, then
      // drive. Otherwise there are no manual waypoints and the robot
      // should sit idle. (ie. only drive according to joystick
      // input).
      sendDriveCommand(result.pd.left,result.pd.right);
    }
  }

  // publish state machine string for user, only if it has changed, though
  if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0)
  {
    stateMachinePublish.publish(stateMachineMsg);
    sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
  }
}

void sendDriveCommand(double left, double right)
{
  velocity.linear.x = left,
      velocity.angular.z = right;

  // publish the drive commands
  driveControlPublish.publish(velocity);
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

  // Don't pass April tag data to the logic controller if the robot is not in autonomous mode.
  // This is to make sure autonomous behaviours are not triggered while the rover is in manual mode.
  if(currentMode == 0 || currentMode == 1)
  {
    return;
  }

  if (message->detections.size() > 0) {
    vector<Tag> tags;

    for (int i = 0; i < message->detections.size(); i++) {

      // Package up the ROS AprilTag data into our own type that does not rely on ROS.
      Tag loc;
      loc.setID( message->detections[i].id );

      // Pass the position of the AprilTag
      geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
      loc.setPosition( make_tuple( tagPose.pose.position.x,
				   tagPose.pose.position.y,
				   tagPose.pose.position.z ) );

      // Pass the orientation of the AprilTag
      loc.setOrientation( ::boost::math::quaternion<float>( tagPose.pose.orientation.x,
							    tagPose.pose.orientation.y,
							    tagPose.pose.orientation.z,
							    tagPose.pose.orientation.w ) );
      tags.push_back(loc);

    }

    logicController.SetAprilTags(tags);
    
    //TODO AJH added code here to send swarmie msg when more than 3 resources are found
    //3 is a magic number - it's a good cutoff, because sometimes a swarmie sees one 
    //resource & counts up to 3 tags
    //& also if it's not home tags that it's detecting (only resources)
    //and I'm glad I caught this now, otherwise we'd just keep calling a swarmie 
    //& telling it about home, which is obvs not useful 

    //check if we're near center, & if we're not, check tags.size()
    if (hypot(currentLocationMap.x-centerLocationMap.x, currentLocationMap.y-centerLocationMap.y) < 0.750){
      //if we're not at home, check if we're seeing more than three resources 
      if(tags.size()>3 && message->detections[0].id == 0){
        //giv ourselves another minute on our task timer
        nextTask += 1;
        //0 == resource message within the handler MyMessageHandler
        my_msg.action = 0;
        //set position x & y vals to our current location (where we see resources)
        my_msg.x = currentLocationMap.x;
        my_msg.y = currentLocationMap.y;
        //send resource messa
        if(comms.size()<4){
          //send resource message to the gatherer swarmie
          comms.at(0).publish(my_msg);
        }
        if(comms.size()>4){
          //switch off sending to gatherer 1 & hybrid
          //for every third message?
          if(switchSwarmie%2 == 0){
            //publish every other message to gatherer #2
            comms.at(3).publish(my_msg);
          }
          else{
            comms.at(0).publish(my_msg);
          }
          switchSwarmie++;
        }
      }
    }

  }

}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
  currentMode = message->data;
  if(currentMode == 2 || currentMode == 3) {
    logicController.SetModeAuto();
  }
  else {
    logicController.SetModeManual();
  }
  sendDriveCommand(0.0, 0.0);
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {

  logicController.SetSonarData(sonarLeft->range, sonarCenter->range, sonarRight->range);

}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message){
  //Get (x,y) location directly from pose
  currentLocation.x = message->pose.pose.position.x;
  currentLocation.y = message->pose.pose.position.y;

  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentLocation.theta = yaw;

  linearVelocity = message->twist.twist.linear.x;
  angularVelocity = message->twist.twist.angular.z;


  Point currentLoc;
  currentLoc.x = currentLocation.x;
  currentLoc.y = currentLocation.y;
  currentLoc.theta = currentLocation.theta;
  logicController.SetPositionData(currentLoc);
  logicController.SetVelocityData(linearVelocity, angularVelocity);
}

// Allows a virtual fence to be defined and enabled or disabled through ROS
void virtualFenceHandler(const std_msgs::Float32MultiArray& message)
{
  // Read data from the message array
  // The first element is an integer indicating the shape type
  // 0 = Disable the virtual fence
  // 1 = circle
  // 2 = rectangle
  int shape_type = static_cast<int>(message.data[0]); // Shape type

  if (shape_type == 0)
  {
    logicController.setVirtualFenceOff();
  }
  else
  {
    // Elements 2 and 3 are the x and y coordinates of the range center
    Point center;
    center.x = message.data[1]; // Range center x
    center.y = message.data[2]; // Range center y

    // If the shape type is "circle" then element 4 is the radius, if rectangle then width
    switch ( shape_type )
    {
    case 1: // Circle
    {
      if ( message.data.size() != 4 ) throw ROSAdapterRangeShapeInvalidTypeException("Wrong number of parameters for circle shape type in ROSAdapter.cpp:virtualFenceHandler()");
      float radius = message.data[3];
      logicController.setVirtualFenceOn( new RangeCircle(center, radius) );
      break;
    }
    case 2: // Rectangle
    {
      if ( message.data.size() != 5 ) throw ROSAdapterRangeShapeInvalidTypeException("Wrong number of parameters for rectangle shape type in ROSAdapter.cpp:virtualFenceHandler()");
      float width = message.data[3];
      float height = message.data[4];
      logicController.setVirtualFenceOn( new RangeRectangle(center, width, height) );
      break;
    }
    default:
    { // Unknown shape type specified
      throw ROSAdapterRangeShapeInvalidTypeException("Unknown Shape type in ROSAdapter.cpp:virtualFenceHandler()");
    }
    }
  }
}

void mapHandler(const nav_msgs::Odometry::ConstPtr& message) {
  //Get (x,y) location directly from pose
  currentLocationMap.x = message->pose.pose.position.x;
  currentLocationMap.y = message->pose.pose.position.y;

  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  currentLocationMap.theta = yaw;

  linearVelocity = message->twist.twist.linear.x;
  angularVelocity = message->twist.twist.angular.z;

  Point curr_loc;
  curr_loc.x = currentLocationMap.x;
  curr_loc.y = currentLocationMap.y;
  curr_loc.theta = currentLocation.theta; // was currentLocationMap
  logicController.SetMapPositionData(curr_loc);
  logicController.SetMapVelocityData(linearVelocity, angularVelocity);
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
  const int max_motor_cmd = 255;
  if (currentMode == 0 || currentMode == 1) {
    float linear  = abs(message->axes[4]) >= 0.1 ? message->axes[4]*max_motor_cmd : 0.0;
    float angular = abs(message->axes[3]) >= 0.1 ? message->axes[3]*max_motor_cmd : 0.0;

    float left = linear - angular;
    float right = linear + angular;

    if(left > max_motor_cmd) {
      left = max_motor_cmd;
    }
    else if(left < -max_motor_cmd) {
      left = -max_motor_cmd;
    }

    if(right > max_motor_cmd) {
      right = max_motor_cmd;
    }
    else if(right < -max_motor_cmd) {
      right = -max_motor_cmd;
    }

    sendDriveCommand(left, right);
  }
}


void publishStatusTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "online";
  status_publisher.publish(msg);
}

void manualWaypointHandler(const swarmie_msgs::Waypoint& message) {
  Point wp;
  wp.x = message.x;//message.x;
  wp.y = message.y;//message.y;
  wp.theta = 0.0;
  std_msgs::String temp_msg;
  switch(message.action) {
  case swarmie_msgs::Waypoint::ACTION_ADD:
    logicController.AddManualWaypoint(wp, message.id);
    //temp_msg.data = "Entering manual mode to reach waypoint ";
    //infoLogPublisher.publish(temp_msg);
    //AJH: if we add a manual waypoint, we switch to manual mode
    //logicController.SetModeManual();
    break;
  //case swarmie_msgs::Waypoint::ACTION_REACHED:
    //AJH: if we have reached our waypoint, we switch to auto mode
    //temp_msg.data = "Entering auto mode after reaching waypoint";
    //infoLogPublisher.publish(temp_msg);
    //logicController.SetModeAuto();
    //break;
  case swarmie_msgs::Waypoint::ACTION_REMOVE:
    logicController.RemoveManualWaypoint(message.id);
    break;
  }
}

void sigintEventHandler(int sig) {
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "";
  heartbeatPublisher.publish(msg);
}

long int getROSTimeInMilliSecs()
{
  // Get the current time according to ROS (will be zero for simulated clock until the first time message is recieved).
  ros::Time t = ros::Time::now();

  // Convert from seconds and nanoseconds to milliseconds.
  return t.sec*1e3 + t.nsec/1e6;

}


Point updateCenterLocation()
{
  transformMapCentertoOdom();

  Point tmp;
  tmp.x = centerLocationOdom.x;
  tmp.y = centerLocationOdom.y;

  return tmp;
}

void transformMapCentertoOdom()
{

  // map frame
  geometry_msgs::PoseStamped mapPose;

  // setup msg to represent the center location in map frame
  mapPose.header.stamp = ros::Time::now();

  mapPose.header.frame_id = publishedName + "/map";
  mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
  mapPose.pose.position.x = centerLocationMap.x;
  mapPose.pose.position.y = centerLocationMap.y;
  geometry_msgs::PoseStamped odomPose;
  string x = "";

  try
  { //attempt to get the transform of the center point in map frame to odom frame.
    tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
    tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
  }

  catch(tf::TransformException& ex) {
    ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
    x = "Exception thrown " + (string)ex.what();
    std_msgs::String msg;
    stringstream ss;
    ss << "Exception in mapAverage() " + (string)ex.what();
    msg.data = ss.str();
    infoLogPublisher.publish(msg);
    cout << msg.data << endl;
  }

  // Use the position and orientation provided by the ros transform.
  centerLocationMapRef.x = odomPose.pose.position.x; //set centerLocation in odom frame
  centerLocationMapRef.y = odomPose.pose.position.y;

 // cout << "x ref : "<< centerLocationMapRef.x << " y ref : " << centerLocationMapRef.y << endl;

  float xdiff = centerLocationMapRef.x - centerLocationOdom.x;
  float ydiff = centerLocationMapRef.y - centerLocationOdom.y;

  float diff = hypot(xdiff, ydiff);

  if (diff > drift_tolerance)
  {
    centerLocationOdom.x += xdiff/diff;
    centerLocationOdom.y += ydiff/diff;
  }

  //cout << "center x diff : " << centerLocationMapRef.x - centerLocationOdom.x << " center y diff : " << centerLocationMapRef.y - centerLocationOdom.y << endl;
  //cout << hypot(centerLocationMapRef.x - centerLocationOdom.x, centerLocationMapRef.y - centerLocationOdom.y) << endl;

}

void humanTime() {

  float timeDiff = (getROSTimeInMilliSecs()-startTime)/1e3;
  if (timeDiff >= 60) {
    minutesTime++;
    startTime += 60  * 1e3;
    if (minutesTime >= 60) {
      hoursTime++;
      minutesTime -= 60;
    }
  }
  timeDiff = floor(timeDiff*10)/10;

  double intP, frac;
  frac = modf(timeDiff, &intP);
  timeDiff -= frac;
  frac = round(frac*10);
  if (frac > 9) {
    frac = 0;
  }

  //cout << "System has been Running for :: " << hoursTime << " : hours " << minutesTime << " : minutes " << timeDiff << "." << frac << " : seconds" << endl; //you can remove or comment this out it just gives indication something is happening to the log file
}

void startOrderHandler(const std_msgs::String& msg)		//startOrder
{

  string msg_name = msg.data;
  swarmieNames.push_back(msg.data);
  if(msg_name == publishedName){
    Msg.data = string("That's my published name! I am " + publishedName);
    //infoLogPublisher.publish(Msg);
  }
  else{
    Msg.data = string("Boo, that's not my published name! I'm not " + msg_name + ", I'm " + publishedName);
    //infoLogPublisher.publish(Msg);
  }

}

void CNMFirstBoot()
{
    //FIRST TIME IN THIS FUNCTION
    if(testCount)		//startOrder
    {
      testCount = false;
      /*  std_msgs::String msg;
        msg.data = "first boot running ";
          infoLogPublisher.publish(msg);*/
      msg.data = (publishedName);
	    startOrderPub.publish(msg);
    }
}

void sortOrder()
{
  if(sortTrigger)
  {
    sortTrigger = false;
    //std::string str(ip);
    stringstream name_ss;
    for(int i = 0; i < swarmieNames.size(); i++)
    {
      if(i != 0){name_ss << ",";}
      name_ss << swarmieNames.at(i);
    }
    string allNames = name_ss.str();
    msg.data = allNames;
    //msg.data = ip;
    //infoLogPublisher.publish(msg);
    sortOrderPub.publish(msg);
    //msg.data = "sortTrigger is running ";
    //infoLogPublisher.publish(msg);
  }
}

void sortOrderHandler(const std_msgs::String& msg)
{
   //Hijacking this event handler for my own nefarious purposes
   //in theory, we only get one message from each swarmie on boot up
   //and in this case, that should just be our all the hostnames we have received thus far
   //so for this event handler, we just want to compare the received list of hostnames & see if it is
   //larger than our list. if it is, copy the received list and discard ours.
   //after that, we'll sort the list so we have an ordered list of swarmies

  //splitting our message stream:
  string allNames = msg.data;
  vector<string> results;

  stringstream ss(allNames);
  string item;
  char delim = ',';
  while (getline(ss, item, delim)) {
      results.push_back(item);
  }

   //std_msgs::String msgList;
   //stringstream ffs;
   //ffs << publishedName <<"'s list has " << swarmieNames.size() << " elements, rcv'd list has " << results.size();

   if(results.size() >= swarmieNames.size()){
     swarmieNames = results;
   }
   //msgList.data = ffs.str();
   //infoLogPublisher.publish(msgList);

   //AJH this might be breaking the whole damn thing eta: egads, I was right!!! 
   //but only because I don't know the difference between i & j
   //this will sort our list every time we add a name, so for now it's okay,
   //but it will not work in the case that a swarmie enters the arena part-way
   //thru the competition
   for(int i = 0; i < swarmieNames.size(); i++){
      //this loops through the entire list i times, which I think is
      //sufficient to sort the whole thing?
      //should be i^2 comparisons,
      //but maybe double check the math at some point? questions? ...bueller?
      for(int j = 0; j < swarmieNames.size()-1; j++){
      //if swarmieNames[j] is 'bigger' than swarmieNames[j+1], switch
        if(swarmieNames.at(j) > swarmieNames.at(j+1)){
          string temp = swarmieNames.at(j);
          swarmieNames.at(j) = swarmieNames.at(j+1);
          swarmieNames.at(j+1) = temp;
       }
     }
   }
} 

void myMessageHandler(const swarmie_msgs::Waypoint& my_msg)
{
  stringstream rcvd;

  msg.data = rcvd.str();
  infoLogPublisher.publish(msg);
  //AJH: do stuff
  int msg_type = my_msg.action; //static_cast<int>(my_msg.data[0]); // Shape type  
  if (msg_type == swarmie_msgs::Waypoint::ACTION_REACHED)
  {
    //do something?
  }
  else
  {
    // Elements 2 and 3 are the x and y coordinates of the range center
    Point center;
    center.x = my_msg.x; // Range center x
    center.y = my_msg.y; // Range center y

    // If the shape type is "circle" then element 4 is the radius, if rectangle then width
    switch ( my_msg.action )
    {
      case swarmie_msgs::Waypoint::ACTION_ADD: //resource message -> action = 0
      {
        float radius = 1.5;
        //TODO: AJH for now, just see if we can effectively pull usable info out of these messages
        rcvd << "rcv'd resource msg points: (" << my_msg.x << ", " << my_msg.y << ")";
        msg.data = rcvd.str();
        infoLogPublisher.publish(msg);
        logicController.goHelp(center, radius);
        //logicController.
        //logicController.setVirtualFenceOn( new RangeCircle(center, radius) );
        //logicController.setInterconnectedCOntroller thing here
        break;
      }
      case swarmie_msgs::Waypoint::ACTION_REMOVE: //obstacle message -> action = 1
      {
        //build inside-out fence around obstacle
        //logicController.setVirtualFenceOn( new RangeCircle(center, radius) );
        //logicController.setInterconnectedCOntroller thing here
        //KAILY:
        //check our obstacle gridpoints:
        rcvd << "rcv'd obstacle msg points: (" << my_msg.x << ", " << my_msg.y << ")";
        msg.data = rcvd.str();
        infoLogPublisher.publish(msg);
        /*float gridPoints[2][3] = {{1.000, 1.250, 1.500}, {1.000, 1.250, 1.500}};
        int size = *(&gridPoints + 1) - gridPoints;
        for(int i=0; i < size; i++) {
          int nSize = *(&gridPoints[i] + 1) - gridPoints[i];
          for(int j=0; j < nSize; j++) {
            //do stuff with Range...
          }
        }
         */
        //point obstacleLocation = currentLocation + 0.8; // or however you send 0.8 directly in front of your currentLocation
        Point obstacleLoc;
        obstacleLoc.x = center.x;
        obstacleLoc.y = center.y;
        //RangeController::RangeCircle newFence = RangeController::RangeCircle(obstacleLoc, 0.5);
        //myFences.push_back(newFence);
        //logicController.setVirtualFenceOn(newFence);
        //RangeController::RangeCircle(obstacleLoc, 0.5);
        break;
      }
      default:
      { // Unknown msg type specified
        throw ROSAdapterRangeShapeInvalidTypeException("Unknown Shape type in ROSAdapter.cpp:virtualFenceHandler()");
        //throw new Exception("Unknown message type in ROSAdapter.cpp:myMessageHandler()");
      }
  }

  }
}

void CNMProjectCenter()
{
    //NOTES ON THIS FUNCTION:
    //- Takes current point and projects it out to where the center SHOULD be
    //- Calls CNMAVGCenter to avg new center location searchController new avg center

    if(resetMap)
    {
	    resetMap = false;
	    maxedCenterArray = false;
	    centerIndex = 0;
    }

    //NORMALIZE ANGLEf
    double normCurrentAngle = angles::normalize_angle_positive(currentLocationMap.theta);

    //CenterXCoordinates[centerIndex] = currentLocation.x + (CENTEROFFSET * (cos(normCurrentAngle)));
    CenterXCoordinates[centerIndex] = currentLocationMap.x;
    //CenterYCoordinates[centerIndex] = currentLocation.y + (CENTEROFFSET * (sin(normCurrentAngle)));
    CenterYCoordinates[centerIndex] = currentLocationMap.y;

    CNMAVGCenter();
}


void CNMAVGCenter()
{

  if(resetMap)
  {
    resetMap = false;
    maxedCenterArray = false;
    centerIndex = 0;
  }


    std_msgs::String msg;
    msg.data = "Averaging Center Location";
    infoLogPublisher.publish(msg);

    if(centerIndex >= ASIZE)
    {
        if(!maxedCenterArray) {  maxedCenterArray = true; }
        centerIndex = 0;
    }
    else
    {
        centerIndex++;
    }

    float avgX = 0;
    float avgY = 0;

    for(int i = 0; i < ASIZE; i++)
    {
       avgX += CenterXCoordinates[i];
       avgY += CenterYCoordinates[i];
    }

        avgX = (avgX / ASIZE);
        avgY = (avgY / ASIZE);

    //UPDATE CENTER LOCATION
    //---------------------------------------------
    cnmCenterLocation.x = (avgX);
    cnmCenterLocation.y = (avgY);
    logicController.cnmSetCenterLocationMAP(cnmCenterLocation);

    stringstream ss;
    ss << "Center Position X: " <<   cnmCenterLocation.x << " Y: " << cnmCenterLocation.y << "  Index: " << centerIndex << endl;
    msg.data = ss.str();
    infoLogPublisher.publish(msg);

    //AJH now that we have our initial start stuff done,
    //we are ready to start moving
    //isReady = true;

}

void CNMCurrentLocationAVG()
{

  static int index = 0;

  std_msgs::String msg;
  msg.data = "Averaging Current Location";
  infoLogPublisher.publish(msg);



    if(index < CASIZE)
    {

	     avgCurrentCoordsX[index] = currentLocationMap.x;
    	 avgCurrentCoordsY[index] = currentLocationMap.y;

	     index++;

	     //return false;
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

      logicController.cnmSetAvgCurrentLocation(cnmAVGCurrentLocation);


      stringstream ff;
      ff << "Current Postion Average is X: "<< cnmAVGCurrentLocation.x << "  Y: " << cnmAVGCurrentLocation.y << "  Theta: " << currentLocation.theta << endl;
           Msg.data = ff.str();
           infoLogPublisher.publish(Msg);


	    index = 0;
	    //return true;
    }
}

void assignSwarmieRoles(int currentTime){
  myStartTime = currentTime;
  int numSwarmies = swarmieNames.size();
  //comms = array<ros::Publisher,numSwarmies>;
  //build one publisher per swarmie
  for(int i = 0; i < numSwarmies; i++){
    string& name = swarmieNames.at(i);
    //cnm_NH points to mNH, so we dereference it here to build our comms publishers
    namedSwarmiePub = cnm_NH->advertise<swarmie_msgs::Waypoint>(("dear" + name), 100, true);
    comms.push_back(namedSwarmiePub);
    //msg.data = ("I created a publisher for " + name);
    if(name == publishedName)
    {
      myID = i;
      //stringstream temp;
      //temp << "That's me! I created a publisher to myself. My ID is " << myID << "!";
      //msg.data = temp.str();
    }
    //infoLogPublisher.publish(msg);
  }

  //write our roles to our swarmie's memory - that way, if 
  //he drops offline & reboots at some point, he still has an idea of what he was doing & 
  //when he started doing it.
  //system();

  //fire initial behavior starts now
  //assign my role based on myID and swarmie team size:
  double increment = 0.0;
  switch(myID){
      //if gather, set initial fence area around home & begin searching/waiting for input there
      //gatherers should be on call for the searchers (if they find a resource, etc.)
      case 0://gather1:
        myRole = Role::gather1;           
        msg.data = ("I am a gatherer! (gather1)"); //My role is: " + myRole.toString());
        infoLogPublisher.publish(msg);
        static const int myBigArray1[8] = {6,7,8,11,12,16,17,18};
        myAreasBig = &myBigArray1[0];
        //TODO: start the octagon (inner loop)
        if(numSwarmies > 4)
        {
          increment = 0.50;
        }
        else
        {
          increment = 1.00;
        }
        //start our first octagon 1.5 m from center
        logicController.startGather(increment, 1.5, cnmCenterLocation);
        break;  
      //searchers should get their initial grid areas, which are calculated based on team size and 
      //divided based on location
      //see map on slack 
      case 3://gather2
      {
        myRole = Role::gather2;
        msg.data = ("I am a gatherer! (gather2)");
        infoLogPublisher.publish(msg);
        static const int myBigArray2[8] = {6,7,8,11,12,16,17,18};
        myAreasBig = &myBigArray2[0];
        //TODO: start the octagon (outer loop)
        increment = 1.00;
        //start our first octagon 1.5 m from center
        logicController.startGather(increment, 2.0, cnmCenterLocation);
        break;
      }  
      case 1://searcher1:
        myRole = Role::searcher1;
        msg.data = ("I am a searcher! (searcher1)"); 
        infoLogPublisher.publish(msg);
        //build fence grids & assign subsets of grid:
        // for the grid fence
        if(numSwarmies < 4)
        {
          static const int myBigArray3[8] = {1,0,5,10,15,20,21,22};
          myAreasBig = &myBigArray3[0];
          taskTime = 1.75;
          //logicController.changeAreas(mapAreas[myAreasBig[0]],  0.75);//1.5);
          //logicController.changeAreas(mapAreas[searchStart],  1.5);
          logicController.startGather(1.0, 1.0, mapAreas[searchStart]); 
          cout << "MAPPING - current search's center location: " <<  mapAreas[searchStart].x << ", " << mapAreas[searchStart].y << endl;

        }
        else
        {
          static const int mySmallArray1[4] = {1,0,5,10};
          myAreasSmall = &mySmallArray1[0];
          taskTime = 3.50;
          //logicController.changeAreas(mapAreas[myAreasSmall[0]], 1.1);//2.2);
          //logicController.changeAreas(mapAreas[searchStart], 2.2);
          logicController.startGather(1.0, 1.0, mapAreas[searchStart]); 

          cout << "MAPPING - current search's center location: " <<  mapAreas[searchStart].x << ", " << mapAreas[searchStart].y << endl;

        }

        break;
      case 2://searcher2:
        myRole = Role::searcher2;
        msg.data = ("I am a searcher! (searcher2)"); 
        infoLogPublisher.publish(msg);
        //build fence grids & assign subsets of grid:
        // for the grid fence
        if(numSwarmies < 4)
        {
          static const int myBigArray4[8] = {2,3,4,9,14,19,24,23};
          myAreasBig = &myBigArray4[0];
          //logicController.changeAreas(mapAreas[myAreasBig[0]], 0.75);//1.5);
          //logicController.changeAreas(mapAreas[searchStart], 1.5);
          logicController.startGather(1.0, 1.0, mapAreas[searchStart]);
          taskTime = 1.75;
          cout << "MAPPING - current search's center location: " <<  mapAreas[searchStart].x << ", " << mapAreas[searchStart].y << endl;

        }
        else
        {
          static const int mySmallArray2[4] = {2,3,4,9};
          myAreasSmall = &mySmallArray2[0];
          taskTime = 3.50;
          //logicController.changeAreas(mapAreas[myAreasSmall[0]], 1.1);//2.2);
          //logicController.changeAreas(mapAreas[searchStart], 2.2);
          logicController.startGather(1.0, 1.0, mapAreas[searchStart]);
          cout << "MAPPING - current search's center location: " <<  mapAreas[searchStart].x << ", " << mapAreas[searchStart].y << endl;

        }

        break;
      case 4://searcher3:
        myRole = Role::searcher3;
        msg.data = ("I am a searcher! (searcher3)"); 
        infoLogPublisher.publish(msg);
        static const int mySmallArray3[4] = {15,20,21,22};
        myAreasSmall = &mySmallArray3[0];
        taskTime = 3.50;
        //logicController.changeAreas(mapAreas[myAreasSmall[0]], 1.1);//2.2);
        //logicController.changeAreas(mapAreas[searchStart], 2.2);
        logicController.startGather(1.0, 1.0, mapAreas[searchStart]);
        cout << "MAPPING - current search's center location: " <<  mapAreas[searchStart].x << ", " << mapAreas[searchStart].y << endl;


        break;
      case 5://searcher4
        myRole = Role::searcher4;
        msg.data = ("I am a searcher! (searcher4)");
        infoLogPublisher.publish(msg);
        static const int mySmallArray4[4] = {14,19,24,23};
        myAreasSmall = &mySmallArray4[0];
        taskTime = 3.50;
        //logicController.changeAreas(mapAreas[myAreasSmall[0]], 1.1);//2.2);
        //logicController.changeAreas(mapAreas[searchStart], 2.2);
        logicController.startGather(1.0, 1.0, mapAreas[searchStart]);
        cout << "MAPPING - current search's center location: " <<  mapAreas[searchStart].x << ", " << mapAreas[searchStart].y << endl;

        break;
      default:
        msg.data = ("I don't recognize that role, sorry!");
        infoLogPublisher.publish(msg);
        //swarmie > 6 defaults to search?
        break;
    }
    nextTask = 0;
    nextTask = taskTime+(currentTime/60);
    cout << "MAPPING - starting search, next task schedule for " << nextTask << endl;
    return;
}

void updateBehavior(int currentTime, int update){
    //update behavior roles for searchers and hybrids,
    //update gatherer behavior as necessary (if timer > 15 minutes-startTime)
    //AJH TODO: timer needs to be in seconds, because that is what we are passing it
    msg.data = ("MAPPING - Updating role - the current time is: " + currentTime);
    infoLogPublisher.publish(msg);
    Point next;
    switch(myRole){
      //if gather, set initial fence area around home & begin searching/waiting for input there
      //gatherers should be on call for the searchers (if they find a resource, etc.)
      case Role::gather1: 
      case Role::gather2:
        break;
      //searchers should get their initial grid areas, which are calculated based on team size and 
      //divided based on location
      //see map on slack 
      case Role::searcher1:
      case Role::searcher2:
        if(swarmieNames.size()<4){
          next = mapAreas[myAreasBig[update]];
          cout << "MAPPING - changing to new point " << next.x << ", " << next.y << endl;
          logicController.changeAreas(next, 0.75);//1.5);
        }
        else{
          next = mapAreas[myAreasSmall[update]];
          cout << "MAPPING - changing to new point " << next.x << ", " << next.y << endl;
          logicController.changeAreas(next, 1.1);//2.2);
        }
        break;
      case Role::searcher3:
      case Role::searcher4:
        next = mapAreas[myAreasSmall[update]];
        cout << "MAPPING - changing to new point " << next.x << ", " << next.y << endl;
        logicController.changeAreas(next, 1.1);//2.2);
        break;
      //hybrids will search areas and be on call for other swarmies' resource calls.
      //they search a smaller subset of areas than regular searchers receive
      default:
        //swarmie > 6 defaults to search?
        break;
    }

    return;
}

void testStuff(){
  //std_msgs::Float32MultiArray temp_msg;
  int type = 1;
  float x = 73.000;
  float y = 91.00;
  my_msg.action = type;
  my_msg.x = x;
  my_msg.y = y;
  for(int i = 0; i < comms.size(); i++){
    //ros::Publisher& pub = comms.at(i);
    //pub.publish(my_msg); 
    comms.at(i).publish(my_msg);
    //msg.data = "testing publishers...";
    //infoLogPublisher.publish(msg);
  }
}

void buildMap()
{
  //find our closest corner
  //in our map using searchStart as the index within our gridpoint array
  if (currentLocation.theta + M_PI <= angles::from_degrees(20) && currentLocation.theta + M_PI > angles::from_degrees(320))
  {
      searchStart = 14;
  }
  else if (currentLocation.theta  + M_PI  <= angles::from_degrees(50))
  {
      searchStart = 4;
  }
  else if (currentLocation.theta  + M_PI  <= angles::from_degrees(110))
  {
      searchStart = 2;
  }
  else if (currentLocation.theta  + M_PI <= angles::from_degrees(140))
  {
      searchStart = 0;
  }
    else if (currentLocation.theta  + M_PI  <= angles::from_degrees(200))
  {
      searchStart = 10;
  }
  else if (currentLocation.theta  + M_PI  <= angles::from_degrees(230))
  {
      searchStart = 20;
  }
  else if (currentLocation.theta  + M_PI <= angles::from_degrees(290))
  {
      searchStart = 22;
  }
  else if (currentLocation.theta  + M_PI <= angles::from_degrees(320))
  {
      searchStart = 24;
  }
  cout << "MAPPING - search start is "<< searchStart << endl;
  
  //offset depends on swarmie team size
  double y_offset = 0;
  double y_start = 0;
  double x_offset = 0;
  double x_start = 0;
  if(swarmieNames.size() > 4)
  {
    y_offset = 22/5;
    y_start = 22/2.5;
    x_offset = 22/5;
    x_start = -22/2.5;
  }
  else
  {
    y_offset = 3;
    y_start = 6;
    x_offset = 3;
    x_start = -6;
  }
  //this value is hardcoded because we always subdivide our areas into 
  //5x5 grids, for reasons thank-you-very-much.
  //I'd ask that you not continue to question my judgement on this matter  
  for(int i = 0; i < 5; i++){
    Point temp;
    temp.y = cnmCenterLocation.y + y_start;
    //decrement y offset in our outer loop each time we fire 
    //reset our x_offset each time we loop through
    int x = x_start;
    for(int k = 0; k < 5; k++){
      //TODO AJH centerLocation, or centerLocationMap?
      temp.x = cnmCenterLocation.x + x;
      temp.theta = atan(temp.y/temp.x);
      int index = i*5 + k;
      cout << "MAPPING - gridpoint # "<< index <<" is " << temp.x << ", " << temp.y << endl;
      mapAreas[index] = temp;
      //increment x value each time we go through inner loop
      //& outer loop will reset value each time this loop finishes
      x += x_offset;
    }
    y_start = y_start - y_offset;
  }
}

Point removeOutliers(int data_types, Point gps_points[], int data_points) {
  // print
  cout << "Outlier - Averaging Points" << endl;
  
  // return array
  Point mean_new; // Averaged point returned as the centerLocation.x & *.y
  
  switch(data_types) {
    case 3: // theta std_dev
    {     
      // variables for summed data_points
      float sum_data = 0;
      float sum_data_squared = 0;
      float n = (float)data_points;
      
      // find sum_data & sum_data_squared
      for (int i = 0; i < data_points; i++) {
        sum_data += gps_points[i].theta;
        sum_data_squared += pow(gps_points[i].theta,2);
      }
      
      float mean_theta = sum_data/n;
      float std_dev = pow((sum_data_squared-(pow(sum_data,2)/n))/n,0.5);
      float sum_new = 0;
      
      // ignore if abs(theta) +- 1.5 std_dev
      // calculates ranges
      float small = mean_theta - (1.5*std_dev);
      float big = mean_theta + (1.5*std_dev);
      
      // finds new sums without ignored values
      for (int i = 0; i < data_points; i++) {
        if ((gps_points[i].theta > small)&&(gps_points[i].theta < big)) {
          sum_new += gps_points[i].theta;
        } else {
          n -= 1;
        }
      }
      
      // result
      mean_new.theta = sum_new/n;
      cout << "Averaged theta: " << mean_new.theta << endl;
      
    } 
    case 2: // x,y std_dev
    {
      // variables for summed data_points
      float sum_data_x = 0;
      float sum_data_y = 0;
      float sum_data_squared_x = 0;
      float sum_data_squared_y = 0;
      //TODO removed float cast AJH
      int n_x = data_points;
      int n_y = data_points;
      
      // find sum_data & sum_data_squared
      for (int i = 0; i < data_points; i++) {
        // x
        sum_data_x += gps_points[i].x;
        sum_data_squared_x += pow(gps_points[i].x,2);
        
        // y
        sum_data_y += gps_points[i].y;
        cout << "OUTLIER - y val " << gps_points[i].y << endl;
        sum_data_squared_y += pow(gps_points[i].y,2);
      }
      
      // x values
      float mean_x = sum_data_x/n_x;
      float std_dev_x = pow((sum_data_squared_x-(pow(sum_data_x,2)/n_x))/n_x,0.5);
      float sum_new_x = 0;
      
      // y values
      float mean_y = sum_data_y/n_y;
      cout << "OUTLIER - mean y " << mean_y << endl;
      float std_dev_y = pow((sum_data_squared_y-(pow(sum_data_y,2)/n_y))/n_y,0.5);
      cout << "OUTLIER - std dev y " << std_dev_y << endl;
      float sum_new_y = 0;
      
      // ignore if abs(theta) +- 1.5 std_dev
      // calculates ranges
      float small_x = mean_x - (1.5*std_dev_x);
      float big_x = mean_x + (1.5*std_dev_x);
      float small_y = mean_y - (1.5*std_dev_y);
      float big_y = mean_y + (1.5*std_dev_y);
      
      // finds new sums without ignored values
      for (int i = 0; i < data_points; i++) {
        if ((gps_points[i].x > small_x)&&(gps_points[i].x < big_x)) {
          sum_new_x += gps_points[i].x;
        } else {
          n_x--;// 1;
        }
        
        if ((gps_points[i].y > small_y)&&(gps_points[i].y < big_y)) {
          sum_new_y += gps_points[i].y;
        } else {
          n_y--;// 1;
        }
      }
      
      // result
      cout << "OUTLIER - x pts to average " << n_x << endl;
      cout << "OUTLIER - y pts to average " << n_y << endl;
      mean_new.x = sum_data_x/n_x;
      mean_new.y = sum_data_y/n_y;
      cout << "OUTLIER - Averaged (x,y): (" << mean_new.x << "," << mean_new.y << ")" << endl;
      
    }
    
  }

  return mean_new;
  
}
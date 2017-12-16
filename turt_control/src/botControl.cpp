#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "string.h"
#include "nav_msgs/Odometry.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Header.h"

#include <cmath>

using namespace std;

//each array increment covers 10 centimeters
const double binWidth = 0.1;

/*
10 centemeters * ~200 bins is ~20m. Odd # so that there is 1 center index
These values are for a 2D array that stores log odds for each 10cm x 10cm
location of the area. The starting location will NOT be [0,0] but rather
the middle of the array ([100,100] in this case)
*/
const int horMAX = 201;
const int vertMAX = 201;
const int horCent = 100;
const int vertCent = 100;

//This stores the log odds that each 10cm x 10cm gridcell is occupied
double logOdds[horMAX][vertMAX];

//Will sotre below the initial X and Y position (in meters) of the bot, so that 
//the exact center of the map is known
double startX;
double startY;

//A flag that gets set to 1 after the first time the bot's location is obtained
double locFlag=0;

//set at the start of main()
double lZero;
double lFree; 
double lOcc;
double lValue;

//Declaring this here to make it global in scope
ros::Publisher vel_pub;
geometry_msgs::Twist vel_msg;



//This class is responsible for all things related to obtaining the XY position
//and angle (in radians) of the robot
class poseInfo{
public:

  double x,y,z;
  //Store Quat XYZW values to below
  double quats[4];

  //These are all variables used to do calculations when converting from quats
  double yawCalcOne, yawCalcTwo, siny, cosy;

  //This is the final angle (yaw, converted from quaternions)
  double eulerYaw, eulerDegrees;

  void getLoc(nav_msgs::Odometry pose_msgF){
    x = pose_msgF.pose.pose.position.x;
    y = pose_msgF.pose.pose.position.y;
    z = pose_msgF.pose.pose.position.z;
  }

  void getRot(nav_msgs::Odometry pose_msgF){
    //The angle is initially in quaternians. This function first obtains the
    //quaternian values, and then converts it to radians

    quats[0] = pose_msgF.pose.pose.orientation.x;
    quats[1] = pose_msgF.pose.pose.orientation.y;
    quats[2] = pose_msgF.pose.pose.orientation.z;
    quats[3] = pose_msgF.pose.pose.orientation.w;

    //w*z + x*y
    yawCalcOne = (quats[3] * quats[2]) + (quats[0] * quats[1]);
    //y*y + z*z
    yawCalcTwo = (quats[1] * quats[1])+(quats[2]*quats[2]);

    siny = +2.0 * (yawCalcOne);
    cosy = +1.0 - 2.0 * (yawCalcTwo);

    //The below value is the angle (in radians) the bot is facing
    eulerYaw = atan2(siny, cosy);

    //The below value is never actually used, only found for reference
    eulerDegrees = eulerYaw * (180/M_PI);
  }

}MBPose;


//The below class contains everything that initially needs to be set before
//mapping can be accomplished
class mapCreate{
public:

  double angleMin, angleMax, angleIncrement;
  double rangeMin, rangeMax;
  
  //How many different individual lasers there are
  int angCount;

  //The below values all are from the inverse range sensor model
  //algorithm on p.288 of Probabalistic Robotics (Sebastian Thrun)
  double alpha, beta;
  double r, fi;
  int k;

  void setupFunc(const sensor_msgs::LaserScan scan_msgF){
    angleMin = scan_msgF.angle_min;
    angleMax = scan_msgF.angle_max;
    angleIncrement = scan_msgF.angle_increment;
    rangeMin = scan_msgF.range_min;
    rangeMax = scan_msgF.range_max;
    angCount = (int) ((rangeMax-rangeMin) / angleIncrement);

    //There is a chance these values should be tweaked?
    //alpha is the thickness you consider occupied around your scan
    //beta is the angle that each individual laser is responsible for covering
    alpha = binWidth;
    beta = angleIncrement;
  }

}MBMapper;

//Recover a probability from log odds
double recov(double logVal){
  1.0 - (1.0 / ( 1.0+pow(10,logVal) ) );
}


void lasers(const sensor_msgs::LaserScan scan_msg){

  //perform the setup of parameters that must be done before mapping
  MBMapper.setupFunc(scan_msg);

  //Only start this mapping function if location has been obtained at least once
  if(locFlag == 0){
    return;
  }
  
  //These are the X Y values corresponding to the current array indexes.
  //They are NOT related to the X Y coordinates of Turtlebot's current pos.
  double currX;
  double currY;

  //Values used to search for the laser that is closest in angle from turtleBot
  //to the current array square
  double tempMin;
  double currDMin;

  for(int hor = 0; hor<horMAX; hor++){
    for(int vert = 0; vert<vertMAX; vert++){

      lValue = lZero;//to be overwritten later
      
      //Finding the X Y values relating to the current array indexes by figuring
      //out how far the bot has strayed from its starting position
      currX = startX + ( (float)(hor - horCent) * binWidth );
      currY = startY + ( (float)(vert - vertCent) * binWidth );

      //INVERSE RANGE SENSOR MODEL START
      //For this section I shall number each step as according to the table on p.288 of probabalistic robotics (Thrun)
      
      //3:
      MBMapper.r = sqrt( pow(currX-MBPose.x,2) + pow(currY-MBPose.y,2) );
      //4:
      MBMapper.fi = atan2(currY-MBPose.y,currX-MBPose.x) - MBPose.eulerYaw;
      
      //5:
      MBMapper.k = -9999;//range index correxponding to the closest correct ang.
      tempMin = 9999;
      currDMin = 9999;//minimum angle difference found so far
  
      //Find the individual laser closest in angle to the angle from the bot to
      //the current array index locations.
      //Store the particular laser index to MBMapper.k
      for( int i = 0; i<MBMapper.angCount ; i++){
        tempMin = abs(MBMapper.fi - ( MBMapper.angleMin + (MBMapper.angleIncrement * (float)i) ) );
        if(tempMin < currDMin){
          currDMin = tempMin;
          MBMapper.k = i;
        }
    
      }



      //There is a chance that the individual laser we would like to use returns
      //nan. Only change lValue from lZero if the laser is NOT nan.
      if( !std::isnan(scan_msg.ranges[MBMapper.k])  ){

	//6 setup:
	//Right here is found the minimum of either the rangemax value or the [current range found +(alpha/2)] value
	double minResult = -9999;
	if(MBMapper.rangeMax < (scan_msg.ranges[MBMapper.k]+(MBMapper.alpha / 2.0)) ){
	  minResult = MBMapper.rangeMax;
	}
	else{
	  minResult =  scan_msg.ranges[MBMapper.k]+(MBMapper.alpha / 2.0);
	}

	//6 and 7:
	if( (MBMapper.r > minResult) || (currDMin > (MBMapper.beta / 2.0) ) ){
	  lValue = lZero;
	}

	//8 and 9:
	if( (scan_msg.ranges[MBMapper.k] < MBMapper.rangeMax) && ( abs(MBMapper.r - scan_msg.ranges[MBMapper.k]) < (MBMapper.alpha / 2.0)) ) {
	  lValue = lOcc;
	}

	//10 and 11:
	if(MBMapper.r <= scan_msg.ranges[MBMapper.k]){
	  lValue = lFree;
	}

      }//end of nan if() check

      //Update the logOdds array by lValue (which is the change in belief that
      //the gridcell is occupied)
      logOdds[hor][vert] = logOdds[hor][vert] + lValue;


    }//for hor end
  }//for vert end

  //INVERSE RANGE SENSOR MODEL END

}

void locator(const nav_msgs::Odometry pose_msg){

  //Get the current location and rotation of turtlebot(using the poseInfo class)
  MBPose.getLoc(pose_msg);
  MBPose.getRot(pose_msg);

//This will set only the very first X and Y value found to the staring XY values
  if(locFlag == 0){
    startX = MBPose.x;
    startY = MBPose.y;
  }

  //Set this flag so that startX and startY are never overwritten
  locFlag = 1;


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "botControl");

  ros::NodeHandle n;

  lZero = 0.0;//Set lValue to this if you want no change to belief occupied
  lOcc = log(4);//4 times more likely space is occupied than is not (arbitrary)
  lFree = log(1.0/4.0);//4 times more likely unoccupied (arbitrary)


 
  //initialize logodds array to completely uncertain log(0.5/0.5) = 0

  for(int i = 0; i<horMAX; i++){
    for(int j = 0; j<vertMAX; j++){
      logOdds[i][j] = 0.0;
    }
  }

  //2d scan topic
  ros::Subscriber sub1 = n.subscribe("/scan", 10, lasers);

  //True position and rotation topic
  ros::Subscriber sub2 = n.subscribe("/odom", 10, locator);
  
  //Publisher must publish twist values for robot to use
  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    
  //MAP VISUALIZING STARTS HERE

  ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
  ros::Rate loop_rate(10);
	
  //Create a header, populate the fields.
  std_msgs::Header header = std_msgs::Header();
  header.stamp = ros::Time::now();
  header.frame_id = "/world";

  //Create the map meta data
  nav_msgs::MapMetaData metaD = nav_msgs::MapMetaData();
  metaD.map_load_time = ros::Time::now();


  metaD.resolution = binWidth; //each pixel will represent .05 meters
  metaD.width = horMAX; //2400 pixels long , AKA 120 meters //OLD
  metaD.height = vertMAX; //600 pixels tall, AKA 30 meters//OLD
  //metaD.origin will just init to 0, no need to change

  nav_msgs::OccupancyGrid map = nav_msgs::OccupancyGrid();
  map.header = header;
  map.info = metaD;
  //As per http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html
  //The data is in row-major order, starting witht he 0,0 origin.
  //Probabilities are in a range from [0,100], with -1 being 0


  
  int indexer = (horMAX*vertMAX);

  for(int i =0; i < indexer; i++){
    //entire map initialized as unknown
      map.data.push_back(-1);
  }

  //The commented out code right below sets the map to be a gradient (as a test)
  
  // for(int g =0; g < indexer; g++){
  //   tempDoub = ( (double) g / (indexer) ) * 100.0;
  //   map.data[g]= (int) (tempDoub + 0.5); 
  // }


  double tempDoub = 0.0;
  for(int i = 0; i<horMAX; i++){
    for(int j = 0; j<vertMAX; j++){
      tempDoub = recov(logOdds[i][j]) * 100.0;

      map.data[i*horMAX+j]= (int) tempDoub; 
    }
  }

  //MAP VISUALIZING ENDS HERE
    


  //   for(int i = 0; i<xBins; i++){
  //     for(int j = 0; j<yBins; j++){
  // 	tempDoub = recov(logOdds[i][j]) * 100.0;
	
  // 	map.data[i+j*xBins]= (int) tempDoub; 
  //     }
  //   }



  while (ros::ok())
  {

    pub.publish(map);

    vel_msg.linear.x = 1.0;
    vel_msg.angular.z = 1.0;

    //vel_pub.publish(vel_msg);

    ros::spinOnce();


 

  }

  return 0;
}




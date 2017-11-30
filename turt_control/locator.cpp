#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "string.h"

#include <cmath>

//CODE BELOW BELONGS TO UMASS LOWELL

 // definition of normal curve
    float gauss(float x, float mu, float sigma);

    // this experimentally approximates door sensor performance
    float door(float mu, float x);

    // doors are centered at 11m, 18.5m, and 41m
    float p_door(float x);
    
    //
    float p_wall(float x);

//CODE ABOVE BELONGS TO UMASS LOWELL
using namespace std;

//Code written by Jeffrey Conde for COMP 5490 assignment 4

//Declaring this here to make it global in scope
ros::Publisher vel_pub;

//Used to determine if speed is positive or negative. 1 is pos, 0 is negative.
int velocityIsPos;

void histDisplay();

geometry_msgs::Twist vel_msg;

float beliefBar[600];//1st step of belief (only motion model included)

//Intermediate calculations when incl. sensors, before normalizer is applied
float beliefIntermediateXI[600];
float beliefIntermediateNotXI[600];

float beliefFinal[600];//Final belief array (sensor data included, with norm.)

//I will combine bins into fewer "superbins" when displaying histo.
float superBin[60];
int xCount[60]; // histo related

//Will store to this the odds of moving 0-8 bins (3std devs to both sides)
float binMoveOdds[9];

//Convenience temp variable for calculation use
float beliefBarSum;
int testInt;

//CODE BELOW BELONGS TO UMASS LOWELL
    float gauss(float x, float mu, float sigma)
    {
        float exp = 0 - (std::pow((x-mu), 2) / (2 * sigma * sigma));
        return (1/(sigma * std::sqrt(2*M_PI))) * (std::pow(M_E, exp));
    }

    float door(float mu, float x)
    {
        float sigma = 0.75;
        float peak = gauss(0, 0, sigma);   
        return 0.8 * gauss(x, mu, sigma)/peak;
    }

    float p_door(float x)
    {
        return 0.1 + door(11, x) + door(18.5, x) + door(41, x);
    }
    
    float p_wall(float x)
    {
        return 1.0 - p_door(x);
    }
//CODE ABOVE BELONGS TO UMASS LOWELL

void callback(std_msgs::String door_scan_msg)
{

 
  
  
  if(velocityIsPos == 1){
    vel_msg.linear.x = 4.0;
  }
  else{
     vel_msg.linear.x = -4.0;
  }

  vel_pub.publish(vel_msg);


 //Baye's filter start

  //MOTION MODEL
  //Have a gaussian model



    for(int i = 0; i<600; i++){
      beliefBar[i] = 0;
    }

  

    for(int i = 0; i<600 ; i++){
      for(int j = 0; j<9 ; j++){

      
	if(velocityIsPos == 1){
	  if( (i-j) >= 0){
	    beliefBar[i] = beliefBar[i] + (beliefFinal[i-j] * binMoveOdds[j]);

	  }
	}

	else{
	  if( (i+j) < 600){
	    beliefBar[i] = beliefBar[i] + (beliefFinal[i+j] * binMoveOdds[j]);
	  }

	}
    
      }//end for 2


    }//end for 1



   

  
  //Sensor data inclusion start
  
    //Temp value for use later
    beliefBarSum = 0;

    //door_scan_msg.data == "Door"
    if(door_scan_msg.data == "Door"){//start of if door

      for (int i = 0; i<600 ; i++){
	beliefBarSum = beliefBarSum + ( p_door( ( (float)i * 0.1)+0.05 ) * beliefBar[i] );
      }

    
      for(int i = 0; i<600; i++){

	//Part 1: belief bot is at xi (without normalizer)
	//0.05 is for the center of the grid square. 0.1* is to conv to m
	beliefIntermediateXI[i] = p_door( ( (float)i * 0.1) + 0.05 ) * beliefBar[i];

	//Part 2: belief not at xi (odds of being anywhere else)
	beliefIntermediateNotXI[i] = beliefBarSum - p_door( ( (float)i * 0.1) + 0.05 );

	float  normParam = 0;
	normParam = 1.0 / (beliefIntermediateXI[i] + beliefIntermediateNotXI[i] );

	beliefFinal[i] = beliefIntermediateXI[i] * normParam;
    
      }

    }//end of if door

    else if(door_scan_msg.data == "Wall"){//start of if wall


      for (int i = 0; i<600 ; i++){
	beliefBarSum = beliefBarSum + ( p_wall( ( (float)i * 0.1) + 0.05 ) * beliefBar[i] );
      }

    
      for(int i = 0; i<600; i++){

	//Part 1: belief bot is at xi (without normalizer)
	//0.05 is for the center of the grid square. 0.1* is to conv to m
	beliefIntermediateXI[i] = p_wall( ( (float)i * 0.1) + 0.05 ) * beliefBar[i];

	//Part 2: belief not at xi (odds of being anywhere else)
	beliefIntermediateNotXI[i] = beliefBarSum - p_wall( ( (float)i * 0.1) + 0.05 );

	float  normParam = 0;
	normParam = 1.0 / (beliefIntermediateXI[i] + beliefIntermediateNotXI[i] );

	beliefFinal[i] = beliefIntermediateXI[i] * normParam;
    
      }



    }//end of if wall

    //sensor inclusion end

    
    

  //Baye's filter end


      histDisplay();
      testInt++;


}

void histDisplay(){

  int superOffset = 0;

  for(int i = 0; i<60; i++){
    superBin[i] = 0;
  }

  //Since I cannot display 600 bins in terminal, I shall condense to 60
  for(int superIndex = 0; superIndex < 60; superIndex++){
    for(int belOffset = 0; belOffset < 10; belOffset++){
      superOffset = superIndex*10;
      //The next line makes superBin[i] be a sum of 10 belief values
      superBin[superIndex] = beliefFinal[superOffset + belOffset] + superBin[superIndex];
    }
  }



  float tempXCount = 0;

  //Global array. Will be used to cast float tempXCounts into ints
  for (int i =0; i<60; i++){
    xCount[i] = 0;
  }
  
  //How many 'x's will be displayed per column is found here
  for(int i = 0; i<60; i++){
    tempXCount = superBin[i] / 0.05;
    xCount[i] = int(tempXCount + 0.5);
  }

  //Finally I will display 'x' and ' ' onscreen to display a histogram
  for(int row = 0; row <20 ; row++){
    for(int col = 0; col<60 ; col++){
      if( row > (19 - xCount[col] ) ){
	printf("x");
      }
      else{
	printf(" ");
      }

    }

    printf("\n");

  }

  for(int k = 0; k<= 60 ; k++){
    if( (k % 5 == 0) || k == 0 ){
      printf("%u",k);
    }
    else if(k < 10){
      printf(" ");
    }
    //Arbitrarily choose not to display 1 of the spaces after any double digit #
    else if( k % 5 != 1){
      printf(" ");
    }

  }

  printf("\n");

  //printing an 9 character long message
  printf("Door pos:");
  
  //Will start this loop @ index 9 (1 character after above msg)
  for (int i = 9;i<=41;i++){
    if(i==11){
      printf("1");
    }
    else if(i==19){
      printf("2");
    }

    else if(i==41){
      printf("3");
    }
    else{
      printf(" ");
    }

  }

  printf("\n");
  printf("X axis units are meters. Each X is worth 0.05 odds\n");

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "locator");

  //SET THE DIRECTION HERE. 1 leads to 4m/s, 0 leads to -4m/s
  velocityIsPos = 1;

  //Start with maximum uncertainty. Fill other arrays w/ empty values
  for(int i = 0; i<600 ; i++){
    beliefBar[i] = 0;
    beliefIntermediateXI[i] = 0;
    beliefIntermediateNotXI[i] = 0;
    beliefFinal[i] = 1.0/600.0;
  }

 


  testInt = 0;
  beliefBarSum = 0;

  float stdDevVel;
  stdDevVel = 4.0/3.0;

  for (int i = 0; i<9 ; i++){
    binMoveOdds[i] = gauss((float)i, 4.0, stdDevVel );
  }

  //Initializing values to prevent possible program crash
    vel_msg.linear.x = 4.0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

  ros::NodeHandle n;

  
   //Subscriber must get the door scan string
  ros::Subscriber sub = n.subscribe("/robot/wall_door_sensor", 1000, callback);

  //Publisher must publish twist values for robot to use
  vel_pub = n.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1000);



  

  while (ros::ok())
  {

    ros::spin();
 

  }


  return 0;
}



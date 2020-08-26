Program Code

//Include Required packages and nodes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <cstdlib>
#include "nav_msgs/Odometry.h"
#include <fstream>
using namespace std;

class Stopper {
public:
   //Rosbot Speed Parameters
   constexpr const static double FORWARD_SPEED = 0.3; //ROSBot Constant Velocity
   constexpr const static double TURN_SPEED = 0.3; //ROSBot Constant Turning Speed
   constexpr const static double TURN_STRENGTH_1 = -0.5; //ROSBot Turn Strength Values
   constexpr const static double TURN_STRENGTH_2 = -0.24;
   constexpr const static double TURN_STRENGTH_3 = -0.28;
   constexpr const static double FORWARD_SPEED_STOP = 0; //ROSBot Speed 0 to Stop
   constexpr const static double TURN_LEFT_SPEED_HIGH = 0.5; //Faster Turn Speed Variables
   constexpr const static double TURN_RIGHT_SPEED_HIGH = -2.4;
   //Rosbot Code Parameters
   constexpr const static double POS_OFFSET = 0.3; //Positional offset for correct map/trajectory
   constexpr const static double RIGHT_CHECK = 0.1; //Check ROSBot is on Start Position Variable
   constexpr const static double VELOCITY_CHECK = 0.001; //Check the ROSBot is moving
   constexpr const static double FINAL_DISTANCE = 0.39; //Prevent Collision on Charger
   constexpr const static double COLLISION = 0.2; //Prevent Collision during ROSBot runtime
   constexpr const static double FINAL_POSITION = 0.01; //Stop ROSBot on Charger
   constexpr const static double RDR_GAP_1 = 0.64; //RightDiagonalRange Check 1 First Gap in Range
   constexpr const static double RDR_GAP_2 = 0.7; //RightDiagonalRange Check 2 Second Gap in Range
   constexpr const static double LEFT_GAP = 0.2; //LeftRange for Clearance of Second Gap
   constexpr const static double HALF_MAP_Y = -1; //Through First Gap, Begin Looking for Second
   Stopper();
   void startMoving();
   void moveForward(double forwardSpeed);
   void moveStop();   
   void moveRight(double turn_right_speed = TURN_RIGHT_SPEED_HIGH);
   void moveLeft(double turn_left_speed = TURN_LEFT_SPEED_HIGH);
   void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
   void rotateAndMove(double angle, double turnSpeed);
   void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);

private:
   ros::NodeHandle    node; //Subscribe and publish to nodes and topics
   ros::Publisher commandPub;
   ros::Subscriber laserSub;
   ros::Subscriber odomSub;
   int state = 0; //Defined ROSBot variables
   double frontRange, leftRange, rightRange, rearRange, leftDiagonalRange, rightDiagonalRange, 
PositionX, PositionY, LinearVelocity, yaw_angle, map_X, map_Y, fr1, fr2, fr3, fr4, lr1, lr2, lr3, lr4, rr1, rr2, rr3, rr4, fr, rr, lr, rdr1, rdr2, rdr3, rdr4;
   ros::Time current_time;
   ros::Time new_time;
   ros::Duration real_time;
};

Stopper::Stopper(){
   //Advertise a new publisher for the simulated robot's velocity command topic at 10Hz
   commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
   laserSub = node.subscribe("scan",60,&Stopper::scanCallback, this);
   odomSub = node.subscribe("odom", 20, &Stopper::odomCallback, this);
}

struct Quaternion {double x, y, z, w;};

struct EulerAngles {double roll, pitch, yaw;};

EulerAngles ToEulerAngles(Quaternion q) {
   EulerAngles angles;
   // roll (x-axis rotation)
   double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
   double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
   angles.roll = atan2(sinr_cosp, cosr_cosp);
   // pitch (y-axis rotation)
   double sinp = +2.0 * (q.w * q.y - q.z * q.x);
   if (fabs(sinp) >= 1)
      angles.pitch = copysign(M_PI / 2, sinp); //use 90 degrees if out of range
   else
      angles.pitch = asin(sinp); 
   // yaw (z-axis rotation)
   double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
   double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
   angles.yaw = atan2(siny_cosp, cosy_cosp);
   return angles;
}

//Send a velocity command to Rosbot
void Stopper::moveForward(double forwardSpeed){
   geometry_msgs::Twist msg; 
   msg.linear.x = forwardSpeed;
   commandPub.publish(msg);
}

//Stop the rosbot
void Stopper::moveStop(){
   geometry_msgs::Twist msg; 
   msg.linear.x = FORWARD_SPEED_STOP;
   commandPub.publish(msg);
}

//Move rosbot right in place
void Stopper::moveRight(double turn_right_speed){
   geometry_msgs::Twist msg; 
   msg.angular.z = turn_right_speed;
   commandPub.publish(msg);
}

//Move rosbot left in place
void Stopper::moveLeft(double turn_left_speed){
   geometry_msgs::Twist msg; 
   msg.angular.z = turn_left_speed;
   commandPub.publish(msg);
}

//Rotate and Move robot with defined angle and turnspeed
void Stopper:: rotateAndMove(double angle, double turnSpeed){
   geometry_msgs::Twist msg;
   msg.angular.z = angle;
   msg.linear.x = turnSpeed;  
   commandPub.publish(msg);   
}

void Stopper::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
   //Retrieve the positional x and y data of the rosbot and the velocity
   PositionX = odomMsg->pose.pose.position.x;
   PositionY = odomMsg->pose.pose.position.y;
   LinearVelocity = odomMsg->twist.twist.linear.x;
   Quaternion q;
   q.x = odomMsg->pose.pose.orientation.x;
   q.y = odomMsg->pose.pose.orientation.y;
   q.z = odomMsg->pose.pose.orientation.z;
   q.w = odomMsg->pose.pose.orientation.w;
   ToEulerAngles(q);
   yaw_angle = ToEulerAngles(q).yaw;
}

void Stopper::startMoving(){
   /*Open all files, set timer to 1 and format output files*/
   ofstream TrajectoryFile;
   int timer = 1;
   ofstream VelocityFile;
   ofstream MapFile;
   TrajectoryFile.open("/ufs/servh02/users/nr17076/Desktop/trajectoryXY.csv", ios::trunc);
   VelocityFile.open("/ufs/servh02/users/nr17076/Desktop/velocityTime.csv", ios::trunc);
   MapFile.open("/ufs/servh02/users/nr17076/Desktop/mapFile.csv", ios::trunc);
   TrajectoryFile << "X   Y" << endl;
   VelocityFile << "Time  Velocity" << endl;
   MapFile << "Map X  Map Y" << endl;
   ros::Rate rate(20); /*Rate at which operations are repeated & Loop until user presses Ctrl+C*/
   while (ros::ok()){
      fr = (frontRange + fr1 + fr2 + fr3 + fr4) / 5.0; /*Moving window average of ranges*/
      rr = (rightRange + rr1 + rr2 + rr3 + rr4) / 5.0;
      lr = (leftRange + lr1 + lr2 + lr3 + lr4) / 5.0;          
      if (LinearVelocity < VELOCITY_CHECK){} //If ROSbot is not moving, do nothing
      else { /*Else ROSbot is moving*/
         if (timer == 1){new_time = ros::Time::now(); timer = timer + 1;} /*ROS Timer initialisation*/
         /*Draw the map using the rosbot ranges and write them to MapFile*/
         if (rightRange > 0){
            map_X = rightRange * cos (4.71239 + yaw_angle) + PositionX;
            map_Y = rightRange * sin (4.71239 + yaw_angle) + PositionY;    
            MapFile << (map_X + POS_OFFSET) << "   " << -(map_Y - POS_OFFSET) << endl;    
         } if (leftRange > 0){
            map_X = leftRange * cos (1.5708 + yaw_angle) + PositionX;
            map_Y = leftRange * sin (1.5708 + yaw_angle) + PositionY;  
            MapFile << (map_X + POS_OFFSET) << "   " << -(map_Y - POS_OFFSET) << endl;
         } if (frontRange > 0){
            map_X = frontRange * cos (0 + yaw_angle) + PositionX;
            map_Y = frontRange * sin (0 + yaw_angle) + PositionY;  
            MapFile << (map_X + POS_OFFSET) << "   " << -(map_Y - POS_OFFSET) << endl;
         } if (leftDiagonalRange > 0){
            map_X = leftDiagonalRange * cos (0.785398 + yaw_angle) + PositionX;
            map_Y = leftDiagonalRange * sin (0.785398 + yaw_angle) + PositionY;    
            MapFile << (map_X + POS_OFFSET) << "   " << -(map_Y - POS_OFFSET) << endl;
         } if (rightDiagonalRange > 0){
            map_X = rightDiagonalRange * cos (5.49779 + yaw_angle) + PositionX;
            map_Y = rightDiagonalRange * sin (5.49779 + yaw_angle) + PositionY;    
            MapFile << (map_X + POS_OFFSET) << "   " << -(map_Y - POS_OFFSET) << endl;
         }
         current_time = ros::Time::now(); /*Update time*/
         real_time = current_time - new_time;
         VelocityFile   << real_time << "   " << LinearVelocity  << endl; /*Format files for correct output*/
         TrajectoryFile << (-PositionY + POS_OFFSET); /*Add Offset To Variable for Correct Graph Production*/
         TrajectoryFile << "    " << (PositionX + POS_OFFSET) << endl;
      }
      //Switch case used for each stage of traversing the real environment
      switch(state) {
         case 0:
            if (rightRange < RIGHT_CHECK){/*Do nothing until ROSbot is in start pos*/} else {state = 1;/*ROSbot has loaded and is in starting position goto case 1*/}
            break;
         case 1:
            moveForward(FORWARD_SPEED); /*Move towards first gap*/
            if (rightDiagonalRange < RDR_GAP_1 || rdr1 < RDR_GAP_1 || rdr2 < RDR_GAP_1 || rdr3 < RDR_GAP_1 || rdr4 < RDR_GAP_1){state = 2;} /*If first gap is in range*/
            if (frontRange < COLLISION){moveStop();} /*Checks frontRange for collision*/   
            break;
         case 2:
            rotateAndMove(TURN_STRENGTH_1, TURN_SPEED); /*Rotate through first gap*/
            if (PositionY < HALF_MAP_Y){state = 3;} /*Check Y-axis position go to next state*/
            break;
         case 3: 
            rotateAndMove(TURN_STRENGTH_2, TURN_SPEED); /*Continue turning softer than previous turn*/
            if (rightDiagonalRange < RDR_GAP_2 || rdr1 < RDR_GAP_2 || rdr2 < RDR_GAP_2 || rdr3 < RDR_GAP_2 || rdr4 < RDR_GAP_2 ){state = 4;} /*Check for final post with rightDiagonalRanges, move to next state*/
            break;
         case 4:                      
            rotateAndMove(TURN_STRENGTH_3, TURN_SPEED); /*Continue turning tighter than previous*/
            if ((lr1 < LEFT_GAP || lr2 < LEFT_GAP || lr3 < LEFT_GAP || lr4 < LEFT_GAP || rr1 < LEFT_GAP || rr2 < LEFT_GAP || rr3 < LEFT_GAP || rr4 < LEFT_GAP) || (rightRange < LEFT_GAP || leftRange < LEFT_GAP)){state = 5;} /*Check sideRanges for final gap clearance*/ 
            break;
         case 5:
            moveForward(FORWARD_SPEED); /*Move through the second gap*/
            if (frontRange < FINAL_DISTANCE || PositionX < FINAL_POSITION){/*Check for Charger location*/
               moveStop(); /*Stop on the charger*/
               TrajectoryFile.close(); /*Close Trajectory File*/
               VelocityFile.close(); /*Close Velocity File*/
               MapFile.close(); /*Close Map File*/
               exit (EXIT_SUCCESS); /*Exit Program with Success*/
            }
            break; 
      }
      ros::spinOnce(); /*Allow ROS to process incoming messages and wait until defined time passes*/
      rate.sleep();
   }
}

void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
   /*Retrieve the laser scan data fron each direction of the ROSbot*/
   frontRange = scan->ranges[0]; fr1 = scan->ranges[5]; fr2 = scan->ranges[10]; fr3 = scan->ranges[1435]; fr4 = scan->ranges[1430]; //Front Ranges
   rightRange = scan->ranges[1079]; rr1 = scan->ranges[1084]; rr2 = scan->ranges[1089]; rr3 = scan->ranges[1074]; rr4 = scan->ranges[1069]; //Right Ranges
   leftRange = scan->ranges[359]; lr1 = scan->ranges[364]; lr2 = scan->ranges[369]; lr3 = scan->ranges[354]; lr4 = scan->ranges[349]; rearRange = scan->ranges[719]; //Left Ranges
   rightDiagonalRange = scan->ranges[1259]; rdr1 = scan->ranges[1250]; rdr2 = scan->ranges[1255]; rdr3 = scan->ranges[1265]; rdr4 = scan->ranges[1370]; //Right Diagonal Ranges
}

int main(int argc, char **argv) {
   //Initiate new ROS node named "stopper", create a new stopper object and begin movement
   ros::init(argc, argv, "stopper");   
   Stopper stopper;    
   stopper.startMoving();
   return 0;
}

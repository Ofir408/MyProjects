/*
 * Stopper.cpp
 *
 *  Created on: 22/11/2018
 *      Author: Ofir Ben Shoham.
 */

#include "Stopper.h"
#include "geometry_msgs/Twist.h"

Stopper::Stopper(string robot_name, double forward_speed, double rotation_speed, double min_angle, double max_angle, double step_rotation_constant)
{
	//char *argv[] = {"Stopper", "scan:=scan2", NULL};
	//int argc = sizeof(argv) / sizeof(char*) - 1;

	//char *argv[] = {"stopper", "robot1", NULL};
	//int i = 3; 
	//ros::init(argc, argv, "stopper");

    robotName = robot_name; 
    forwardSpeed = forward_speed;
    rotationSpeed = rotation_speed; 
    MIN_SCAN_ANGLE = min_angle / 180 * M_PI; // to radians
    MAX_SCAN_ANGLE = max_angle / 180 * M_PI; // to radians
    stepRotationConstant = step_rotation_constant;  
    keepMoving = true;  // used to rotate the robot.
    shouldStop = false;  // used to stop the moving of the robot at all. 
	ROS_ERROR_STREAM ("Robot Name is: " << robotName);

    // Advertise a new publisher for the robot's velocity command topic
    commandPub = nh.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);

    // Subscribe to the simulated robot's laser scan topic
    laserSub = nh.subscribe(robot_name + "/scan", 1, &Stopper::scanCallback, this);

}

// Send a velocity command
void Stopper::moveForward() {
    //ROS_INFO("On MoveForward");
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = forwardSpeed;
    commandPub.publish(msg);
}

// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //ROS_INFO("On ScanCallback!");
    bool isObstacleInFront = false;

    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
            isObstacleInFront = true;
            break;
        }
    }

    if (isObstacleInFront) {
        ROS_INFO("Obstacle!");
        keepMoving = false;
        setRotateSide(scan); 
    }
}

// rotate the robot in order to change his way
// to avoid from the obstacles.
void Stopper::rotateTheRobot(double angle, double speed) {
     
     ROS_INFO("starting rotateTheRobot"); 
     double const PieConst = 3.1415926;
     double relativeSpeed = speed*2*PieConst/360;
     double relativeAngle = angle*2*PieConst/360; 
     double currentAngle = 0;
     double t0 = ros::Time::now().toSec();
     double t1; 
     

     geometry_msgs::Twist vel_msg;
     vel_msg.angular.z = relativeSpeed;
     double absVel = vel_msg.angular.z >= 0 ? vel_msg.angular.z:-vel_msg.angular.z;
     std::cout << "speed is: " << speed << " , angular.z is: " << vel_msg.angular.z << " , abs ang.z is:" << absVel << "\n";

     // continue to rotate until we got the required angle.
     while (currentAngle < relativeAngle) { 
     //std::cout << "currentAngle is: " << currentAngle<<"\n";
     //std::cout << "relativeAngle is: " << relativeAngle<<"\n";
         commandPub.publish(vel_msg); // post the vel_msg.
         t1 = ros::Time::now().toSec();
         currentAngle = absVel * (t1 - t0);
     }
	
     ROS_INFO("Done rotateTheRobot"); 
     // sending zero velocity to pause the robot after reaching the angle.
     vel_msg.angular.z = 0;
     commandPub.publish(vel_msg); 
}

// change to 1 for ClockWise OR -1 to notClockWise according the side that the robot should rotates.
void Stopper::setRotateSide(const sensor_msgs::LaserScan::ConstPtr& scan) {
    double maxRange = -10000; // just for default.
    int indexOfMaxRange; 
    double currentRange; 
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        currentRange = scan->ranges[currIndex];
        if (currentRange < maxRange) {
           maxRange = currentRange;
           indexOfMaxRange = currIndex;  
        }
    }

    // check if closer more to max or more to min (max - will rotate clockwise)
    // min - will rotate not-clockwise.
    int middleInx = (minIndex + maxIndex) / 2; // middle index between minIndex & maxIndex, rounding to int.
    int speedSign = indexOfMaxRange > middleInx ? -1:1; // -1 : clockwise, 1 : not clockwise.
    double absSpeed = rotationSpeed >= 0 ? rotationSpeed:-rotationSpeed;
    rotationSpeed = absSpeed * speedSign; 
}

// start moving function. The robot walks until there is an obstacle, 
// then, will rotate.
void Stopper::startMoving() {
    ros::Rate rate(10);
    ROS_INFO("Start moving according cpp code");


    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while (ros::ok() && !this->shouldStop) {
        moveForward();
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        //rate.sleep();
        if (!keepMoving) {
           rotateTheRobot(stepRotationConstant, rotationSpeed); // rotate the robot.
           keepMoving = true;
		}
	}
}


void Stopper::stopRobot() {
	this->shouldStop = true; 
}


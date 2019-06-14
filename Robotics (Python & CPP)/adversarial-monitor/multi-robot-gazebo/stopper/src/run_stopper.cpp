/*
 * run_stopper.cpp
 * Used to control the stopper that moves the robot.
 * This class takes care to control the stopper from "high level", the meaning is 
 * that this class just send him start/stop commands, and doesn't really know his logic.
 * This creates better seperation between them. 
 */

#include "run_stopper.h"

// defs info: 
#define forward_speed  0.5
#define rotation_speed 20.0
#define min_angle  -30.0
#define max_angle 30.0
#define step_rotation_constant 30.0


// called when we want that the robot will start to move
void run_stopper::startMovingRobot() {
	if (this->stopper != NULL) {
		// Start the movement
		this->stopper->startMoving();
	} else {
		throw std::invalid_argument("Stopper Object is NULL");
	}
}


// called when the robot need to stop moving.
// For example, when caught in a threat.
void run_stopper::	stopMovingRobot(const std_msgs::Bool::ConstPtr& msg) {
     bool shouldStop = msg->data;
     if (shouldStop) {
		ROS_INFO("%s", "Stop the movement of the robot...");
		//TODO: this->stopper->stopRobot(); 
     }
}


run_stopper::run_stopper(string robotName) {
	this->robotName = robotName;
	// Create new stopper object
	this->stopper = new Stopper(robotName, forward_speed, rotation_speed, min_angle, max_angle, step_rotation_constant); 
	// subscribe to events from ros_pam pacakge, will be called when the robot caught in a threat. 
	this->subToThreatsEvents = nh.subscribe(this->robotName + "/robot_caught_in_threat", 10, 
															&run_stopper::stopMovingRobot, this);
}

run_stopper::~run_stopper() {
	if (this->stopper != NULL)
		delete this->stopper;
}




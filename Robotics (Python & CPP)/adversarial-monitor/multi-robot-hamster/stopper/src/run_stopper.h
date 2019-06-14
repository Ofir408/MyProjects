/*
 * run_stopper.h
 *
 */

#ifndef STOPPER_SRC_RUN_STOPPER_H_
#define STOPPER_SRC_RUN_STOPPER_H_

#include "ros/ros.h"
#include "Stopper.h"
#include "std_msgs/Bool.h"
#include <stdexcept>

using namespace std; 

class run_stopper {
	
public:
    run_stopper(string robotName);
	~run_stopper(); // destructor.
    void startMovingRobot();
	void stopMovingRobot(const std_msgs::Bool::ConstPtr& msg);
	
private:
		string robotName; 
		ros::NodeHandle nh;
		ros::Subscriber subToThreatsEvents;
		Stopper* stopper = NULL;  
	

};

#endif /* STOPPER_SRC_RUN_STOPPER_H_ */

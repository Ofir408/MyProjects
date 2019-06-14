/*
 * Stopper.h
 *
 */

#ifndef STOPPER_SRC_STOPPER_H_
#define STOPPER_SRC_STOPPER_H_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"

using namespace std; 


class Stopper {
public:

    Stopper(string robot_name, double forward_speed, double rotation_speed, double min_angle, double max_angle, double step_rotation_constant);
    void startMoving();


private:
    string robotName; 
    double MIN_SCAN_ANGLE;
    double MAX_SCAN_ANGLE;
    double forwardSpeed;
    double rotationSpeed; 
    double minAngle;
    double maxAngle;
    double stepRotationConstant; 
    ros::NodeHandle nh;
    const static float MIN_DIST_FROM_OBSTACLE = 0.65; // Should be smaller than sensor_msgs::LaserScan::range_max
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    ros::Subscriber subToThreatsEvents; // Subscriber to the threat events of the robot.
    bool keepMoving; // Indicates whether the robot should continue moving
    bool shouldStop; // used to stop the moving of the robot at all. 
	
    // private function declerations.
    void moveForward();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void rotateTheRobot(double angle, double speed); // rotate the robot.
    void setRotateSide(const sensor_msgs::LaserScan::ConstPtr& scan); // decide to which side to rotate.
    void stopRobot(const std_msgs::Bool::ConstPtr& msg);

    
};

#endif /* STOPPER_SRC_STOPPER_H_ */

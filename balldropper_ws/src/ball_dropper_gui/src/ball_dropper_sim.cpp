/*
 * Author: Christian Laney
 * Date: November 11, 2015
 * Description: Code for simulating the ball dropper system, used for testing GUI
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "ball_dropper/Heartbeat.h"
#include "ball_dropper/Operation.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/thread.hpp"
#include "boost/interprocess/sync/scoped_lock.hpp"

#define OP_IDLE 0
#define OP_OPEN_HATCH 1
#define OP_CLOSE_HATCH 2
#define OP_ROTATE 3
#define OP_DRIVE_ON 4
#define OP_DRIVE_OFF 5
#define OP_INJECT 6
#define OP_RELOAD 7
#define OP_FLUSH 8
#define OP_CALIBRATE 100
#define OP_GET_ERROR_STRING 101
#define OP_CLEAR_ERRORS 102
#define OP_EMG_STOP 255

ros::Time timestamp;
uint8_t opCode;
uint8_t opCodeOfLastAction;
uint8_t remainingInjections = 0;
bool calibrated = false;
bool hatchOpen = false;
bool wheelInPosition = false;
bool driverForward = false;
bool driverBack = false;
bool fireDanger = false;
bool criticalFireDanger = false;
bool rotateMotorOn = false;
bool driveMotorOn = false;
bool injectMotorOn = false;
bool hatchFailure = false;
bool wheelFailure = false;
bool driverFailure = false;
bool injectorFailure = false;
uint32_t actionStartTime;
uint32_t actionDuration;
uint32_t counterStartVal;
uint32_t counterEndVal;
uint16_t instantaneousCurrent;
uint16_t lowestCurrent;
uint16_t highestCurrent;
uint32_t totalCurrent;

double timeOfLastOp = 0;
bool idle = true;

ros::Publisher heartbeatPub;
void publishHeartbeatMsg()
{
    ball_dropper::Heartbeat heartbeatMsg;

    heartbeatMsg.header.stamp = ros::Time::now();
    if(ros::Time::now().toSec() - timeOfLastOp > 1 && !idle) {
        opCodeOfLastAction = opCode;
        opCode = 0;
        idle = true;
    }

    heartbeatMsg.currentOpCode = opCode;
    heartbeatMsg.opCodeOfLastAction = opCodeOfLastAction;
    heartbeatMsg.remainingInjections = remainingInjections;
    heartbeatMsg.calibrated = calibrated ? 1 : 0;
    heartbeatMsg.hatchOpen = hatchOpen ? 1 : 0;
    heartbeatMsg.wheelInPosition = wheelInPosition ? 1 : 0;
    heartbeatMsg.driverForward = driverForward? 1 : 0;
    heartbeatMsg.driverBack = driverBack ? 1 : 0;
    heartbeatMsg.fireDanger = fireDanger ? 1 : 0;
    heartbeatMsg.criticalFireDanger = criticalFireDanger ? 1 : 0;
    heartbeatMsg.hatchFailure = hatchFailure ? 1 : 0;
    heartbeatMsg.wheelFailure = wheelFailure ? 1 : 0;
    heartbeatMsg.driverFailure = driverFailure ? 1 : 0;
    heartbeatMsg.injectorFailure = injectorFailure ? 1 : 0;
    heartbeatMsg.rotateMotorOn = rotateMotorOn ? 1 : 0;
    heartbeatMsg.driveMotorOn = driveMotorOn ? 1 : 0;
    heartbeatMsg.injectMotorOn = injectMotorOn ? 1 : 0;
    heartbeatMsg.actionStartTime = actionStartTime;
    heartbeatMsg.actionDuration = actionDuration;
    heartbeatMsg.counterStartVal = counterStartVal;
    heartbeatMsg.counterEndVal = counterEndVal;
    heartbeatMsg.instantaneousCurrent = instantaneousCurrent;
    heartbeatMsg.lowestCurrent = lowestCurrent;
    heartbeatMsg.highestCurrent = highestCurrent;
    heartbeatMsg.totalCurrent = totalCurrent;

    heartbeatPub.publish(heartbeatMsg);
}

void printStatus()
{
    printf("\nOp Code: %u\n", opCode);
    printf("\nOp Code of Last Action: %u\n", opCode);
    printf("Remaining Injections: %d\n", remainingInjections);
    calibrated ? printf("Calibrated\n") : printf("Uncalibrated\n");
    if (hatchOpen) {
        printf("Hatch Open\n");
    } else {
        printf("Hatch Closed\n");
    }

    wheelInPosition ? printf("Wheel in Position\n") : printf("Wheel not in Position!\n");

    if (driverForward) {
        printf("Driver Forward\n");
    } else if (driverBack) {
        printf("Driver Back\n");
    } else {
        printf("Driver neither Forward nor Back!\n");
    }

    if (criticalFireDanger)
    {
        printf("System failure during injection! Critical Fire Danger!\n");
    }
    else if (fireDanger)
    {
        printf("Ball is going to be on fire.\n");
    }

    if (hatchFailure) {
        printf("Hatch Failure!\n");
    }
    if (wheelFailure) {
        printf("Wheel Failure!\n");
    }
    if (driverFailure) {
        printf("Driver Failure!\n");
    }
    if (injectorFailure) {
        printf("Injector Failure!\n");
    }
    printf("Action Start Time: %u\n", actionStartTime);
    printf("Action Duration: %u\n", actionDuration);
    printf("Counter Start Val: %u\n", counterStartVal);
    printf("Counter End Val: %u\n", counterEndVal);
    printf("Lowest Current: %u\n", lowestCurrent);
    printf("Highest Current: %u\n", highestCurrent);
    printf("Total Current: %u\n", totalCurrent);
}

void genHeartbeat(const ros::TimerEvent& e) {
    publishHeartbeatMsg();
    return;
}


boost::mutex operationMtx;

/*
 * Commands the ball dropper to perform the requested operation
 */
bool operation(ball_dropper::Operation::Request &req,
                ball_dropper::Operation::Response &res)
{
    //Lock the mutex so that we don't try concurrent operations
    //Will be unlocked when the function returns and this object is destroyed
    boost::interprocess::scoped_lock<boost::mutex> slock(operationMtx);

    bool success = true;
    
    ROS_INFO("Op sent: %d", req.opCode);
    opCodeOfLastAction = opCode;
    opCode = req.opCode;
    actionStartTime = (int)(ros::Time::now().toSec()*1000.0);
    actionDuration = 5;

    if(opCode == 1) {
        hatchOpen = true;
    }

    if(opCode == 2) {
        hatchOpen = false;
    }

    if(opCode == 4) {
        driverForward = true;
        driverBack = false;
    }

    if(opCode == 5) {
        driverBack = true;
        driverForward = false;
    }

    if(opCode == 6) {
        remainingInjections--;
    }

    if(opCode == 7) {
        remainingInjections = 30;
    }

    if(opCode == 8) {
        driverBack = true;
        driverForward = false;
        hatchOpen = false;
    }

    if(opCode == 100) {
        calibrated = true;
    }

    timeOfLastOp = ros::Time::now().toSec();
    idle = false;

    return true;
}

//Main
int main(int argc, char **argv)
{
    //Create a node
    ros::init(argc, argv, "ball_dropper_node");
    ros::NodeHandle n;

    ros::Timer heartbeatSim = n.createTimer(ros::Duration(0.1),genHeartbeat);

    heartbeatPub = n.advertise<ball_dropper::Heartbeat>("heartbeat", 100);
    ros::ServiceServer operationService = n.advertiseService("operation", operation);

    //Handle subscriber and service server callbacks in this thread
    ros::spin(); 

    return 0;
}


#ifndef MASTER_ROBOCUP2019_HPP
#define MASTER_ROBOCUP2019_HPP

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>

#include "pid_control_float.h"

//MESSAGE HEADER=================================================
#include "RoboCupGameControlData.h"
#include "robocupController2.h"
#include "pan_tilt_msg.h"
#include "info_kinematics_msg.h"
#include "robocupVision_msg.h"
#include "motion_end.h"
#include "Mt2Serial_msg.h"
#include "imu_msg.h"

//MOTION=========================================================
#define SHOOT 0x01



//ROBOCUP MODE===================================================
#define MODE_TEST                0

#define MODE_BALL_DETECT                11
#define MODE_BALL_STAND                 12
#define MODE_BALL_WALK                  13
#define MODE_KICK_READY                 14
#define MODE_KICK                       15

#define MODE_GOALPOST_DETECT            20
#define MODE_STOP                       999

#define MODE_NO_BALL                            1000

/** inverse kinematics defines
 *
 * */

//ROBOT INFO==============================================================
#define DEFAULT_X       -16
#define DEFAULT_Y       0
#define DEFAULT_Z       102
#define DEFAULT_YAW     0

//foot to ball distance parameter
#define TILT_INIT_POSITION  0.0
#define ROBOT_HEIGHT        505
#define CAMERA_HEIGHT       55


//PARAMETER================================================================
//Goalpost direction
#define LEFT    1
#define RIGHT   2

#define ROBIT               37

#define FIRSTHALF_GOAL      -90
#define SECONDHALF_GOAL     90

typedef struct _GameState
{
    int firstHalf;
    int kickoffTeam;
    int state;
    int readyTime;
    int penalty;

    int stateBefore;

} GameState;

typedef struct _VisionMsg
{
    int Ballx;
    int Bally;
    int BallDist;
    int Yaw;

} VisionMsg;

typedef struct _ImuMsg
{
    // Euler
    float roll;
    float pitch;
    float yaw;

}ImuMsg;

int motion_end = 0;
bool motion_flag = false;


//MESSAGE===================================================================
pan_tilt::pan_tilt_msg ptMsg;
inverse_kinematics::info_kinematics_msg ikMsg;
serial_mcu::Mt2Serial_msg motionMsg;

PID Tracking_pid_pan;
PID Tracking_pid_tilt;
//subscriber
ros::Subscriber gameSub;
ros::Subscriber visionSub;
ros::Subscriber imuSub;
ros::Subscriber motionendSub;
//publisher
ros::Publisher pantiltPub;
ros::Publisher ikPub;
ros::Publisher motionPub;


//FUCTION====================================================================
#define WALK_STOP ikMsg.flag = 0
void WALK_START(double x, double y, double yaw);
void TRACKING_WHAT(int TrackingPoint_x, int TrackingPoint_y, int TrackingThing_x, int TrackingThing_y);

void robocupCallback(const ros::TimerEvent&);
void gameCallback(const gamecontroller::robocupController2::ConstPtr&);
void visionCallback(const robocup2019_vision::robocupVision_msg::ConstPtr&);
void imuCallback(const mw_ahrsv1::imu_msg::ConstPtr&);
void motionCallback(const serial_mcu::motion_end::ConstPtr&);

void DXL_Motion(unsigned char Motion_Num);
int Tracking(double now_x, double X_POINT_STANDARD, double now_y, double Y_POINT_STANDARD);


//TRACKING PARAMETER===========================================================
const int trackingDefaultPointX        = 320;
const int trackingDefaultPointY        = 240;
const int defaultPanMax                = 50;
const int defaultPanMin                = -50;
const int defaultTiltMax               = 0;
const int defaultTiltMin               = -80;
const int defaultScanPanRange          = 30;
const int defaultScanTiltRange         = -30;

// goalpost
const int goalpostTrackingPointX       = 160;
const int goalpostTrackingPointY       = 150;
const int goalpostPanMax               = 120;
const int goalpostPanMin               = -120;
const int goalpostTiltMax              = 40;
const int goalpostTiltMin              = 0;
const int goalpostScanPanRange         = goalpostPanMax;
const int goalpostScanTiltRange        = goalpostTiltMin;

const double goalpostYawingYLengthAcc                   = 0.0;
const double goalpostYawingYLengthAccUnit               = 30.0;
const double goalposGROUND2CAMERAtYawingYLengthLimit    = 0.0;

// ball
const int ballTrackingPointX           = 320;
const int ballTrackingPointY           = 240;
const int ballPanMax                   = 55;
const int ballPanMin                   = -55;
const int ballTiltMax                  = 80;
const int ballTiltMin                  = 0;
const int ballScanPanRange             = ballPanMax;
const int ballScanTiltRange            = ballTiltMin;

const double ballDistanceTiltAngle          = -32;
const double ballPixelXRange                = 8.0;
const double ballPixelYRange                = 6.0;
const double ballDistancePanAngleRange      = 5.0;
const double ballDistanceForShootTiltAngle  = 5.0;

#endif // MASTER_ROBOCUP2019_HPP


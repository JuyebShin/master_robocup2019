#include "../include/master_robocup2019/master_robocup2019.hpp"

using namespace std;

GameState gameInfo;
VisionMsg visionInfo;
ImuState imuInfo;

int Tracking_point_x = trackingDefaultPointX;
int Tracking_point_y = trackingDefaultPointY;
double Tracking_thing_x = trackingDefaultPointX;
double Tracking_thing_y = trackingDefaultPointY;
int Tilt_Max = defaultTiltMax;
int Tilt_Min = defaultTiltMin;
int Pan_Max = defaultPanMax;
int Pan_Min = defaultPanMin;
int scan_pan_range = defaultScanPanRange;
int scan_tilt_range = defaultScanTiltRange;

double PAN_POSITION = 0.0;
double TILT_POSITION = 0.0;

double temp_pan_position = 0.0;
double temp_tilt_position = 0.0;

int robocup_case = MODE_BALL_DETECT;
int temp_case;

int finding_turn = 0;
int walk_term = 0;

int GOALPOST = 0;

bool scan_start = false;
bool local = true;
bool isUDP = true;
bool isPlaying = false;
bool isSet = false;

int YourBallD = 0;
int You_cnt = 0;
int My_cnt = 0;
int DesireBallD_walk = 170;
int DesireBallD_stand = 60;

int motion_what = SHOOT_R;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_robocup2019");
    ros::NodeHandle n;

    double kP = 0.003;//0.035
    double kI = 0.0;
    double kD = 0.001;

    PID_Control_init(&Tracking_pid_pan, kP, kI, kD, 150, 90);
    PID_Control_init(&Tracking_pid_tilt, kP, kI, kD, 150, 90);

    gameSub = n.subscribe("gamecontroller", 100, gameCallback);
    visionSub = n.subscribe("robocup2019_vision", 100, visionCallback);
    imuSub = n.subscribe("imu", 100, imuCallback);
    motionSub = n.subscribe("motion_end", 100, motionCallback);
    udpsub = n.subscribe("udp_order", 100, udpCallback);

    pantiltPub = n.advertise<pan_tilt::pan_tilt_msg>("pantilt", 100);
    ikPub = n.advertise<inverse_kinematics::info_kinematics_msg>("kinematics", 100);
    mtPub = n.advertise<serial_mcu::Mt2Serial_msg>("Motion", 100);

    ros::Timer robocuoTimer = n.createTimer(ros::Duration(0.01), robocupCallback);

    ros::spin();

    return 0;
}

void robocupCallback(const ros::TimerEvent&)
{
    int TRacking_flag_C = Tracking(Tracking_thing_x, Tracking_point_x, Tracking_thing_y, Tracking_point_y);
    static int tracking_cnt = 0;
    static int lost_ball_count = 0;

    if(gameInfo.firstHalf == true)
        GOALPOST = FIRSTHALF_GOAL;
    else
        GOALPOST = SECONDHALF_GOAL;

    /** messages to publish
     * @ pan tilt message
     * @ inverse kinematics message
     * @ motion info message
     * */
    pan_tilt::pan_tilt_msg ptMsg;
    inverse_kinematics::info_kinematics_msg ikMsg;
    serial_mcu::Mt2Serial_msg mtMsg;

    /*For Test*/
//    gameInfo.state = STATE_PLAYING;
//    gameInfo.kickoffTeam = ROBIT;
//    GOALPOST = FIRSTHALF_GOAL;

    if(gameInfo.penalty == HL_PICKUP_OR_INCAPABLE || gameInfo.penalty == HL_SERVICE)
    {
        Tracking_point_x = ballTrackingPointX;
        Tracking_point_y = ballTrackingPointY;
        Tracking_thing_x = ballTrackingPointX;
        Tracking_thing_y = ballTrackingPointY;

        ikMsg.flag = false;

        robocup_case = MODE_BALL_DETECT;
    }
    else
    {
        switch (gameInfo.state)
        {
        case STATE_INITIAL:
            cout << "********************\n";
            cout << "   INITIAL\n";
            cout << "********************\n";

            Tracking_point_x = ballTrackingPointX;
            Tracking_point_y = ballTrackingPointY;
            Tracking_thing_x = 0;
            Tracking_thing_y = 0;

            ikMsg.flag = false;

            break;
        case STATE_READY:
            cout << "********************\n";
            cout << "   READY\n";
            cout << "********************\n";

            static int targetYaw = 0;

            ikMsg.flag = true;

            if(local)
            {
                int tempYaw = visionInfo.yaw - visionInfo.targetYaw;

                if(tempYaw > 180) tempYaw -= 360;
                else if(tempYaw <= -180) tempYaw += 360;

                cout << "tempYaw = " << tempYaw << endl;

//                ikMsg.L_yaw = DEFAULT_YAW + (abs(tempYaw) < 90 ?
//                                                 (abs(tempYaw) > 10 ? tempYaw : 0) :
//                                                 (abs(tempYaw) < 170 ? (tempYaw > 0 ? tempYaw - 170 : tempYaw + 170) : 0));
                ikMsg.L_yaw = DEFAULT_YAW + (abs(tempYaw) > 10 ? tempYaw : 0);
                ikMsg.X_length = DEFAULT_X + (abs(ikMsg.L_yaw) > 10 ? 25 : 50);

                if(visionInfo.ballD > 100) // 10 cm
                {
                    ikMsg.X_length = DEFAULT_X + 50;
                }
                else if(visionInfo.ballD || gameInfo.readyTime < 5) // 20 cm
                {
                    ikMsg.X_length = DEFAULT_X;
                    if(gameInfo.firstHalf)
                    {
                        cout << "right" << endl;
                        targetYaw = FIRSTHALF_GOAL;
                    }
                    else
                    {
                        cout << "left" << endl;
                        targetYaw = SECONDHALF_GOAL;
                    }

                    cout << "target yaw = " << targetYaw
                         << " now yaw = " << visionInfo.yaw << endl;

                    if(abs(visionInfo.yaw) <= 90)
                        ikMsg.L_yaw = DEFAULT_YAW + (visionInfo.yaw - targetYaw);
                    else
                    {
                        if(targetYaw == -90)
                        {
                            if(visionInfo.yaw > targetYaw)
                                ikMsg.L_yaw = DEFAULT_YAW + (targetYaw - visionInfo.yaw);
                            else
                                ikMsg.L_yaw = DEFAULT_YAW + (visionInfo.yaw - targetYaw);
                        }
                        else
                        {
                            if(visionInfo.yaw > targetYaw)
                                ikMsg.L_yaw = DEFAULT_YAW + (visionInfo.yaw - targetYaw);
                            else
                                ikMsg.L_yaw = DEFAULT_YAW + (targetYaw - visionInfo.yaw);
                        }
                    }
                }

                ikMsg.Y_length = DEFAULT_Y;
                ikMsg.Z_length = DEFAULT_Z;
            }
            else
            {
                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = 0;
                Tracking_thing_y = 0;

                ikMsg.X_length = DEFAULT_X + 50;
                ikMsg.Y_length = DEFAULT_Y;
                ikMsg.Z_length = DEFAULT_Z;
                ikMsg.L_yaw = DEFAULT_YAW + (abs(visionInfo.yaw) < 90 ?
                                                 (abs(visionInfo.yaw) > 10 ? visionInfo.yaw : 0) :
                                                 (abs(visionInfo.yaw) < 170 ? (visionInfo.yaw > 0 ? visionInfo.yaw - 170 : visionInfo.yaw + 170) : 0));
                if(isPlaying)
                {
                    cout << "scored" << endl;
                    if(gameInfo.readyTime < 5)
                    {
                        if(gameInfo.kickoffTeam == DROPBALL)
                        {
                            cout << "~~~~~~~~~~ DROPPED BALL ~~~~~~~~~~" << endl;
                        }
                        else if(gameInfo.firstHalf)
                        {
                            if(gameInfo.kickoffTeam == ROBIT)
                                targetYaw = SECONDHALF_GOAL;
                            else
                                targetYaw = FIRSTHALF_GOAL;
                        }
                        else
                        {
                            if(gameInfo.kickoffTeam == ROBIT)
                                targetYaw = FIRSTHALF_GOAL;
                            else
                                targetYaw = SECONDHALF_GOAL;
                        }
                    }
                    else
                    {
                        if(gameInfo.kickoffTeam == DROPBALL)
                        {
                            cout << "~~~~~~~~~~ DROPPED BALL ~~~~~~~~~~" << endl;
                        }
                        else if(gameInfo.firstHalf)
                        {
                            if(gameInfo.kickoffTeam == ROBIT)
                                targetYaw = FIRSTHALF_GOAL;
                            else
                                targetYaw = SECONDHALF_GOAL;
                        }
                        else
                        {
                            if(gameInfo.kickoffTeam == ROBIT)
                                targetYaw = SECONDHALF_GOAL;
                            else
                                targetYaw = FIRSTHALF_GOAL;
                        }
                    }

                    ikMsg.Y_length = DEFAULT_Y;
                    ikMsg.Z_length = DEFAULT_Z;
                    ikMsg.L_yaw = abs(targetYaw - visionInfo.yaw) > 10 ? DEFAULT_YAW - (targetYaw - visionInfo.yaw) : DEFAULT_YAW;

                    ikMsg.X_length = DEFAULT_X + (abs(ikMsg.L_yaw) > 10 ? 25 : 50);
                }
                else
                {
                    ikMsg.X_length = DEFAULT_X + 50;

                    if(gameInfo.readyTime < 5)
                    {
                        ikMsg.X_length = DEFAULT_X;
                        ikMsg.Y_length = DEFAULT_Y;
                        ikMsg.Z_length = DEFAULT_Z;

                        if(gameInfo.firstHalf)
                        {
                            cout << "right" << endl;
                            targetYaw = FIRSTHALF_GOAL;
                        }
                        else
                        {
                            cout << "left" << endl;
                            targetYaw = SECONDHALF_GOAL;
                        }

                        if(abs(visionInfo.yaw) <= 90)
                            ikMsg.L_yaw = DEFAULT_YAW + (visionInfo.yaw - targetYaw);
                        else
                        {
                            if(targetYaw == -90)
                            {
                                if(visionInfo.yaw > targetYaw)
                                    ikMsg.L_yaw = DEFAULT_YAW + (targetYaw - visionInfo.yaw);
                                else
                                    ikMsg.L_yaw = DEFAULT_YAW + (visionInfo.yaw - targetYaw);
                            }
                            else
                            {
                                if(visionInfo.yaw > targetYaw)
                                    ikMsg.L_yaw = DEFAULT_YAW + (visionInfo.yaw - targetYaw);
                                else
                                    ikMsg.L_yaw = DEFAULT_YAW + (targetYaw - visionInfo.yaw);
                            }
                        }
                    }

                    ikMsg.Y_length = DEFAULT_Y;
                    ikMsg.Z_length = DEFAULT_Z;
                    ikMsg.L_yaw = DEFAULT_YAW + (abs(visionInfo.yaw) < 90 ?
                                                     (abs(visionInfo.yaw) > 10 ? visionInfo.yaw : 0) :
                                                     (abs(visionInfo.yaw) < 170 ? (visionInfo.yaw > 0 ? visionInfo.yaw - 170 : visionInfo.yaw + 170) : 0));
                }
            }

            if(ikMsg.X_length > 50) ikMsg.X_length = 50;

            if(ikMsg.L_yaw > 10)        ikMsg.L_yaw = 10;
            else if(ikMsg.L_yaw < -10)  ikMsg.L_yaw = -10;

            cout << "X = " << ikMsg.X_length << endl;
            cout << "Y = " << ikMsg.Y_length << endl;
            cout << "Z = " << ikMsg.Z_length << endl;
            cout << "Yaw = " << ikMsg.L_yaw << endl;

            break;
        case STATE_SET:
            cout << "********************\n";
            cout << "   SET\n";
            cout << "********************\n";

            ikMsg.flag = false;

            if(gameInfo.kickoffTeam == ROBIT)
                robocup_case = MODE_KICK;
            else
                robocup_case = MODE_BALL_DETECT;

            break;
        case STATE_PLAYING:
            cout << "********************\n";
            cout << "   PLAYING\n";
            cout << "********************\n";

            if(!isPlaying && gameInfo.kickoffTeam != ROBIT && gameInfo.readyTime)
            {
                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = visionInfo.ballX;
                Tracking_thing_y = visionInfo.ballY;

                cout << "wait time = " << gameInfo.readyTime << endl;
                break;
            }

            isPlaying = true;

            switch (robocup_case)
            {
            case MODE_BALL_DETECT:
                cout << endl << "&&&&&&&&&&&&&&&&&&&  MODE_BALL_DETECT  &&&&&&&&&&&&&&&&&&&&&" << endl << endl;

                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = visionInfo.ballX;
                Tracking_thing_y = visionInfo.ballY;

                ikMsg.flag = false;

                if(TRacking_flag_C == 0) //Find Ball
                    tracking_cnt++;
                else
                {
                    tracking_cnt = 0;
                    lost_ball_count++;

                    if(lost_ball_count > 2000 && tracking_cnt < 50)
                    {
                        robocup_case = MODE_NO_BALL;
                        lost_ball_count = 0;
                    }
                }

                cout<<"tracking_cnt : "<<tracking_cnt<<endl;

                if(tracking_cnt > 50)   //30
                {
                    tracking_cnt = 0;
                    lost_ball_count = 0;
                    robocup_case = MODE_BALL_WALK;
                }

                break;
            case MODE_BALL_STAND:
                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = visionInfo.ballX;
                Tracking_thing_y = visionInfo.ballY;


                cout << endl << "&&&&&&&&&&&&&&&&&&&  MODE_BALL_STAND  &&&&&&&&&&&&&&&&&&&&&" << endl << endl;

                if(TRacking_flag_C)
                {
                    lost_ball_count++;

                    if(lost_ball_count >= 500)
                    {
                        lost_ball_count = 0;
                        robocup_case = MODE_BALL_DETECT;
                    }
                }

                ikMsg.flag = true;

                if(isSet || visionInfo.yaw < GOALPOST+15 && visionInfo.yaw > GOALPOST-15) // 0 ~ 180
                {
                    isSet = true;
                    if(visionInfo.ballD < DesireBallD_stand && visionInfo.ballD > 0 && visionInfo.ballX != 0)//SHOOT  //f2p 250  220
                    {
                        cout<<endl<<"++++++++++++++++++++++++TURN FINISH+++++++++++++++++++++++++++++"<<endl<<endl;
                        ikMsg.flag = false;
                        lost_ball_count = 0;
                        if(visionInfo.ballY < 320)
                            motion_what = SHOOT_L;
                        else
                            motion_what = SHOOT_R;

                        robocup_case = MODE_KICK;
                        isSet = false;
                    }
                    else if(visionInfo.ballD > DesireBallD_stand && visionInfo.ballX != 0)
                    {
                        ikMsg.X_length = DEFAULT_X + 50;
                        ikMsg.Y_length = DEFAULT_Y;
                        ikMsg.Z_length = DEFAULT_Z;
                        ikMsg.L_yaw = DEFAULT_YAW;

                        if(visionInfo.ballD < DesireBallD_stand + 150 && visionInfo.ballD > DesireBallD_stand)
                        {
                            ikMsg.X_length = DEFAULT_X + visionInfo.ballD*0.065;
                        }

                        if(PAN_POSITION > 0)//puck robot
                        {
                            ikMsg.L_yaw = DEFAULT_YAW - (PAN_POSITION * 0.3);
                        }
                        else if(PAN_POSITION < 0)//robot puck
                        {
                            ikMsg.L_yaw = DEFAULT_YAW - (PAN_POSITION * 0.3);
                        }
                        else
                        {
                            ikMsg.L_yaw = DEFAULT_YAW;
                        }

                        if(ikMsg.L_yaw > 10) ikMsg.L_yaw = 10;
                        else if(ikMsg.L_yaw < -10) ikMsg.L_yaw = -10;
                    }
                }
                else //TURN
                {
                    if(visionInfo.ballX == 0)
                        ikMsg.flag = false;

                    else if(visionInfo.ballX != 0)
                    {
                        lost_ball_count = 0;

                        //turn
                        if(GOALPOST == 90)
                        {
                            if(visionInfo.yaw >= -90 && visionInfo.yaw <= 75)
                            {
                                ikMsg.X_length = DEFAULT_X;
                                ikMsg.Y_length = DEFAULT_Y + 35;
                                ikMsg.Z_length = DEFAULT_Z;
                                ikMsg.L_yaw = DEFAULT_YAW - 3;
                                if(abs(visionInfo.yaw - GOALPOST) < 40)
                                {
                                    ikMsg.X_length = DEFAULT_X;
                                    ikMsg.Y_length = DEFAULT_Y + 20;
                                    ikMsg.Z_length = DEFAULT_Z;
                                    ikMsg.L_yaw = DEFAULT_YAW - 3;
                                }
                            }
                            else if(visionInfo.yaw <= -90 || visionInfo.yaw >= 105)
                            {
                                ikMsg.X_length = DEFAULT_X;
                                ikMsg.Y_length = DEFAULT_Y - 35;
                                ikMsg.Z_length = DEFAULT_Z;
                                ikMsg.L_yaw = DEFAULT_YAW + 3;
                                if(abs(visionInfo.yaw-GOALPOST)<40)
                                {
                                    ikMsg.X_length = DEFAULT_X;
                                    ikMsg.Y_length = DEFAULT_Y - 20;
                                    ikMsg.Z_length = DEFAULT_Z;
                                    ikMsg.L_yaw = DEFAULT_YAW + 3;
                                }
                            }
                        }
                        else
                        {
                            if(visionInfo.yaw <= 90)
                            {
                                ikMsg.X_length = DEFAULT_X;
                                ikMsg.Y_length = DEFAULT_Y - 30;
                                ikMsg.Z_length = DEFAULT_Z;
                                ikMsg.L_yaw = DEFAULT_YAW + 6;
                            }
                            else if(visionInfo.yaw <= -100 || visionInfo.yaw >= 90)
                            {
                                ikMsg.X_length = DEFAULT_X;
                                ikMsg.Y_length = DEFAULT_Y + 30;
                                ikMsg.Z_length = DEFAULT_Z;
                                ikMsg.L_yaw = DEFAULT_YAW - 6;
                            }
                        }

                        if(visionInfo.ballD > DesireBallD_walk)
                            ikMsg.X_length = DEFAULT_X + visionInfo.ballD*0.065;
                        else if(visionInfo.ballD < DesireBallD_walk)
                            ikMsg.X_length = DEFAULT_X - visionInfo.ballD*0.065;

                        if(PAN_POSITION > 0)//puck robot
                        {
                            ikMsg.L_yaw = DEFAULT_YAW - (PAN_POSITION * 0.3);
                        }
                        else if(PAN_POSITION < 0)//robot puck
                        {
                            ikMsg.L_yaw = DEFAULT_YAW - (PAN_POSITION * 0.3);
                        }
                        else
                        {
                            ikMsg.L_yaw = DEFAULT_YAW;
                        }

                        if(ikMsg.L_yaw > 10) ikMsg.L_yaw = 10;
                        else if(ikMsg.L_yaw < -10) ikMsg.L_yaw = -10;

                    }

                }


                break;
            case MODE_NO_BALL:
                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = visionInfo.ballX;
                Tracking_thing_y = visionInfo.ballY;

                cout << endl << "&&&&&&&&&&&&&&&&&&&  MODE_NO_BALL  &&&&&&&&&&&&&&&&&&&&&" << endl << endl;

                ikMsg.flag = true;
                ikMsg.X_length = DEFAULT_X;
                ikMsg.Y_length = DEFAULT_Y;
                ikMsg.Z_length = DEFAULT_Z;
                ikMsg.L_yaw = DEFAULT_YAW - 4;

                if(!TRacking_flag_C)
                {
                    tracking_cnt++;
                    ikMsg.flag = false;
                }
                else tracking_cnt = 0;

                if(tracking_cnt > 80)
                {
                    tracking_cnt = 0;
                    robocup_case = MODE_BALL_WALK;
                }

                break;
            case MODE_BALL_WALK:
                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = visionInfo.ballX;
                Tracking_thing_y = visionInfo.ballY;

                cout << endl << "&&&&&&&&&&&&&&&&&&&  MODE_BALL_WALK  &&&&&&&&&&&&&&&&&&&&&" << endl << endl;

                if(TRacking_flag_C)
                {
                    if(lost_ball_count++ >= 400)
                    {
                        lost_ball_count = 0;
                        robocup_case = MODE_BALL_DETECT;
                        ikMsg.flag = false;
                    }
                }
                
                if(isUDP)
                {
                    if(YourBallD >= visionInfo.ballD || YourBallD == 0)  //mydist < youdist
                    {
                        My_cnt++;
                        You_cnt = 0;
                    }
                    else    //mydist > youdist
                    {
                        My_cnt = 0;
                        You_cnt++;
                    }

                    if(My_cnt > 300)   // mydist < youdist
                    {
                        DesireBallD_walk = 170;
                    }
                    else if(You_cnt > 300)
                    {
                        DesireBallD_walk = 1500;
                    }

                }


                if(visionInfo.ballD < DesireBallD_walk && visionInfo.ballD > 0 && visionInfo.ballX != 0)//SHOOT  //f2p 250  220
                {
                    ikMsg.flag = false;
                    if(DesireBallD_walk == 170)
                        robocup_case = MODE_BALL_STAND;

                }
                else if(visionInfo.ballX != 0)
                {
                    ikMsg.flag = true;

                    ikMsg.X_length = DEFAULT_X + 50;
                    ikMsg.Y_length = DEFAULT_Y;
                    ikMsg.Z_length = DEFAULT_Z;
                    ikMsg.L_yaw = DEFAULT_YAW;

                    if(visionInfo.ballD < DesireBallD_walk+150 && visionInfo.ballD > DesireBallD_walk)
                    {
                        ikMsg.X_length = DEFAULT_X + visionInfo.ballD*0.07;
                    }

                    if(PAN_POSITION > 0)//puck robot
                    {
                        ikMsg.L_yaw = DEFAULT_YAW - (PAN_POSITION * 0.3);
                    }
                    else if(PAN_POSITION < 0)//robot puck
                    {
                        ikMsg.L_yaw = DEFAULT_YAW - (PAN_POSITION * 0.3);
                    }
                    else
                    {
                        ikMsg.L_yaw = DEFAULT_YAW;
                    }

                    if(ikMsg.L_yaw > 10) ikMsg.L_yaw = 10;
                    else if(ikMsg.L_yaw < -10) ikMsg.L_yaw = -10;
                }

                break;
            case MODE_KICK_READY:

                break;
            case MODE_KICK:
                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = ballTrackingPointX;
                Tracking_thing_y = ballTrackingPointY;

                cout << endl << "&&&&&&&&&&&&&&&&&&&  MODE_KICK  &&&&&&&&&&&&&&&&&&&&&" << endl << endl;

                ikMsg.flag = false;
                cout << "walk_term : " << walk_term << endl;
                if(walk_term++ >= 200)
                {
                    motionFlag = true;
                    DXL_Motion(motion_what);

                    if(motionFlag)
                        motionFlag = false;

                    robocup_case = MODE_STOP;
                    scan_start = true;
                    walk_term = 0;
                }

                break;
                //            case SCAN:

                //                break;
            case MODE_STOP:
                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = ballTrackingPointX;
                Tracking_thing_y = ballTrackingPointY;

                cout << endl << "&&&&&&&&&&&&&&&&&&&  MODE_STOP  &&&&&&&&&&&&&&&&&&&&&" << endl << endl;
                if(motionEnd)
                {
                    robocup_case = MODE_BALL_DETECT;
                    motionEnd = 0;
                }

                break;
            default:
                break;
            }

            break;
        case STATE_FINISHED:
            cout << "********************\n";
            cout << "1. FINISHED\n";
            cout << "********************\n";

            isPlaying = false;

            Tracking_point_x = ballTrackingPointX;
            Tracking_point_y = ballTrackingPointY;
            Tracking_thing_x = 0;
            Tracking_thing_y = 0;

            ikMsg.flag = false;

            break;
        default:
            break;

            gameInfo.stateBefore = gameInfo.state;
            temp_case = robocup_case;
        }
    }

    ptMsg.Angle_Yaw = PAN_POSITION;
    ptMsg.Angle_Pitch = TILT_POSITION;

    pantiltPub.publish(ptMsg);
    ikPub.publish(ikMsg);
//    mtPub.publish(mtMsg);
}

void gameCallback(const gamecontroller::robocupController2::ConstPtr &msg)
{
    gameInfo.firstHalf = msg->firstHalf;
    gameInfo.kickoffTeam = msg->kickoffTeam;
    gameInfo.state = msg->state;
    gameInfo.readyTime = msg->readyTime;
    gameInfo.penalty = msg->penalty;
}

void visionCallback(const robocup2019_vision::robocupVision_msg::ConstPtr &msg)
{
    visionInfo.ballX = msg->ballX;
    visionInfo.ballY = msg->ballY;
    visionInfo.ballD = msg->ballD;

    visionInfo.targetX = msg->targetX;
    visionInfo.targetY = msg->targetY;
    visionInfo.targetYaw = msg->targetYaw;

    visionInfo.nowX = msg->nowX;
    visionInfo.nowY = msg->nowY;

    visionInfo.yaw = msg->yaw;
}

void imuCallback(const mw_ahrsv1::imu_msg::ConstPtr &msg)
{
    imuInfo.pitch = msg->pitch;
    imuInfo.roll = msg->roll;
    imuInfo.yaw = msg->yaw;
}

void motionCallback(const serial_mcu::motion_end::ConstPtr &msg)
{
    motionEnd = msg->motion_end;
}

void udpCallback(const udpcom::udp_order::ConstPtr &msg)
{
    YourBallD = msg->order;
}

int Tracking(double now_x, double X_POINT_STANDARD, double now_y, double Y_POINT_STANDARD)
{
    int Tracking_flag = 1;
    double Pan_temp_glass;
    double Tilt_temp_glass;
    static int waist_flag = 0;
    int mode = 0;
    static int scan_flag = 0;
    static double scan_var = -0.6;
    static int missing_cnt = 0;

        static int PAN_Flag = 1;
        static int TILT_Flag = 0;
        static int TILT_UP_COUNT = 0;

        if(now_x == 0)
        {
            missing_cnt++;
            mode = 0;
        }
        else
        {
            missing_cnt = 0;
            mode = 1;
        }

    if(missing_cnt > 50)
        mode = 2;


    if(mode == 1)   // Tracking
    {
        Tracking_flag = 0;

        PID_Control_Float(&Tracking_pid_pan, X_POINT_STANDARD, now_x);
        PID_Control_Float(&Tracking_pid_tilt, Y_POINT_STANDARD, now_y);

        Pan_temp_glass = -Tracking_pid_pan.nowOutput;
        PAN_POSITION += Pan_temp_glass;

        Tilt_temp_glass = -Tracking_pid_tilt.nowOutput;
        TILT_POSITION += Tilt_temp_glass;

        scan_var = -0.6;
        scan_flag = 0;

        if(PAN_POSITION > Pan_Max)
        {
            PAN_POSITION = Pan_Max;
        }

        else if(PAN_POSITION < Pan_Min)
        {
            PAN_POSITION = Pan_Min;
        }

        if(TILT_POSITION > Tilt_Max)
        {
            TILT_POSITION = Tilt_Max;
        }

        else if(TILT_POSITION < Tilt_Min)
        {
            TILT_POSITION = Tilt_Min;
        }

    }

    else if(mode == 2)  // Scan
    {
        std::cout << "TILT_Flag = " << TILT_Flag << std::endl;
        std::cout << "PAN_Flag = " << PAN_Flag << std::endl;
        std::cout << "tiltupcount = " << TILT_UP_COUNT << std::endl;
        std::cout << "PAN_POSITION = " << PAN_POSITION << std::endl;
        std::cout << "TILT_POSITION = " << TILT_POSITION << std::endl;

        if(PAN_Flag == 1) // Go To Right
        {
            PAN_POSITION = PAN_POSITION - 0.4;

            if(PAN_POSITION < -ballScanPanRange)
            {
                TILT_Flag = 1;
                TILT_UP_COUNT++;
            }
        }
        else if(PAN_Flag == 2) // Go To Left
        {
            PAN_POSITION = PAN_POSITION + 0.4;

            if(PAN_POSITION > ballScanPanRange)
            {
                TILT_Flag = 1;
                TILT_UP_COUNT++;
            }
        }

        if(TILT_Flag == 1)
        {
            PAN_Flag = 0;
            TILT_POSITION = TILT_POSITION - 0.5;
            if(TILT_POSITION < -80 + (4 - TILT_UP_COUNT) * 20)
            {
                if(TILT_UP_COUNT % 2 == 0)
                    PAN_Flag = 1;
                if(TILT_UP_COUNT % 2 == 1)
                    PAN_Flag = 2;
                TILT_Flag = 0;
            }
        }

        if(scan_start || TILT_UP_COUNT >= 4)
        {
            TILT_POSITION = 0; // 80
            TILT_UP_COUNT = 0;
            scan_start = false;
        }

        if(TILT_POSITION > Tilt_Max)
        {
            TILT_POSITION = Tilt_Max;
        }

        else if(TILT_POSITION < Tilt_Min)
        {
            TILT_POSITION = Tilt_Min;
        }

    }


    return Tracking_flag;
}

void DXL_Motion(unsigned char Motion_Num)
{
    serial_mcu::Mt2Serial_msg motion_msg;
    motion_msg.Motion_Mode = 1;
    motion_msg.Motion_Num = Motion_Num;
    if(motionFlag)
        mtPub.publish(motion_msg);
}

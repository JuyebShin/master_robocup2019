#include "../include/master_robocup2019/master_robocup2019.hpp"

using namespace std;

GameState gameInfo;
VisionMsg visionInfo;
ImuMsg imuInfo;


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
int Goalpost_error = 4;

bool scan_start = false;
bool local = false;
bool isPlaying = false;
bool isSet = false;
bool isUDP = true;
bool isFalldown = false;
bool wasPenalty = false;
bool wasInitial = false;
bool wasSet = false;
bool isNoball = false;
bool isNoball_pan = false;
bool Tracking_ON_OFF = true;

bool stop = false;

int WHAT_YOUR_CASE = 0;
int WHAT_YOUR_BALLD = 0;
int My_cnt = 0;
int You_cnt = 0;
int YourBallD = 0;
int DesireBallD_walk_my = 190;//230
int DesireBallD_walk_your = 2100;
int DesireBallD_stand = 80;
int DesireWalkSpeed = 73;
int DesirePP_R = -23;
int DesirePP_L = 23;

int motion_what = SHOOT_R;
int add_what = 0;


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
    motionendSub = n.subscribe("motion_end", 100, motionCallback);
    udpsub = n.subscribe("udp_order", 100, udpCallback);

    pantiltPub = n.advertise<pan_tilt::pan_tilt_msg>("pantilt", 100);
    motionPub = n.advertise<serial_mcu::Mt2Serial_msg>("Motion", 100);
    ikPub = n.advertise<inverse_kinematics::info_kinematics_msg>("kinematics", 100);
    casePub = n.advertise<udpcom::master2udp>("udp_case", 100);

    ros::Timer robocuoTimer = n.createTimer(ros::Duration(0.01), robocupCallback);

    ros::spin();
}

void robocupCallback(const ros::TimerEvent&)
{
    cout<<"===========motion end==========    "<<motion_end<<endl<<endl;
    cout<<"add_what = " <<add_what<<endl;
    cout<<"PAN_POSITION ======== "<<PAN_POSITION<<endl;
    cout<<"WHAT ?? case??" <<WHAT_YOUR_CASE<<endl;
    int TRacking_flag_C = Tracking(Tracking_thing_x, Tracking_point_x, Tracking_thing_y, Tracking_point_y);

    static int tracking_cnt = 0;

    static int lost_ball_count = 0;
    static int temp_yaw = 0;

    cout<<"YAWWWWWWWWWWWW       "<<visionInfo.yaw<<endl;
    cout<<"temp                 "<<temp_yaw<<endl;

    if(gameInfo.firstHalf == true)
        GOALPOST = FIRSTHALF_GOAL;
    else
        GOALPOST = SECONDHALF_GOAL;

    /*For Test*/
   // gameInfo.state = STATE_PLAYING;
   // GOALPOST = SECONDHALF_GOAL;
////    robocup_case = MODE_BALL_DETECT;

    cout<<"imuInfo.roll = "<<imuInfo.roll<<endl;

    if(imuInfo.roll > 50.0)
    {
        DXL_Motion(STAND_R);
        isFalldown = true;
    }
    else if(imuInfo.roll < -50.0)
    {
        DXL_Motion(STAND_L);
        isFalldown = true;
    }

    if(imuInfo.pitch > 50.0)
    {
        DXL_Motion(STAND_F);
        isFalldown = true;
    }
    else if(imuInfo.pitch < -50.0)
    {
        DXL_Motion(STAND_B);
        isFalldown = true;
    }

    if(gameInfo.penalty)
    {
        TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, ballTrackingPointX, ballTrackingPointY);
        WALK_STOP;
        robocup_case = MODE_BALL_DETECT;
        wasPenalty = true;
    }
    else
    {
        if(isFalldown)
        {
            cout<<"FALL DOWN!!!!!!!!!!!!!!!!!!!!"<<endl<<endl;
            if(motion_end)
            {
                robocup_case = MODE_BALL_DETECT;
                isFalldown = false;
                motion_end = 0;
            }
        }
        else{  // is not Falldown
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

            wasInitial = true;

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

                if(visionInfo.BallDist > 100) // 10 cm
                {
                    ikMsg.X_length = DEFAULT_X + 65;
                }
                else if(visionInfo.BallDist || gameInfo.readyTime < 5) // 20 cm
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

                    if(abs(visionInfo.yaw - targetYaw) <= 10) ikMsg.flag = false;
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
                        if(gameInfo.firstHalf)
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
                        if(gameInfo.firstHalf)
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

                    ikMsg.X_length = DEFAULT_X + (abs(ikMsg.L_yaw) > 10 ? 35 : 75);
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

                        if(abs(targetYaw - visionInfo.yaw) < 10) stop = true;

                        if(stop) ikMsg.flag = false;
                    }

                    ikMsg.Y_length = DEFAULT_Y;
                    ikMsg.Z_length = DEFAULT_Z;
//                    ikMsg.L_yaw = DEFAULT_YAW + (abs(visionInfo.yaw) < 90 ?
//                                                     (abs(visionInfo.yaw) > 10 ? visionInfo.yaw : 0) :
//                                                     (abs(visionInfo.yaw) < 170 ? (visionInfo.yaw > 0 ? visionInfo.yaw - 170 : visionInfo.yaw + 170) : 0));
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
            stop = false;
            wasSet = true;

            if(gameInfo.kickoffTeam == ROBIT)
            {
                robocup_case = MODE_KICK;
                motion_what = SHOOT_L;
            }
            else
                robocup_case = MODE_BALL_DETECT;

//            robocup_case = MODE_BALL_DETECT;

            break;
        case STATE_PLAYING:
            cout << "********************\n";
            cout << "   PLAYING\n";
            cout << "********************\n";

            if(!isPlaying && gameInfo.kickoffTeam != DROPBALL && gameInfo.kickoffTeam != ROBIT && gameInfo.readyTime)
            {
                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = visionInfo.Ballx;
                Tracking_thing_y = visionInfo.Bally;

                cout << "wait time = " << gameInfo.readyTime << endl;
                break;
            }

            isPlaying = true;

            switch (robocup_case)
            {
            case MODE_TEST:
                cout<<"============================"<<endl;
                cout<<"  write goalpost direction"<<endl;
                cout<<"  LEFT:  1        RIGHT: 2"<<endl;
                cout<<"============================"<<endl;

                robocup_case = MODE_BALL_DETECT;

                break;

            case MODE_BALL_DETECT:
                cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_BALL_DETECT  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;

                TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.Ballx, visionInfo.Bally);
                WALK_STOP;


                if(TRacking_flag_C == 0) //Find Ball
                    tracking_cnt++;
                else
                {
                    tracking_cnt = 0;
                    lost_ball_count++;

                    if(lost_ball_count > 800 && tracking_cnt < 50)
                    {
                        robocup_case = MODE_NO_BALL;
                        lost_ball_count = 0;
                    }
                }

                cout<<"tracking_cnt : "<<tracking_cnt<<endl;

                if(tracking_cnt > 80)   //30
                {
                    tracking_cnt = 0;
                    lost_ball_count = 0;
                    robocup_case = MODE_BALL_WALK;
                }

                break;


            case MODE_NO_BALL:
                cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_NO_BALL  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;

                TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.Ballx, visionInfo.Bally);
                isNoball = true;
                static int straight_cnt = 0;
                straight_cnt++;

                if(wasPenalty || wasInitial || wasSet)
                {
                    WALK_START(40,0,0);
                    isNoball_pan = true;
                }
                else if((!wasInitial && !wasPenalty && !wasSet && straight_cnt >= 2000) || straight_cnt > 6000)
                {
                    WALK_START(35,0,6);
                    isNoball_pan = true;
                }
                else if((!wasInitial && !wasPenalty && !wasSet) || ((wasInitial || wasPenalty || wasSet) && (straight_cnt >= 2000 && straight_cnt <= 6000)))
                {
                    WALK_START(0,0,6);
                    isNoball_pan = false;
                }



                if(TRacking_flag_C == 0) //Find Ball
                {
                    tracking_cnt++;
                    //WALK_STOP;
                    WALK_START(0,0,0);
                }

                else
                    tracking_cnt = 0;

                cout<<"tracking_cnt : "<<tracking_cnt<<endl;

                if(tracking_cnt > 80)   //30
                {
                    tracking_cnt = 0;
                    isNoball = false;
                    isNoball_pan = false;
                    robocup_case = MODE_BALL_WALK;
                    wasPenalty =false;
                    wasInitial = false;
                    wasSet = false;
                    straight_cnt = 0;
                }

                break;


            case MODE_BALL_WALK:
                cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_BALL_WALK  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;
                static int Desire_BallD = 0;
                Desire_BallD = DesireBallD_walk_my;
                wasPenalty = false;
                wasInitial = false;
                wasSet = false;
                static int flag = 1;


                cout<<"lost_ball_count   ===   "<<lost_ball_count<<endl;



                TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.Ballx, visionInfo.Bally);

                if(TRacking_flag_C)
                {
                    lost_ball_count++;

                    if(lost_ball_count >= 400)
                    {
                        lost_ball_count = 0;
                        robocup_case = MODE_BALL_DETECT;
                        flag = 1;
                        WALK_STOP;
                    }
                }

                if(isUDP)
                {
                    if(WHAT_YOUR_CASE > robocup_case)// your
                    {
                        Desire_BallD = DesireBallD_walk_your;
                        DesireWalkSpeed = 73;
                    }
                    else if(WHAT_YOUR_CASE <= robocup_case)
                    {
                        Desire_BallD = DesireBallD_walk_my;
                        DesireWalkSpeed = 73;
                    }
                    else if(WHAT_YOUR_CASE == robocup_case)
                    {
                        static int my_cnt = 0;
                        static int your_cnt = 0;

                        if(WHAT_YOUR_BALLD >= visionInfo.BallDist && WHAT_YOUR_BALLD != 0)
                        {
                            my_cnt++;
                            your_cnt = 0;
                        }
                        else if(WHAT_YOUR_BALLD < visionInfo.BallDist && WHAT_YOUR_BALLD != 0)
                        {
                            your_cnt++;
                            my_cnt = 0;
                        }

                        if(flag){
                        if(my_cnt > 10)
                        {
                            DesireWalkSpeed = 73;
                            my_cnt = 0;
                            your_cnt = 0;
                            flag = 0;

                        }
                        else if(your_cnt > 10)
                        {
                            DesireWalkSpeed = 0;
                            my_cnt = 0;
                            your_cnt = 0;
                            flag = 0;
                        }

                        }

                    }

                }

                cout<<"SSSSSSSSIIIIIBBBAAAALLLLLLLLLLLLLLLLLLLLLLLLLL"<<Desire_BallD<<endl<<endl;

                if(visionInfo.BallDist < Desire_BallD && visionInfo.BallDist > 0 && visionInfo.Ballx != 0)//SHOOT
                {
                    if(Desire_BallD == DesireBallD_walk_my)
                    {
                        robocup_case = MODE_BALL_STAND;
                        flag = 1;
                    }
                    else if(Desire_BallD == DesireBallD_walk_your)
                    {
                        if(visionInfo.BallDist < 1600)
                        {
                             WALK_START(-25, 0, 0);
                        }
                        else
                            WALK_STOP;
                    }
                }
                else if(visionInfo.Ballx != 0)
                {
                    WALK_START(DesireWalkSpeed, 0, 0);

                    if(Desire_BallD == DesireBallD_walk_my)
                    {
                        if(visionInfo.BallDist < DesireBallD_walk_my+150 && visionInfo.BallDist > DesireBallD_walk_my)
                        {
                            ikMsg.X_length = DEFAULT_X + visionInfo.BallDist*0.1;
                        }
                    }

                    SET_YAW_TO_BALL();
                 }

                break;

            case MODE_BALL_STAND:
                cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_BALL_STAND  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;
                cout<<"isSet   =  "<<isSet<<endl;
                static int stand_cnt = 0;
                TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, visionInfo.Ballx, visionInfo.Bally);
                stand_cnt++;

                if(TRacking_flag_C)// IF LOST BALL
                {
                    lost_ball_count++;

                    if(lost_ball_count >= 500)
                    {
                        lost_ball_count = 0;
                        robocup_case = MODE_BALL_DETECT;
                    }
                }

                if(isSet||visionInfo.yaw < GOALPOST+Goalpost_error && visionInfo.yaw > GOALPOST-Goalpost_error) //TURN FINISH
                {
                    temp_yaw = visionInfo.yaw;
                    isSet = true;

                    if(visionInfo.BallDist < DesireBallD_stand && visionInfo.BallDist > 0 && visionInfo.Ballx != 0)//KICK
                    {
                        cout<<endl<<"+++++++++TURN FINISH++++++++++"<<endl<<endl;
                        //WALK_STOP;
                        lost_ball_count = 0;
                        isSet = false;


                        robocup_case = MODE_SET_Y;    //MODE_KICK
                    }
                    else if(visionInfo.BallDist > DesireBallD_stand && visionInfo.Ballx != 0)
                    {
                        WALK_START(35, 0, 0);

                        if(visionInfo.BallDist < DesireBallD_stand+150 && visionInfo.BallDist > DesireBallD_stand)
                        {
                             WALK_START(visionInfo.BallDist*0.075, 0, 0);//0.08
                        }
                        SET_YAW_TO_BALL();

//                        if(visionInfo.yaw < GOALPOST) // LEFT TURN
//                            ikMsg.L_yaw = DEFAULT_YAW - (abs(visionInfo.yaw - GOALPOST)*0.8);
//                        else if(visionInfo.yaw > GOALPOST) //RIGHT TURN
//                            ikMsg.L_yaw = DEFAULT_YAW + (abs(visionInfo.yaw - GOALPOST)*0.8);


                    }
                }
                else //TURN TO SET GOALPOST DIRECTION
                {
                    cout<<"visioninfo Yaw   =====   "<<visionInfo.yaw<<endl;

                    //lost ball
                    if(visionInfo.Ballx == 0)
                        WALK_STOP;

                    else if(visionInfo.Ballx!=0)
                    {
                        lost_ball_count = 0;

                        //turn
                        if(GOALPOST == 90)
                        {
                            if(visionInfo.yaw >= -90 && visionInfo.yaw <= GOALPOST-Goalpost_error)
                            {
                                //WALK_START(0,40,-3);
                                WALK_START(0,45,-2);
                                if(abs(visionInfo.yaw-GOALPOST)<40)
                                    WALK_START(0,42,-1);
                            }
                            else if(visionInfo.yaw <= -90 || visionInfo.yaw >= GOALPOST + Goalpost_error)
                            {
                                //WALK_START(0,-40,3);
                                WALK_START(0,-35,2);
                                if(abs(visionInfo.yaw-GOALPOST)<40)
                                    WALK_START(0,-32,1);
                            }
                        }
                        else
                        {
                            if(visionInfo.yaw <= 90 && visionInfo.yaw >= GOALPOST + Goalpost_error)
                            {
                                //WALK_START(0,-40,3);
                                WALK_START(0,-35,2);
                                if(abs(visionInfo.yaw-GOALPOST)<40)
                                    WALK_START(0,-32,1);
                            }
                            else if(visionInfo.yaw <= GOALPOST-Goalpost_error || visionInfo.yaw >= 90)
                            {
                                //WALK_START(0,40,-3);
                                WALK_START(0,45,-2);
                                if(abs(visionInfo.yaw-GOALPOST)<40)
                                    WALK_START(0,42,-1);
                            }
                        }

                        // SET BALLDISTANCE
                        if(visionInfo.BallDist > DesireBallD_walk_my)
                            ikMsg.X_length = DEFAULT_X + visionInfo.BallDist*0.05;
                        else if(visionInfo.BallDist < DesireBallD_walk_my)
                            ikMsg.X_length = DEFAULT_X - visionInfo.BallDist*0.05;

                        SET_YAW_TO_BALL();

                    }
                }

                break;


            case MODE_SET_Y:
                cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_SET_Y  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;
                cout<<"PAN = "<<PAN_POSITION<<endl;
                TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY,  visionInfo.Ballx, visionInfo.Bally);
                static int set_cnt = 0;
                set_cnt++;

                cout<<"SET_Y_cnt       ===============      "<<set_cnt<<endl;

                if(PAN_POSITION > 0) //BALL ROBOT
                {
                    if(set_cnt > 1000)
                    {
                        motion_what = SHOOT_L;
                        robocup_case = MODE_KICK;
                        set_cnt = 0;
                    }
                    else if(abs(DesirePP_L - PAN_POSITION) < 15)
                    {
                        motion_what = SHOOT_L;
                        robocup_case = MODE_KICK;
                        set_cnt = 0;
                    }
                    else if(PAN_POSITION < DesirePP_L - 15)
                    {
                        WALK_START(0,5,0);
                    }
                    else if(PAN_POSITION > DesirePP_L + 15)
                    {
                        WALK_START(0,-5,0);
                    }
                }
                else if(PAN_POSITION < 0)
                {
                    if(set_cnt > 1000)
                    {
                        motion_what = SHOOT_R;
                        robocup_case = MODE_KICK;
                        set_cnt = 0;
                    }
                    if(abs(DesirePP_R - PAN_POSITION) < 15)
                    {
                        motion_what = SHOOT_R;
                        robocup_case = MODE_KICK;
                        set_cnt = 0;
                    }
                    else if(PAN_POSITION < DesirePP_R - 15)
                    {
                        WALK_START(0,5,0);
                    }
                    else if(PAN_POSITION > DesirePP_R + 15)
                    {
                        WALK_START(0,-5,0);
                    }
                }


                break;


            case MODE_KICK:
                cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_KICK  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;

                WALK_STOP;
                cout<<"walk_term : "<<walk_term<<endl;
                if(walk_term++ >= 170)
                {
                    motion_flag = true;
                    DXL_Motion(motion_what);

                    if(motion_flag)
                        motion_flag = false;

                    robocup_case = MODE_STOP;
                    add_what -= 3;
                    scan_start = true;
                    walk_term = 0;
                }
                break;


            case MODE_STOP:
                cout<<endl<<"&&&&&&&&&&&&&&&&&&&  MODE_STOP  &&&&&&&&&&&&&&&&&&&&&"<<endl<<endl;

                if(motion_end)
                {
                    robocup_case = MODE_BALL_DETECT;
                    motion_end = 0;
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

            TRACKING_WHAT(ballTrackingPointX, ballTrackingPointY, 0, 0);
            WALK_STOP;

            break;
        default:
            break;

            gameInfo.stateBefore = gameInfo.state;
            temp_case = robocup_case;
        }}

    }

    ptMsg.Angle_Yaw = PAN_POSITION;
    ptMsg.Angle_Pitch = TILT_POSITION;

    pantiltPub.publish(ptMsg);
    ikPub.publish(ikMsg);

    if(gameInfo.state == STATE_PLAYING)
        m2uMsg.gamecase = robocup_case;
    else
        m2uMsg.gamecase = 0;
    m2uMsg.ball_dt = visionInfo.BallDist;
    casePub.publish(m2uMsg);
}



void SET_YAW_TO_BALL()
{
    if(PAN_POSITION > 0)//BALL ROBOT
    {
        ikMsg.L_yaw = DEFAULT_YAW - (PAN_POSITION * 0.3);
    }
    else if(PAN_POSITION < 0)//ROBOT BALL
    {
        ikMsg.L_yaw = DEFAULT_YAW - (PAN_POSITION * 0.3);
    }
    else
    {
        ikMsg.L_yaw = DEFAULT_YAW;
    }
    //yaw value limit
    if(ikMsg.L_yaw > 8) ikMsg.L_yaw = 8;
    else if(ikMsg.L_yaw < -8) ikMsg.L_yaw = -8;
}

void WALK_START(double x, double y, double yaw)
{
    ikMsg.flag = true;

    ikMsg.X_length = DEFAULT_X + x;
    ikMsg.Y_length = DEFAULT_Y + y;
    ikMsg.Z_length = DEFAULT_Z;
    ikMsg.L_yaw = DEFAULT_YAW + yaw;
}

void TRACKING_WHAT(int TrackingPoint_x, int TrackingPoint_y, int TrackingThing_x, int TrackingThing_y)
{
    Tracking_point_x = TrackingPoint_x;
    Tracking_point_y = TrackingPoint_y;
    Tracking_thing_x = TrackingThing_x;
    Tracking_thing_y = TrackingThing_y;
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
    visionInfo.Ballx = msg->ballX;
    visionInfo.Bally = msg->ballY;
    visionInfo.BallDist = msg->ballD;
    visionInfo.yaw = msg->yaw;// - add_what;
    visionInfo.targetY = msg->targetY;
    visionInfo.nowY = msg->nowY;
    visionInfo.targetYaw = msg->targetYaw;

//    if(visionInfo.yaw > 180)        visionInfo.yaw -= 360;
//    else if(visionInfo.yaw <= -180) visionInfo.yaw += 360;
}

void imuCallback(const mw_ahrsv1::imu_msg::ConstPtr &msg)
{
    imuInfo.pitch = msg->pitch;
    imuInfo.roll = msg->roll;
}

void motionCallback(const serial_mcu::motion_end::ConstPtr &msg)
{
    motion_end = msg->motion_end;
}
void udpCallback(const udpcom::udp2master::ConstPtr &msg)
{
    WHAT_YOUR_CASE = msg->yourcase;
    WHAT_YOUR_BALLD = msg->yourballdt;
}

void DXL_Motion(unsigned char Motion_Num)
{
    serial_mcu::Mt2Serial_msg motion_msg;
    motion_msg.Motion_Mode = 1;
    motion_msg.Motion_Num = Motion_Num;
    motionPub.publish(motion_msg);
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
    {
        mode = 2;

        if(isNoball && !isNoball_pan)
            mode = 3;
    }



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
            PAN_POSITION = PAN_POSITION - 1.0;

            if(PAN_POSITION < -ballScanPanRange)
            {
                TILT_Flag = 1;
                TILT_UP_COUNT++;
            }
        }
        else if(PAN_Flag == 2) // Go To Left
        {
            PAN_POSITION = PAN_POSITION + 1.0;

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

    else if(mode == 3)  // Only Tilt
    {
        std::cout << "TILT_Flag = " << TILT_Flag << std::endl;
        std::cout << "PAN_Flag = " << PAN_Flag << std::endl;
        std::cout << "tiltupcount = " << TILT_UP_COUNT << std::endl;
        std::cout << "PAN_POSITION = " << PAN_POSITION << std::endl;
        std::cout << "TILT_POSITION = " << TILT_POSITION << std::endl;

        PAN_POSITION = 0;

        if(!TILT_Flag)
        {
            TILT_POSITION = TILT_POSITION - 0.5;
            if(TILT_POSITION < -80)
            {
                TILT_Flag = 1;
            }
        }
        else if(TILT_Flag)
        {
            TILT_POSITION = TILT_POSITION + 0.5;
            if(TILT_POSITION >= 0)
            {
                TILT_Flag = 0;
            }
        }

        if(scan_start)
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

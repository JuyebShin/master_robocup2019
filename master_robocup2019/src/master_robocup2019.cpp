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

int robocup_case;
int temp_case;

int finding_turn = 0;

bool local = false;
bool isPlaying = false;

int Gp_direction = 0;

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

    /** messages to publish
     * @ pan tilt message
     * @ inverse kinematics message
     * @ motion info message
     * */
    pan_tilt::pan_tilt_msg ptMsg;
    inverse_kinematics::info_kinematics_msg ikMsg;
    serial_mcu::Mt2Serial_msg mtMsg;

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

            if(isPlaying)
            {
                cout << "scored" << endl;
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

                ikMsg.Y_length = DEFAULT_Y;
                ikMsg.Z_length = DEFAULT_Z;
                ikMsg.L_yaw = DEFAULT_YAW - (targetYaw - visionInfo.yaw);

                ikMsg.X_length = DEFAULT_X + (abs(ikMsg.L_yaw) > 10 ? 15 : 30);
            }
            else
            {
                if(local)
                {
                    if(gameInfo.kickoffTeam == ROBIT)
                    {
                        cout << "KICK OFF" << endl;
                        Tracking_point_x = ballTrackingPointX;
                        Tracking_point_y = ballTrackingPointY;
                        Tracking_thing_x = 0;//visionInfo.ballX;
                        Tracking_thing_y = 0;//visionInfo.ballY;

                        if(TRacking_flag_C == 0)    tracking_cnt++; // Find Ball
                        else                        tracking_cnt = 0;

                        //            if(tracking_cnt > 10)
                        //            {
                        //                if(TILT_POSITION > -60)
                        //                    ikMsg.X_length = DEFAULT_X + 30;
                        //                else
                        //                    ikMsg.X_length = DEFAULT_X;

                        //                tracking_cnt = 0;
                        //            }
                        //            else
                        //                ikMsg.X_length = DEFAULT_X + 30;

                        cout << "targetY = " << visionInfo.targetY << endl;
                        cout << "nowY = " << visionInfo.nowY << endl;

                        if(visionInfo.nowY != 0)
                        {
                            int distance = abs(visionInfo.targetY - visionInfo.nowY);
                            cout << "distance = " << distance << endl;

                            ikMsg.X_length = DEFAULT_X + distance;
                        }
                        else
                            ikMsg.X_length = DEFAULT_X + 15;
                    }
                    else
                    {
                        Tracking_point_x = ballTrackingPointX;
                        Tracking_point_y = ballTrackingPointY;
                        Tracking_thing_x = 0;
                        Tracking_thing_y = 0;
                    }

                    //        ikMsg.X_length = DEFAULT_X + 30;
                    ikMsg.Y_length = DEFAULT_Y;
                    ikMsg.Z_length = DEFAULT_Z;
                    ikMsg.L_yaw = DEFAULT_YAW + visionInfo.yaw;

                    if(gameInfo.readyTime < 5)
                    {
                        cout << "turn" << endl;
                        ikMsg.X_length = DEFAULT_X;
                        ikMsg.Y_length = DEFAULT_Y;
                        ikMsg.Z_length = DEFAULT_Z;

                        if(gameInfo.firstHalf)
                        {
                            cout << "right" << endl;
                            targetYaw = -90;
                        }
                        else
                        {
                            cout << "left" << endl;
                            targetYaw = 90;
                        }

                        cout << "target yaw = " << targetYaw
                             << " now yaw = " << visionInfo.yaw << endl;

                        ikMsg.L_yaw = DEFAULT_YAW - (targetYaw - visionInfo.yaw);
                    }
                }
                else
                {
                    Tracking_point_x = ballTrackingPointX;
                    Tracking_point_y = ballTrackingPointY;
                    Tracking_thing_x = 0;
                    Tracking_thing_y = 0;

                    ikMsg.X_length = DEFAULT_X + 30;
                    ikMsg.Y_length = DEFAULT_Y;
                    ikMsg.Z_length = DEFAULT_Z;
                    ikMsg.L_yaw = DEFAULT_YAW + (abs(visionInfo.yaw) < 90 ? visionInfo.yaw : (visionInfo.yaw > 0 ? visionInfo.yaw - 180 : visionInfo.yaw + 180));

                    if(gameInfo.readyTime < 5)
                    {
                        ikMsg.X_length = DEFAULT_X + 30;
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

                        ikMsg.L_yaw = DEFAULT_YAW - (targetYaw - visionInfo.yaw);
                    }
                }
            }

            if(ikMsg.X_length > 30) ikMsg.X_length = 30;

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


            if(gameInfo.kickoffTeam == ROBIT)
            {
                robocup_case = MODE_KICK;
            }
            else
            {
                robocup_case = MODE_BALL_DETECT;
            }

            ikMsg.flag = false;

            break;
        case STATE_PLAYING:
            cout << "********************\n";
            cout << "   PLAYING\n";
            cout << "********************\n";

            isPlaying = true;

            switch (robocup_case)
            {
            case MODE_BALL_DETECT:
                Tracking_point_x = ballTrackingPointX;
                Tracking_point_y = ballTrackingPointY;
                Tracking_thing_x = visionInfo.ballX;
                Tracking_thing_y = visionInfo.ballY;

                break;
            case MODE_BALL_STAND:

                break;
            case MODE_BALL_DETAIL:

                break;
            case MODE_KICK_READY:

                break;
            case MODE_KICK:

                robocup_case = MODE_BALL_DETECT;

                break;
            case SCAN:

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
    static int turn_cnt = 0;
    static int detecting_cnt = 0;
    int waista = 0;
    static int waista_past = 0;
    static int waista_cnt = 0;
    static int PAN_Flag = 1;
    static int TILT_Flag = 0;
    static int TILT_UP_COUNT = 0;

    if(now_x == 0 && (turn_cnt <= 8))
    {
        missing_cnt++;
        waista = 0;
        mode = 0;
    }
    else if(finding_turn)
    {
        mode = 2;
        waist_flag = 0;
        turn_cnt = 0;
    }
    else
    {
        missing_cnt = 0;
        mode = 1;
        waista = 1;
    }

    if(waista != waista_past)
        waista_cnt++;
    else
        waista_cnt = 0;

    if(waista_cnt > 5)
    {
        mode = 2;
        waista_cnt = 0;

        if(PAN_POSITION > 0)
        {
            turn_cnt =3;
            waist_flag = -1;
        }

        else
        {
            turn_cnt =3;
            waist_flag = 1;
        }
    }

    waista_past = waista;

    if(missing_cnt > 50)
        mode = 2;


    if(mode == 1)   // Tracking
    {
        turn_cnt = 0;
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

        if(waist_flag != 0 && robocup_case != MODE_GOALPOST_DETECT)
        {
            if(detecting_cnt ++ > 10)
            {
                waist_flag = 0;
                temp_pan_position = PAN_POSITION;
                temp_tilt_position = TILT_POSITION;
                temp_case = robocup_case;
                robocup_case = SCAN;
                detecting_cnt = 0;
            }
        }

    }

    else if(mode == 2)  // Scan
    {
        if(PAN_Flag == 1) // Go To Right
        {
            PAN_POSITION = PAN_POSITION - 0.2;

            if(PAN_POSITION < -ballScanPanRange)
            {
                TILT_Flag = 1;
                TILT_UP_COUNT++;
            }
        }
        else if(PAN_Flag == 2) // Go To Left
        {
            PAN_POSITION = PAN_POSITION + 0.2;

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

        if(TILT_UP_COUNT >= 4)
        {
            TILT_POSITION = 0; // 80
            TILT_UP_COUNT = 0;
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

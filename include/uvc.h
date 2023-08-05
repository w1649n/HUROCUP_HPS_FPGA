#ifndef UVC_H_
#define UVC_H_

//#include "system.h"
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <fstream>
#include <vector>
#include <map>
#include <Eigen/Eigen>
#include "Inverse_kinematic.h"
#include "Fuzzy_Controller.h"
#include "Walkinggait.h"
#include "Sensor.h"
#include "kalman.h"
#include "math.h"
#include "DefineDataStruct.h"
#include "ZMPProcess.h"
#include "walking_balance.h"

using namespace std;

#define PI                  3.1415926535897932384626433832795   //pi
#define PI_2                1.5707963267948966192313216916398   //pi/2
#define Angle_2_PI          PI/180
#define PI_2_Angle          180/PI
#define DEGREE2RADIAN		(M_PI / 180.0)
#define RADIAN2DEGREE		(180.0/ M_PI)


#define RPY_ROLL_LIMIT 		0.785	//45 degree
#define RPY_PITCH_LIMIT 	0.785	//45 degree
#define RPY_STAND_RANGE 	0.52	//30 degree
/////////////////////////////////////////////////////////
class UVC
{
    public:
        UVC();
        ~UVC();
        void uvc_maincontrol();
        void uvc_first_postcontrol();
        void uvc_second_postcontrol();
        void load_imu();
        
    private:
        double pitcht,rollt;
        double pitch,roll,
        double pitch_gyrg,roll_gyrg;
        double wk,wt;
        double dyi,dyib,dyis;
        double dxi,dxib,dxis;
};

extern Initial init;
extern Walkinggait walkinggait;
extern Datamodule datamodule;
extern BalanceControl balance;
extern kickgait_space::KickingGait kickinggait;
extern SensorDataProcess sensor;

#endif /*UVC_H_*/

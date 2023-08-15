#ifndef WALKING_BALANCE_H_
#define WALKING_BALANCE_H_

//#include "system.h"
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <fstream>
#include <vector>
#include <map>
#include <Eigen/Eigen>
#include "Inverse_kinematic.h"
// #include "IMU_base_obs.h"
#include "Fuzzy_Controller.h"
#include "Walkinggait.h"
#include "Sensor.h"
// #include <kalman.h>
#include "math.h"
#include "DefineDataStruct.h"
#include "ZMPProcess.h"
class ZMPProcess;

#define PI                  3.1415926535897932384626433832795   //pi
#define PI_2                1.5707963267948966192313216916398   //pi/2
#define Angle_2_PI          PI/180
#define PI_2_Angle          180/PI
#define DEGREE2RADIAN		(M_PI / 180.0)
#define RADIAN2DEGREE		(180.0/ M_PI)

/*******************Function Define******************************/
#define Auto_Gyro_offset
/********************************************************/

#define GRAVITATIONAL_ACCELERATION 9.8	//9.8(m/s^2)
#define ANKLE_HEIGHT 2.6 //2.6(cm)
#define COM_HEIGHT_TO_GROUND (walkinggait.COM_HEIGHT + ANKLE_HEIGHT)
// #define IMU_HEIGHT 7.462 //7.462(cm)

/////////////////////////Posture///////////////////////
#define Acc_offset_X            	0
#define Acc_offset_Y            	0
#define Acc_offset_Z            	0
#define Gyro_manual_offset_X    	0
#define Gyro_manual_offset_Y    	0
#define Gyro_manual_offset_Z    	0
#define kalman_manual_offset_Pitch	0
#define kalman_manual_offset_Roll	0
#define kalman_manual_offset_Yaw	0
#define Gyro_LSB_2_Angle        	16.4
#define Acc_LSB_2_G					2048.0
#define index_Pitch             	0
#define index_Roll              	1
#define index_Yaw					2
#define Posture_Gain				0.3
////////////////////////////////////////////////////////

/////////////////////////Zmp Control///////////////////////
#define DOUBLE_FEET_WEIGHT_FAR_Y	7.5	//8.2//7.6 cm
#define DOUBLE_FEET_WEIGHT_NEAR_Y	3	//2.7//3.0 cm
#define DOUBLE_FEET_WEIGHT_X		4.5//4.15//4.4 cm
#define DOUBLE_FEET_BALANCE_POINT_Y	4.5	//4.5 cm
#define SINGLE_FOOT_WEIGHT_FAR_Y	3	//3.1  5.5
#define SINGLE_FOOT_WEIGHT_NEAR_Y	2	//1.5 cm 
#define SINGLE_FOOT_WEIGHT_X		4.15	//4.4 cm

#define RIGHT_PRESS_SHIFT	4
#define SINGLE_FOOT_WEIGHT_EQUAL_Y	2.75
///////////////////////////////////////////////////////////

//////////////////////for Fall down & Get up/////////////////////////////
#define RPY_ROLL_LIMIT 		1.133	//65 degree
#define RPY_PITCH_LIMIT 	1.133	//65 degree
#define RPY_STAND_RANGE 	0.52	//30 degree
/////////////////////////////////////////////////////////

enum class imu {roll = 0,pitch,yaw};
typedef enum {leftfoot = 0,rightfoot,doublefeet} etSupFoot;

class PID_Controller
{
public:
    PID_Controller();
    ~PID_Controller();
    void initParam();
    void setKpid(double Kp, double Ki, double Kd);
    void setControlGoal(float x1c = 0, float x2c = 0, float x3c = 0);
    void setValueLimit(float upper_limit, float lower_limit);
    float calculateExpValue(float value);
    float getError();
    float getErrors();
    float getErrord();
    // void setErrorValue(float error);
    // void setErrorsValue(float errors);
    // void setErrordValue(float errord);
    // float getFixValue();
// private:
    double Kp;
    double Ki;
    double Kd;
    float error;
    float pre_error;
    float errors;
    float errord;
    float x1c;
    float x2c;
    float x3c;
    float exp_value;
    float value;
    float pre_value;
    float upper_limit;
    float lower_limit;
};

typedef struct IMUParameter IMUParam;
struct IMUParameter
{
    float pos;
    float vel;
    void initialize()
    {
        pos = 0;
        vel = 0;
    }
    // float acc;
};

class ButterWorthParam
{
public:
    static ButterWorthParam set(float a1, float a2, float b1, float b2);
    float a_[2];
    float b_[2];
};

class ButterWorthFilter
{
public:
    ButterWorthFilter();
    ~ButterWorthFilter();
    void initialize(ButterWorthParam param);
    float getValue(float present_value);
private:
    ButterWorthParam param_;
    float prev_output_;
    float prev_value_;
};

typedef struct BalanceParameter BalanceParam;
struct BalanceParameter
{
    float control_value_total;
    float control_value_once;
    void initialize()
    {
        control_value_total = 0;
        control_value_once = 0;
    }
};



typedef struct ButterWorthIMUParameter ButterWorthIMUParam;
struct ButterWorthIMUParameter
{
    ButterWorthFilter pos; //座標
    ButterWorthFilter vel; //速度
    void initialize()
    {
        pos.initialize(ButterWorthParam::set(1, -0.676819, 0.161590, 0.161590)); //fs = 33 , fc = 2, n = 1;
        vel.initialize(ButterWorthParam::set(1, -0.676819, 0.161590, 0.161590)); //fs = 33 , fc = 2, n = 1;
    }
};

class BalanceLowPassFilter
{
public:
	BalanceLowPassFilter();
	~BalanceLowPassFilter();

	void initialize(double control_cycle_sec, double cut_off_frequency);
	void set_cut_off_frequency(double cut_off_frequency);
	double get_cut_off_frequency(void);
	double get_filtered_output(double present_raw_value);

private:
	double cut_off_freq_;
	double control_cycle_sec_;
	double alpha_;
	double prev_output_;
};

class BalanceControl
{
public:
	BalanceControl();
	~BalanceControl();
	
	//		Fall down & Get up	timer  for	count time
	struct timeval timer_start_, timer_end_;
    double timer_dt_;

	//	test
	void initialize(const int control_cycle_msec);
	void initialize_parameter();
	void p2h_get_parameter();
	void get_sensor_value();
	void balance_control();
	void control_after_ik_calculation();


	// LIPM
	void setSupportFoot();
	void resetControlValue();
	void endPointControl();
	void InitEndPointControl();


	BalanceLowPassFilter roll_imu_lpf_;
	BalanceLowPassFilter pitch_imu_lpf_;

	double roll_imu_filtered_;
	double pitch_imu_filtered_;
	bool roll_over_limit_;
	bool pitch_over_limit_;
	int DSP,SSP;
	double cog_roll_offset_;
	double cog_pitch_offset_;
	double foot_cog_x_;
	double foot_cog_y_;
	double original_ik_point_rz_, original_ik_point_lz_;
	double ankle_pitch_;
	float x_offset,y_offset_l,y_offset_r;
	//LIPM
	etSupFoot sup_foot_, pre_sup_foot_;

	IMUParam init_imu_value[3];				//初始IMU值
    IMUParam pres_imu_value[3];				//當前IMU值
    IMUParam prev_imu_value[3];				//前次IMU值
    IMUParam ideal_imu_value[3];			//理想IMU值
    IMUParam passfilter_pres_imu_value[3];	
    IMUParam passfilter_prev_imu_value[3];

	ButterWorthIMUParam butterfilter_imu[3];

	//hip
	BalanceParam leftfoot_hip_roll_value;
    BalanceParam leftfoot_hip_pitch_value;
	BalanceParam rightfoot_hip_roll_value;
	BalanceParam rightfoot_hip_pitch_value;
	PID_Controller PIDleftfoot_hip_roll;
    PID_Controller PIDleftfoot_hip_pitch;
	PID_Controller PIDrightfoot_hip_roll;
    PID_Controller PIDrightfoot_hip_pitch;
	//ankle
	BalanceParam leftfoot_ankle_roll_value;
    BalanceParam leftfoot_ankle_pitch_value;
	BalanceParam rightfoot_ankle_roll_value;
	BalanceParam rightfoot_ankle_pitch_value;
	PID_Controller PIDleftfoot_ankle_roll;
    PID_Controller PIDleftfoot_ankle_pitch;
	PID_Controller PIDrightfoot_ankle_roll;
    PID_Controller PIDrightfoot_ankle_pitch;
    
	PID_Controller PIDleftfoot_stand_pitch;
	PID_Controller PIDrightfoot_stand_pitch;
    
	BalanceParam leftfoot_EPx_value;	//EP = End point
	BalanceParam leftfoot_EPy_value;
	BalanceParam rightfoot_EPx_value;
	BalanceParam rightfoot_EPy_value;
	PID_Controller PIDleftfoot_zmp_x;
	PID_Controller PIDleftfoot_zmp_y;
	PID_Controller PIDrightfoot_zmp_x;
	PID_Controller PIDrightfoot_zmp_y;

	BalanceParam CoM_EPx_value;
    BalanceParam CoM_EPy_value;

	PID_Controller PIDCoM_x;
    PID_Controller PIDCoM_y;
	PID_Controller PIDCoM_z;

	ZMPParam pres_ZMP;
    ZMPParam prev_ZMP;
    ZMPParam ideal_ZMP;

	ZMPProcess *ZMP_process;

	float foot_pitch;
	float leftfoot_hip_roll;
    float leftfoot_hip_pitch;
    float leftfoot_ankle_roll;
    float leftfoot_ankle_pitch;
	float rightfoot_hip_roll;
    float rightfoot_hip_pitch;
    float rightfoot_ankle_roll;
    float rightfoot_ankle_pitch;

    double imu_desire_[3];
    double roll_pid_[3];
    double pitch_pid_[3];
    double com_pid_[3];
    double foot_offset_[3];
    double kalman_rpy_[2];
    double pre_kalman_rpy_[2];

	// for debug
	int now_step_, last_step_;
	void saveData();
    std::string DtoS(double value);
	std::map<std::string, float> map_param;
    std::map<std::string, std::vector<float>> map_roll;
    std::map<std::string, std::vector<float>> map_pitch;
	std::map<std::string, std::vector<float>> map_ZMP;
	std::map<std::string, std::vector<float>> map_CoM;
	std::map<std::string, std::vector<float>> map_Accel;

	int name_cont_;
    int pitch_count_=0, roll_count_=0;
	float tmp_total;
	float tmp;
	float tmp_com;
	float tmp_com_total;
	bool support_flag_l = true,support_flag_r = true;
	bool set_offset_; 
	bool flag_r = false ,flag_l = false , balance_time;

	//LIPM end
    //kalman
    double get_angle(double, double, double, int);
    double get_force(double, int);
    double get_q_bias();
    struct timeval kalman_start,kalman_end;
    // private:
        double p_[3][2][2];
        double k_[2];

        double q_angle_;
        double q_bias_;
        double r_measure_;

        double angle_[3];
        double bias_[3];

        double force_r_[8];
        double force_q_;
        double force_w_;  

        double force_x_[8];
        double force_p_[8];
        double force_k_[8];
    //
};

#endif
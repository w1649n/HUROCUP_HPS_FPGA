/*
 * walkinggait.h
 *
 *  Created on: 2019/09/21
 *      Author: Yu-Chih, Wang
 */

#ifndef WALKINGGAIT_H_
#define WALKINGGAIT_H_

#include <stdio.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include "Feedback_Motor.h"
#include "Initial.h"
#include "WalkingCycle.h"
#include "WalkingTrajectory.h"
#include "KickingGait.h"
// #include <std_msgs/String.h>

#define WALKING_INTERVAL 30000.0   //30 ms
#define STARTSTEPCOUNTER 2
typedef enum
{
    etStart = 0x01,
    etStop = 0x02,
    etChangeValue = 0x04
}etWalkingCmd;

typedef enum
{
    Single = 0,
    Continuous,
    LC_up,
    LC_down,
    Long_Jump,
    RKickB = 9,
    LKickB
    
}etWalkingStatus;

class WalkingGaitByLIPM
{
public:
    WalkingGaitByLIPM();
    ~WalkingGaitByLIPM();

    void initialize();
    void readWalkData();
    void readWalkParameter();
    void resetParameter();
    void process();
    void LCup();
    void LCdown();
    void final_step();
    void coordinate_transformation();
    void coordinate_offset();

    double com_x,com_y;
    void setcom_pos(double set_com_x, double set_com_y){
        com_x = set_com_x;
        com_y = set_com_y;
    }

    // double wtestComVelocity(double pre_x,double pre_v,double px,double t ,double T);
    double wComVelocityInit(double x0, double xt, double px, double t, double T);
    double wComPosition(double x0, double vx0, double px, double t, double T);
    double wFootPosition(const double start, const double length, const double t, const double T, const double T_DSP);
    double wFootPositionRepeat(const double start, const double length, const double t, const double T, const double T_DSP);
    double wFootPositionZ(const double height, const double t, const double T, const double T_DSP);
    double wFootTheta(const double theta, bool reverse, const double t, const double T, const double T_DSP);

    double unit_step(double x);
    double sinh(double x);
    double cosh(double x);

    //for test
    string DtoS(double value);
    void saveData();

// private:
    bool is_parameter_load_;
    bool ready_to_stop_;
    bool delay_push_;
    bool push_data_;
    int count;
    double step_times,first_change_foot_time;
    float rightfoot_shift_z,com_y_swing;
    int moving_state_;
    int period_t_;
    int time_point_, sample_point_, sample_time_;
    int now_step_, pre_step_;
    int StartStepCount_, StartHeight_;
    int g_;
    double T_DSP_;
    int step_, left_step_, right_step_;
    double TT_, t_,Ts;
    double Tc_;
    double step_length_, width_size_, lift_height_, base_x, now_width_, width_x, width_y;
    double theta_, var_theta_, abs_theta_, last_theta_, last_abs_theta_;
    double last_displacement_x, now_left_x_, now_right_x_, last_base_x;
    double shift_length_, last_displacement_y, base_y, now_left_y_, now_right_y_, last_base_y;
    double footstep_x, footstep_y;
    double zmp_x, zmp_y, last_zmp_x, last_zmp_y;
    double displacement_x, displacement_y;
    double step_point_lx_W_, step_point_rx_W_, step_point_ly_W_, step_point_ry_W_;
    double step_point_lx_, step_point_rx_, step_point_ly_, step_point_ry_;
    double step_point_lz_, step_point_rz_, step_point_lthta_, step_point_rthta_;
    double end_point_lx_, end_point_rx_, end_point_ly_, end_point_ry_;
    double end_point_lz_, end_point_rz_, end_point_lthta_, end_point_rthta_;
    double vx0_, vy0_, px_, py_, pz_,py_u,px_u;
    double lpx_, rpx_, lpy_, rpy_, lpz_, rpz_, lpt_, rpt_;
    double foot_hight, board_hight, c_hight;
    // double COM_HEIGHT = 26.5;
    int cnt = 0;
    //for stepping
    double Control_Step_length_X_,Control_Step_length_Y_;
    bool Stepout_flag_X_,Stepout_flag_Y_;
    int Step_Count_;    
    // double test_v0_save;

    bool plot_once_, if_finish_;
    
    int name_cont_;
	std::map<std::string, std::vector<float>> map_walk;
};

class Walkinggait : public WalkingGaitByLIPM
{
public:
    Walkinggait();
    ~Walkinggait();

    void load_parameter();
    void update_parameter();
    void load_walkdata();
    void update_walkdata(); 
    void calculate_point_trajectory();
    void walking_timer();
    void pushData();

    double x_, y_, z_, thta_;
    char walking_cmd_, sensor_mode_;
    
    struct timeval timer_start_, timer_end_;
    double timer_dt_;
    bool get_parameter_flag_;
    bool get_walkdata_flag_;
    bool locus_flag_;
    bool LIPM_flag_;
    int motion_delay_;
    int pre_walking_mode;
    
    

private:
    bool update_parameter_flag_;
    bool update_walkdata_flag_;
    bool continuous_stop_flag_;
    int parameter_[6];
    int walkdata_[3];
};

#endif /* WALKINGGAIT_H_ */
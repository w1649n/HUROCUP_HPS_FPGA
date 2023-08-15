#include "include/walking_balance.h"
#include "include/data_txt.h"

FuzzyController fuzzy;
extern struct Points_Struct Points;
extern Locus locus;
extern SensorDataProcess sensor;
extern Walkinggait walkinggait;
// extern KalmanFilter kalman;

BalanceControl::BalanceControl()
{
	original_ik_point_rz_ = 0.0;
	original_ik_point_lz_ = 0.0;
	last_step_ = StopStep;
	now_step_ = StopStep;
	sup_foot_ = doublefeet;
	name_cont_ = 0;
	ZMP_process = new ZMPProcess;

    
}

BalanceControl::~BalanceControl()
{
	delete ZMP_process;
}

void BalanceControl::initialize(const int control_cycle_msec)
{
	for(int i = 0; i < sizeof(init_imu_value)/sizeof(init_imu_value[0]); i++)
        init_imu_value[i].initialize();
    for(int i = 0; i < sizeof(pres_imu_value)/sizeof(pres_imu_value[0]); i++)
        pres_imu_value[i].initialize();
    for(int i = 0; i < sizeof(prev_imu_value)/sizeof(prev_imu_value[0]); i++)
        prev_imu_value[i].initialize();
    for(int i = 0; i < sizeof(ideal_imu_value)/sizeof(ideal_imu_value[0]); i++)
        ideal_imu_value[i].initialize();
    for(int i = 0; i < sizeof(passfilter_pres_imu_value)/sizeof(passfilter_pres_imu_value[0]); i++)
        passfilter_pres_imu_value[i].initialize();
    for(int i = 0; i < sizeof(passfilter_prev_imu_value)/sizeof(passfilter_prev_imu_value[0]); i++)
        passfilter_prev_imu_value[i].initialize();

	//kalman
    q_angle_ = 0.003;
    q_bias_ = 0.005;
    r_measure_ = 0.03;//0.1;//0.03;    //0.0005
    
	angle_[0] = 0.0;
	angle_[1] = 0.0;
	angle_[2] = 0.0;
	bias_[0] = 0.0;
	bias_[1] = 0.0;
	bias_[2] = 0.0;

    leftfoot_hip_roll_value.initialize();
    leftfoot_hip_pitch_value.initialize();
	rightfoot_hip_roll_value.initialize();
    rightfoot_hip_pitch_value.initialize();
	leftfoot_ankle_roll_value.initialize();
    leftfoot_ankle_pitch_value.initialize();
	rightfoot_ankle_roll_value.initialize();
    rightfoot_ankle_pitch_value.initialize();
	
	CoM_EPx_value.initialize();
	CoM_EPy_value.initialize();

    PIDleftfoot_hip_roll.initParam();
    PIDleftfoot_hip_pitch.initParam();
	PIDrightfoot_hip_roll.initParam();
    PIDrightfoot_hip_pitch.initParam();

	PIDleftfoot_ankle_roll.initParam();
	PIDleftfoot_ankle_pitch.initParam();
	PIDrightfoot_ankle_roll.initParam();
	PIDrightfoot_ankle_pitch.initParam();
 
	PIDleftfoot_stand_pitch.initParam();
	PIDrightfoot_stand_pitch.initParam();
	
	PIDleftfoot_zmp_x.initParam();
	PIDleftfoot_zmp_y.initParam();
	PIDrightfoot_zmp_x.initParam();
	PIDrightfoot_zmp_y.initParam();
	PIDCoM_x.initParam();

    for(int i = 0; i < sizeof(butterfilter_imu)/sizeof(butterfilter_imu[0]); i++)
        butterfilter_imu[i].initialize();
		
	for(int i = 0; i < sizeof(ideal_p_arry)/sizeof(ideal_p_arry[0]); i++)
    { 
        ideal_p_arry_roll[i] = ideal_p_arry[i];
        ideal_p_arry_pitch[i] = ideal_p_arry[i];
    }

	//ankle
    PIDleftfoot_ankle_roll.setValueLimit(3, -3);
    PIDleftfoot_ankle_pitch.setValueLimit(3, -3);
    PIDleftfoot_ankle_roll.setKpid(0.05,0,0);
    PIDleftfoot_ankle_pitch.setKpid(0.05, 0, 0);
    // PIDleftfoot_ankle_roll.setControlGoal(init_imu_value[(int)imu::roll].pos);
    // PIDleftfoot_ankle_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

	PIDrightfoot_ankle_roll.setValueLimit(3, -3);
    PIDrightfoot_ankle_pitch.setValueLimit(3, -3);
    PIDrightfoot_ankle_roll.setKpid(0.05,0,0);
    PIDrightfoot_ankle_pitch.setKpid(0.05, 0, 0);
    
    // PIDrightfoot_ankle_roll.setControlGoal(init_imu_value[(int)imu::roll].pos);
    // PIDrightfoot_ankle_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

    PIDleftfoot_stand_pitch.setValueLimit(0.5, -0.5);
    PIDleftfoot_stand_pitch.setKpid(0.03, 0, 0.02);//(0.03, 0, 0.02);  //0.03, 0, 0.02
    PIDleftfoot_stand_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

	PIDrightfoot_stand_pitch.setValueLimit(0.5, -0.5);
    PIDrightfoot_stand_pitch.setKpid(0.03, 0, 0.02);//(0.03, 0, 0.02);  //0.03, 0, 0.02
    PIDrightfoot_stand_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);


	imu_desire_[0] = 0;
	imu_desire_[1] = 0;
	imu_desire_[2] = 0;
	roll_pid_[0] = 0;
	roll_pid_[1] = 0;
	roll_pid_[2] = 0;
	pitch_pid_[0] = 0;
	pitch_pid_[1] = 0;
	pitch_pid_[2] = 0;
	com_pid_[0] = 0;
	com_pid_[1] = 0;
	com_pid_[2] = 0;
	foot_offset_[0] = 0;
	foot_offset_[1] = 0;

    initialize_parameter();

	leftfoot_hip_roll = 0;
    leftfoot_hip_pitch = 0;
    leftfoot_ankle_roll = 0;
    leftfoot_ankle_pitch = 0;
	rightfoot_hip_roll = 0;
    rightfoot_hip_pitch = 0;
    rightfoot_ankle_roll = 0;
    rightfoot_ankle_pitch = 0;

	for(int i = 0; i < 3; i++)init_imu_value[i].pos = sensor.rpy_[i];
	pre_kalman_rpy_[0] = 0;
	pre_kalman_rpy_[1] = 0;

	std::vector<float> temp;
	if(map_roll.empty())
	{
		map_roll["init_roll_pos"] = temp;
        map_roll["smaple_times_count"] = temp;
        map_roll["pres_roll_pos"] = temp;
        map_roll["passfilter_pres_roll_pos"] = temp;
        map_roll["ideal_roll_vel"] = temp;
        map_roll["pres_roll_vel"] = temp;
        map_roll["passfilter_pres_roll_vel"] = temp;
        map_roll["left_control_once_roll"] = temp;
        map_roll["left_control_total_roll"] = temp;
		map_roll["right_control_once_roll"] = temp;
        map_roll["right_control_total_roll"] = temp;
		map_roll["leftfoot_hip_roll"] = temp;
		map_roll["rightfoot_hip_roll"] = temp;
		map_roll["support_foot"] = temp;
	}

	if(map_pitch.empty())
	{
		// map_pitch["init_pitch_pos"] = temp;
        map_pitch["smaple_times_count"] = temp;
        map_pitch["pres_pitch_pos"] = temp;
        map_pitch["passfilter_pres_pitch_pos"] = temp;
        map_pitch["ideal_pitch_vel"] = temp;
        map_pitch["pres_pitch_vel"] = temp;
        map_pitch["passfilter_pres_pitch_vel"] = temp;
        map_pitch["left_control_once_pitch"] = temp;
        map_pitch["left_control_total_pitch"] = temp;
		map_pitch["right_control_once_pitch"] = temp;
        map_pitch["right_control_total_pitch"] = temp;

        map_pitch["left_control_once_pitch_ankle"] = temp;
        map_pitch["left_control_total_pitch_ankle"] = temp;
		map_pitch["right_control_once_pitch_ankle"] = temp;
        map_pitch["right_control_total_pitch_ankle"] = temp;
	}

	if(map_ZMP.empty())
	{
		map_ZMP["pres_ZMP_left_pos_x"] = temp;
        map_ZMP["pres_ZMP_left_pos_y"] = temp;
        map_ZMP["pres_ZMP_right_pos_x"] = temp;
        map_ZMP["pres_ZMP_right_pos_y"] = temp;
        map_ZMP["pres_ZMP_feet_pos_x"] = temp;
        map_ZMP["pres_ZMP_feet_pos_y"] = temp;

        map_ZMP["delta_v"] = temp;

        map_ZMP["raw_sensor_data_0"] = temp;
        map_ZMP["raw_sensor_data_1"] = temp;
        map_ZMP["raw_sensor_data_2"] = temp;
        map_ZMP["raw_sensor_data_3"] = temp;
        map_ZMP["raw_sensor_data_4"] = temp;
        map_ZMP["raw_sensor_data_5"] = temp;
        map_ZMP["raw_sensor_data_6"] = temp;
        map_ZMP["raw_sensor_data_7"] = temp;

        map_ZMP["sensor_force_0"] = temp;
        map_ZMP["sensor_force_1"] = temp;
        map_ZMP["sensor_force_2"] = temp;
        map_ZMP["sensor_force_3"] = temp;
        map_ZMP["sensor_force_4"] = temp;
        map_ZMP["sensor_force_5"] = temp;
        map_ZMP["sensor_force_6"] = temp;
        map_ZMP["sensor_force_7"] = temp;

		map_ZMP["leftfoot_control_once_EPx"] = temp;
		map_ZMP["leftfoot_control_once_EPy"] = temp;
		map_ZMP["leftfoot_control_total_EPx"] = temp;
		map_ZMP["leftfoot_control_total_EPy"] = temp;

		map_ZMP["rightfoot_control_once_EPx"] = temp;
		map_ZMP["rightfoot_control_once_EPy"] = temp;
		map_ZMP["rightfoot_control_total_EPx"] = temp;
		map_ZMP["rightfoot_control_total_EPy"] = temp;

		map_ZMP["new_EP_lx"] = temp;
		map_ZMP["new_EP_rx"] = temp;
		map_ZMP["new_EP_ly"] = temp;
		map_ZMP["new_EP_ry"] = temp;
	}

	if(map_CoM.empty())
	{
		map_CoM["CoM_x_control"] = temp;
		map_CoM["new_EP_lx"] = temp;
		map_CoM["new_EP_rx"] = temp;
	}

	if(map_Accel.empty())
	{
		map_Accel["Accel_ax"] = temp;
		map_Accel["Accel_ay"] = temp;
		map_Accel["Accel_az"] = temp;
		map_Accel["Gyro_x"] = temp;
		map_Accel["Gyro_y"] = temp;
		map_Accel["Gyro_z"] = temp;
		map_Accel["Step_out_Y"] = temp;
		map_Accel["Step_out_X"] = temp;
		map_Accel["Step_out_Y_length"] = temp;
		map_Accel["Step_out_X_length"] = temp;

	}
	map_roll.find("init_roll_pos")->second.push_back(init_imu_value[(int)imu::roll].pos);
	// map_pitch.find("init_pitch_pos")->second.push_back(init_imu_value[(int)imu::pitch].pos);
}

void BalanceControl::initialize_parameter()
{
	PIDleftfoot_hip_roll.setValueLimit(300, -300);
    PIDleftfoot_hip_pitch.setValueLimit(300, -300);
    PIDleftfoot_hip_roll.setKpid(roll_pid_[0], roll_pid_[1], roll_pid_[2]);//(0, 0, 0);//(0.005,0,0.003);//(0.03, 0.01, 0.01);//(0.03, 0.01, 0.03); //0.02, 0.01, 0.01 //0.03, 0, 0.02
    PIDleftfoot_hip_pitch.setKpid(pitch_pid_[0], pitch_pid_[1], pitch_pid_[2]);//(0.02, 0, 0.005);//(0.03, 0, 0.02);  //0.03, 0, 0.02
    // PIDleftfoot_hip_roll.setControlGoal(imu_desire_[0]);//(init_imu_value[(int)imu::roll].pos); imu_desire_[0]
    // PIDleftfoot_hip_pitch.setControlGoal(imu_desire_[1]);//(init_imu_value[(int)imu::pitch].pos);imu_desire_[1]

	PIDrightfoot_hip_roll.setValueLimit(300, -300);
    PIDrightfoot_hip_pitch.setValueLimit(300, -300);
    PIDrightfoot_hip_roll.setKpid(roll_pid_[0], roll_pid_[1], roll_pid_[2]);//(0, 0, 0);//(0.005,0,0.003);//(0.03, 0.01, 0.01);//(0.03, 0.01, 0.03); //0.02, 0.01, 0.01 //0.03, 0, 0.02
    PIDrightfoot_hip_pitch.setKpid(pitch_pid_[0], pitch_pid_[1], pitch_pid_[2]);//(0.02, 0, 0.005);//(0.03, 0, 0.02);  //0.03, 0, 0.02
    // PIDrightfoot_hip_roll.setControlGoal(imu_desire_[0]);//(init_imu_value[(int)imu::roll].pos); imu_desire_[0]
    // PIDrightfoot_hip_pitch.setControlGoal(imu_desire_[1]);//(init_imu_value[(int)imu::pitch].pos); imu_desire_[1]


    PIDrightfoot_ankle_roll.setKpid(0.01,0,0);
    PIDrightfoot_ankle_pitch.setKpid(pitch_pid_[0], pitch_pid_[1], pitch_pid_[2]);
    PIDleftfoot_ankle_roll.setKpid(0.01,0,0);
    PIDleftfoot_ankle_pitch.setKpid(pitch_pid_[0], pitch_pid_[1], pitch_pid_[2]);
	//ankle
    // PIDleftfoot_ankle_roll.setValueLimit(0.1, -0.1);
    // PIDleftfoot_ankle_pitch.setValueLimit(0.1, -0.1);
    // PIDleftfoot_ankle_roll.setKpid(roll_pid_[0], roll_pid_[1], roll_pid_[2]);
    // PIDleftfoot_ankle_pitch.setKpid(roll_pid_[0], roll_pid_[1], roll_pid_[2]);
    // PIDleftfoot_ankle_roll.setControlGoal(init_imu_value[(int)imu::roll].pos);
    // PIDleftfoot_ankle_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

	// PIDrightfoot_ankle_roll.setValueLimit(0.075, -0.075);
    // PIDrightfoot_ankle_pitch.setValueLimit(0.075, -0.075);
    // PIDrightfoot_ankle_roll.setKpid(roll_pid_[0], roll_pid_[1], roll_pid_[2]);
    // PIDrightfoot_ankle_pitch.setKpid(roll_pid_[0], roll_pid_[1], roll_pid_[2]);

	PIDleftfoot_zmp_x.setValueLimit(7, -7);
	PIDleftfoot_zmp_y.setValueLimit(7, -7);
	PIDleftfoot_zmp_x.setKpid(0.0125, 0, 0);  //0.0125, 0, 0.02
	PIDleftfoot_zmp_y.setKpid(0.0125, 0, 0);  //0.0125, 0, 0.02
	PIDleftfoot_zmp_x.setControlGoal(0);
	PIDleftfoot_zmp_y.setControlGoal(5);

	PIDrightfoot_zmp_x.setValueLimit(7, -7);
	PIDrightfoot_zmp_y.setValueLimit(7, -7);
	PIDrightfoot_zmp_x.setKpid(0.0125, 0, 0);  //0.0125, 0, 0.02
	PIDrightfoot_zmp_y.setKpid(0.0125, 0, 0);  //0.0125, 0, 0.02
	PIDrightfoot_zmp_x.setControlGoal(0);
	PIDrightfoot_zmp_y.setControlGoal(-5);
	
	PIDCoM_x.setValueLimit(7, -7);
	PIDCoM_x.setKpid(com_pid_[0], com_pid_[1], com_pid_[2]);//(0.03, 0, 0.02);  //0.03, 0, 0.02
	PIDCoM_x.setControlGoal(0);//(0);imu_desire_[2]

	PIDCoM_y.setValueLimit(7, -7);
	PIDCoM_y.setKpid(0.001, 0, 0.002);//(0.03, 0, 0.02);  //0.03, 0, 0.02
}

void BalanceControl::p2h_get_parameter()
{
	if(sensor.gain_set_)
        memcpy(imu_desire_, sensor.imu_desire_, sizeof(sensor.imu_desire_));
    else if(sensor.roll_PID_set_)
        memcpy(roll_pid_, sensor.roll_pid_, sizeof(sensor.roll_pid_));
    else if(sensor.pitch_PID_set_)
        memcpy(pitch_pid_, sensor.pitch_pid_, sizeof(sensor.pitch_pid_));
    else if(sensor.com_PID_set_)
        memcpy(com_pid_, sensor.com_pid_, sizeof(sensor.com_pid_));
    else if(sensor.foot_offset_set_)
        memcpy(foot_offset_, sensor.foot_offset_, sizeof(sensor.foot_offset_));
	initialize_parameter();
// 	cout<<"imu_desire_ = "<<imu_desire_[0]<<", "<<imu_desire_[1]<<", "<<imu_desire_[2]<<endl;
//         cout<<"roll_pid_ = "<<roll_pid_[0]<<", "<<roll_pid_[1]<<", "<<roll_pid_[2]<<endl;
//         cout<<"pitch_pid_ = "<<pitch_pid_[0]<<", "<<pitch_pid_[1]<<", "<<pitch_pid_[2]<<endl;
//         cout<<"com_pid_ = "<<com_pid_[0]<<", "<<com_pid_[1]<<", "<<com_pid_[2]<<endl;
//         cout<<"foot_offset_ = "<<foot_offset_[0]<<", "<<foot_offset_[1]<<", "<<foot_offset_[2]<<endl;
}

void BalanceControl::get_sensor_value()
{
    p2h_get_parameter();
	int i;
	double rpy_radian[3] = {0};
    double gyro_[3] = {0};
    double acc_[3] = {0};
	double  kalman_timer;
	

	gettimeofday(&kalman_end, NULL);
	kalman_timer = (double)(1000000.0 * (kalman_end.tv_sec - kalman_start.tv_sec) + (kalman_end.tv_usec - kalman_start.tv_usec));
	roll_imu_lpf_.initialize(0.01, 1);
	pitch_imu_lpf_.initialize(0.01, 1);
	for(i=0; i<2; i++)
    {
        kalman_rpy_[i]  = get_angle(sensor.rpy_[i] ,sensor.gyro_[i],0.005,i);
		if(abs(pre_kalman_rpy_[i] - kalman_rpy_[i])<5)
			pre_kalman_rpy_[i] =  kalman_rpy_[i];
		else
			kalman_rpy_[i] = pre_kalman_rpy_[i];
    }
	gettimeofday(&kalman_start, NULL);
	roll_imu_filtered_ = roll_imu_lpf_.get_filtered_output(sensor.rpy_[0]);
	pitch_imu_filtered_ = pitch_imu_lpf_.get_filtered_output(sensor.rpy_[1]);
	roll_over_limit_ = (fabs(roll_imu_filtered_) > 7 ? true : false);
	pitch_over_limit_ = (fabs(pitch_imu_filtered_) > 5 ? true : false);

	cog_roll_offset_ 	  = 0;//sensor.imu_desire_[0] ;
	cog_pitch_offset_ 	  = 0;//sensor.imu_desire_[1] ;
	double cog_y_filtered = kalman_rpy_[0] - roll_imu_filtered_;
	double cog_x_filtered = kalman_rpy_[1] - pitch_imu_filtered_;
	
	// if(cog_x_filtered>2){foot_cog_x_ = cog_x_filtered-1.5;}
	// else if(cog_x_filtered<-2){foot_cog_x_ = cog_x_filtered+1.5;}
	// else
	// {
	// 	foot_cog_x_ = 0;
	// }
	// if(cog_y_filtered>2){foot_cog_y_ = cog_y_filtered-2;}
	// else if(cog_y_filtered<-2){foot_cog_y_ = cog_y_filtered+2;}
	// else
	// {
	// 	foot_cog_y_ = 0;
	// }
    foot_cog_x_ = cog_x_filtered;
    foot_cog_y_ = cog_y_filtered;
}

void BalanceControl::setSupportFoot()
{
	pre_sup_foot_ = sup_foot_;
	if(walkinggait.time_point_% parameterinfo->parameters.Period_T >= parameterinfo->parameters.DSP && walkinggait.time_point_% parameterinfo->parameters.Period_T < (parameterinfo->parameters.DSP + parameterinfo->parameters.SSP))
	{//單支撐期間
		if(parameterinfo->parameters.foot_flag){sup_foot_ = rightfoot;}
		else{sup_foot_ = leftfoot;}
	}
	else{sup_foot_ = doublefeet;resetControlValue();}
}

void BalanceControl::resetControlValue()
{
	leftfoot_hip_pitch_value.initialize();
	leftfoot_hip_roll_value.initialize();
	rightfoot_hip_pitch_value.initialize();
	rightfoot_hip_roll_value.initialize();
	
	leftfoot_ankle_pitch_value.initialize();
	leftfoot_ankle_roll_value.initialize();
	rightfoot_ankle_pitch_value.initialize();
	rightfoot_ankle_roll_value.initialize();

	CoM_EPx_value.initialize();
	CoM_EPy_value.initialize();

	leftfoot_EPx_value.initialize();
	leftfoot_EPy_value.initialize();
	rightfoot_EPx_value.initialize();
	rightfoot_EPy_value.initialize();
}

void BalanceControl::endPointControl()
{
	int raw_sensor_data_tmp[8];
	for(int i=0; i<4; i++)raw_sensor_data_tmp[i] = sensor.press_left_[i];
	for(int i=4; i<8; i++)raw_sensor_data_tmp[i] = sensor.press_right_[i-4];
	prev_ZMP.feet_pos.x += pres_ZMP.feet_pos.x;
	prev_ZMP.feet_pos.y += pres_ZMP.feet_pos.y;
	ZMP_process->setpOrigenSensorData(raw_sensor_data_tmp);
	pres_ZMP = ZMP_process->getZMPValue();

	map_ZMP.find("pres_ZMP_left_pos_x")->second.push_back(pres_ZMP.left_pos.x);
	map_ZMP.find("pres_ZMP_left_pos_y")->second.push_back(pres_ZMP.left_pos.y);
	map_ZMP.find("pres_ZMP_right_pos_x")->second.push_back(prev_ZMP.feet_pos.x);
	map_ZMP.find("pres_ZMP_right_pos_y")->second.push_back(prev_ZMP.feet_pos.y);
	map_ZMP.find("pres_ZMP_feet_pos_x")->second.push_back(pres_ZMP.feet_pos.x);
	map_ZMP.find("pres_ZMP_feet_pos_y")->second.push_back(pres_ZMP.feet_pos.y);
	
	// pres_ZMP.feet_pos.x = foot_cog_x_;
	// pres_ZMP.feet_pos.y = foot_cog_y_;

	double *sensor_force = ZMP_process->getpSensorForce();
	int *raw_sensor_data = ZMP_process->getpOrigenSensorData();

	map_ZMP.find("sensor_force_0")->second.push_back(sensor.gyro_[0]);
	map_ZMP.find("sensor_force_1")->second.push_back(sensor.gyro_[1]);
	map_ZMP.find("sensor_force_2")->second.push_back(sensor.gyro_[2]);
	map_ZMP.find("sensor_force_3")->second.push_back(roll_pid_[0]);
	map_ZMP.find("sensor_force_4")->second.push_back(roll_pid_[1]);
	map_ZMP.find("sensor_force_5")->second.push_back(roll_pid_[2]);
	map_ZMP.find("sensor_force_6")->second.push_back(pitch_pid_[0]);
	map_ZMP.find("sensor_force_7")->second.push_back(pitch_pid_[1]);

	map_ZMP.find("raw_sensor_data_0")->second.push_back(pitch_pid_[2]);
	map_ZMP.find("raw_sensor_data_1")->second.push_back(sensor.rpy_[0]);
	map_ZMP.find("raw_sensor_data_2")->second.push_back(sensor.rpy_[1]);
	map_ZMP.find("raw_sensor_data_3")->second.push_back(sensor.rpy_[2]);
	map_ZMP.find("raw_sensor_data_4")->second.push_back(kalman_rpy_[0]);
	map_ZMP.find("raw_sensor_data_5")->second.push_back(kalman_rpy_[1]);
	map_ZMP.find("raw_sensor_data_6")->second.push_back(foot_cog_y_);
	map_ZMP.find("raw_sensor_data_7")->second.push_back(foot_cog_x_);


	map_ZMP.find("leftfoot_control_once_EPx")->second.push_back(leftfoot_EPx_value.control_value_once);
	map_ZMP.find("leftfoot_control_total_EPx")->second.push_back(leftfoot_EPx_value.control_value_total);
	map_ZMP.find("rightfoot_control_once_EPx")->second.push_back(rightfoot_EPx_value.control_value_once);
	map_ZMP.find("rightfoot_control_total_EPx")->second.push_back(rightfoot_EPx_value.control_value_total);


	map_ZMP.find("new_EP_lx")->second.push_back(parameterinfo->points.IK_Point_LX);
	map_ZMP.find("new_EP_rx")->second.push_back(parameterinfo->points.IK_Point_RX);
	map_ZMP.find("new_EP_ly")->second.push_back(parameterinfo->points.IK_Point_LY);
	map_ZMP.find("new_EP_ry")->second.push_back(parameterinfo->points.IK_Point_RY);
}

void BalanceControl::balance_control()
{
	p2h_get_parameter();
	int i;
	// LinearAlgebra LA;
	// int Change_Step_Y,Change_Step_X;

	//get accel for test and save data 
	map_Accel.find("Accel_ax")->second.push_back(sensor.accel_[0]);
	map_Accel.find("Accel_ay")->second.push_back(sensor.accel_[1]);
	map_Accel.find("Accel_az")->second.push_back(sensor.accel_[2]);
	map_Accel.find("Gyro_x")->second.push_back((double)sensor.gyro_[0]);
	map_Accel.find("Gyro_y")->second.push_back((double)sensor.gyro_[1]);
	map_Accel.find("Gyro_z")->second.push_back((double)sensor.gyro_[2]);
	// original_ik_point_rz_ = parameterinfo->points.IK_Point_RZ;
	// original_ik_point_lz_ = parameterinfo->points.IK_Point_LZ;

	int raw_sensor_data_tmp[8];
	for(int i=0; i<4; i++)raw_sensor_data_tmp[i] = sensor.press_left_[i];
	for(int i=4; i<8; i++)raw_sensor_data_tmp[i] = sensor.press_right_[i-4];
	prev_ZMP = pres_ZMP;
	ZMP_process->setpOrigenSensorData(raw_sensor_data_tmp);
	pres_ZMP = ZMP_process->getZMPValue();

	for(i=0; i<3; i++)prev_imu_value[i].pos = pres_imu_value[i].pos;
    for(i=0; i<3; i++)pres_imu_value[i].pos = sensor.rpy_[i];
 
	double Accel_limit = 1;
	int Step_Gain = 3;
	if(walkinggait.Stepout_flag_Y_)
	{
		walkinggait.Control_Step_length_Y_ = walkinggait.Control_Step_length_Y_;
		walkinggait.Stepout_flag_Y_ = walkinggait.Stepout_flag_Y_;
	}
	else
	{
		if(fabs(sensor.accel_[0]) > Accel_limit)
		{
			if(sensor.accel_[0] > Accel_limit)//go left
			{
				walkinggait.Control_Step_length_Y_ = (sensor.accel_[0]  * Step_Gain);
				walkinggait.Stepout_flag_Y_ = true;
			}
			else if (sensor.accel_[0] < -Accel_limit)//go right
			{
				walkinggait.Control_Step_length_Y_ = (sensor.accel_[0] * Step_Gain);
				walkinggait.Stepout_flag_Y_ = true;
			}
			else//
			{
				walkinggait.Control_Step_length_Y_ = walkinggait.Control_Step_length_Y_;
				walkinggait.Stepout_flag_Y_ = walkinggait.Stepout_flag_Y_;
			}
		}
	}
	map_Accel.find("Step_out_Y")->second.push_back(walkinggait.Stepout_flag_Y_);
	map_Accel.find("Step_out_Y_length")->second.push_back(walkinggait.Control_Step_length_Y_);

	if(walkinggait.Stepout_flag_X_)
	{
		walkinggait.Control_Step_length_X_ = walkinggait.Control_Step_length_X_;
		walkinggait.Stepout_flag_X_ = walkinggait.Stepout_flag_X_;
	}
	else
	{
		if(fabs(sensor.accel_[1]) > Accel_limit)
		{
			if(sensor.accel_[1] > Accel_limit)//go forward
			{
				walkinggait.Control_Step_length_X_ = (sensor.accel_[1]  * Step_Gain);
				walkinggait.Stepout_flag_X_ = true;
			}
			else if (sensor.accel_[1] < -Accel_limit)//go backward
			{
				walkinggait.Control_Step_length_X_ = (sensor.accel_[1] * Step_Gain);
				walkinggait.Stepout_flag_X_ = true;
			}
			else//
			{
				walkinggait.Control_Step_length_X_ = walkinggait.Control_Step_length_X_;
				walkinggait.Stepout_flag_X_ = walkinggait.Stepout_flag_X_;
			}
		}
	}
	map_Accel.find("Step_out_X")->second.push_back(walkinggait.Stepout_flag_X_);
	map_Accel.find("Step_out_X_length")->second.push_back(walkinggait.Control_Step_length_X_);
	


	//----------- pitch ---------------------
    ideal_imu_value[(int)imu::pitch].vel = getIdealV(pres_imu_value[(int)imu::pitch].pos, ideal_p_arry_pitch, ideal_v_arry);
	pres_imu_value[(int)imu::pitch].vel = (pres_imu_value[(int)imu::pitch].pos-prev_imu_value[(int)imu::pitch].pos)/(0.03);
	passfilter_pres_imu_value[(int)imu::pitch].pos = butterfilter_imu[(int)imu::pitch].pos.getValue(pres_imu_value[(int)imu::pitch].pos);
	passfilter_pres_imu_value[(int)imu::pitch].vel = butterfilter_imu[(int)imu::pitch].vel.getValue(pres_imu_value[(int)imu::pitch].vel);
	passfilter_prev_imu_value[(int)imu::pitch] = passfilter_pres_imu_value[(int)imu::pitch];
	
	//----------- roll ----------------------
	ideal_imu_value[(int)imu::roll].vel = getIdealV(pres_imu_value[(int)imu::roll].pos, ideal_p_arry_roll, ideal_v_arry);
	pres_imu_value[(int)imu::roll].vel = (pres_imu_value[(int)imu::roll].pos-prev_imu_value[(int)imu::roll].pos)/(0.03);
	passfilter_pres_imu_value[(int)imu::roll].pos = butterfilter_imu[(int)imu::roll].pos.getValue(pres_imu_value[(int)imu::roll].pos);
	passfilter_pres_imu_value[(int)imu::roll].vel = butterfilter_imu[(int)imu::roll].vel.getValue(pres_imu_value[(int)imu::roll].vel);
	passfilter_prev_imu_value[(int)imu::roll] = passfilter_pres_imu_value[(int)imu::roll];

	PIDleftfoot_hip_pitch.setControlGoal(ideal_imu_value[(int)imu::pitch].vel);
	PIDleftfoot_hip_roll.setControlGoal(ideal_imu_value[(int)imu::roll].vel); 
	PIDrightfoot_hip_pitch.setControlGoal(ideal_imu_value[(int)imu::pitch].vel);
	PIDrightfoot_hip_roll.setControlGoal(ideal_imu_value[(int)imu::roll].vel);
	
	if(walkinggait.LIPM_flag_)
	{
		CoM_EPx_value.control_value_once = PIDCoM_x.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].pos);
		CoM_EPx_value.control_value_total -= CoM_EPx_value.control_value_once;
		if(CoM_EPx_value.control_value_total>3){CoM_EPx_value.control_value_total = 3;}else if(CoM_EPx_value.control_value_total<-3){CoM_EPx_value.control_value_total = -3;}
		
		PIDCoM_y.setControlGoal(walkinggait.py_u);
		CoM_EPy_value.control_value_once = PIDCoM_y.calculateExpValue(passfilter_pres_imu_value[(int)imu::roll].pos);
		CoM_EPy_value.control_value_total -= CoM_EPy_value.control_value_once;
		if(CoM_EPy_value.control_value_total>1){CoM_EPy_value.control_value_total = 1;}else if(CoM_EPy_value.control_value_total<-1){CoM_EPy_value.control_value_total = -1;}

		parameterinfo->points.IK_Point_LX  = CoM_EPx_value.control_value_total;
		parameterinfo->points.IK_Point_RX  = CoM_EPx_value.control_value_total;
		walkinggait.Control_Step_length_X_ = 0.5*CoM_EPx_value.control_value_total;
		walkinggait.Control_Step_length_Y_ = 0.3*CoM_EPy_value.control_value_total;

	}
	
	if(parameterinfo->complan.walking_stop)
	{
		InitEndPointControl();
	}
	else if(sup_foot_ == leftfoot)
	{
        if(flag_r)
        {
            //swing
            rightfoot_hip_pitch_value.initialize();
            rightfoot_hip_roll_value.initialize();
            rightfoot_ankle_pitch_value.initialize();
            rightfoot_ankle_roll_value.initialize();
            rightfoot_hip_pitch = 0;//-= rightfoot_hip_pitch_value.control_value_total/180.0*PI;
            rightfoot_hip_roll = 0;//+= rightfoot_hip_roll_value.control_value_total/180.0*PI;
            rightfoot_ankle_pitch = 0;
            rightfoot_ankle_roll = 0;
            flag_l = true;
            flag_r = false;
        }
		// pres_ZMP.feet_pos.x pres_ZMP.feet_pos.y
		//sup
		PIDleftfoot_ankle_roll.setControlGoal(2); 
		//----------- pitch ---------------------
		leftfoot_hip_pitch_value.control_value_once = PIDleftfoot_hip_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		leftfoot_hip_pitch_value.control_value_total -= leftfoot_hip_pitch_value.control_value_once;
		leftfoot_hip_pitch = leftfoot_hip_pitch_value.control_value_total/180.0*PI;

        // leftfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		// leftfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(kalman_rpy_[1]);//dt = 0.03
		leftfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(foot_cog_x_);//dt = 0.03
		leftfoot_ankle_pitch_value.control_value_total -= leftfoot_ankle_pitch_value.control_value_once;
		// leftfoot_ankle_pitch_value.control_value_total = asin(leftfoot_ankle_pitch_value.control_value_total/COM_HEIGHT);		
		leftfoot_ankle_pitch = leftfoot_ankle_pitch_value.control_value_total/180.0*PI;
		//----------- roll ----------------------
		leftfoot_hip_roll_value.control_value_once = PIDleftfoot_hip_roll.calculateExpValue(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03;//dt = 0.03;
		leftfoot_hip_roll_value.control_value_total += leftfoot_hip_roll_value.control_value_once;
		// leftfoot_hip_roll += leftfoot_hip_roll_value.control_value_total/180.0*PI;
		leftfoot_hip_roll = leftfoot_hip_roll_value.control_value_total/180.0*PI;

        // leftfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03;//dt = 0.03;
		// leftfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue(kalman_rpy_[0]);//dt = 0.03;
		leftfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue(foot_cog_y_);//dt = 0.03;
		leftfoot_ankle_roll_value.control_value_total -= leftfoot_ankle_roll_value.control_value_once;
		// leftfoot_ankle_roll_value.control_value_total = asin(leftfoot_ankle_roll_value.control_value_total/COM_HEIGHT);				
		leftfoot_ankle_roll = leftfoot_ankle_roll_value.control_value_total/180.0*PI;

		
		// parameterinfo->points.IK_Point_LX -= 0.5 * CoM_EPx_value.control_value_once;//CoM點控制 左腳
        if(abs(kalman_rpy_[0])> 2.5){roll_count_++;}else{roll_count_--;}
        if(abs(kalman_rpy_[1])> 2.5){pitch_count_++;}else{pitch_count_--;}
        if(roll_count_ < 0){
            leftfoot_hip_roll_value.initialize();
            leftfoot_ankle_roll_value.initialize();
            roll_count_ = 0;}
        if(pitch_count_< 0){
            leftfoot_hip_pitch_value.initialize();
            leftfoot_ankle_pitch_value.initialize();
            pitch_count_ = 0;}
    }
	else if(sup_foot_ == rightfoot)
	{
        if(flag_l)
        {
            //swing
            leftfoot_hip_pitch_value.initialize();
            leftfoot_hip_roll_value.initialize();
            leftfoot_ankle_pitch_value.initialize();
            leftfoot_ankle_roll_value.initialize();
            leftfoot_hip_pitch = 0;//-= rightfoot_hip_pitch_value.control_value_total/180.0*PI;
            leftfoot_hip_roll = 0;//+= rightfoot_hip_roll_value.control_value_total/180.0*PI;
            leftfoot_ankle_pitch = 0;
            leftfoot_ankle_roll = 0;
            flag_l = false;
            flag_r = true;
        }
		//sup
		PIDrightfoot_ankle_roll.setControlGoal(-2); 
		//----------- pitch ---------------------
		rightfoot_hip_pitch_value.control_value_once = PIDleftfoot_hip_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03 ;//dt = 0.03
		rightfoot_hip_pitch_value.control_value_total -= rightfoot_hip_pitch_value.control_value_once;
		rightfoot_hip_pitch = rightfoot_hip_pitch_value.control_value_total/180.0*PI;
		
        // rightfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03 ;
		// rightfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(kalman_rpy_[1]) ;//dt = 0.03
		rightfoot_ankle_pitch_value.control_value_once = PIDleftfoot_ankle_pitch.calculateExpValue(foot_cog_x_);
		rightfoot_ankle_pitch_value.control_value_total -= rightfoot_ankle_pitch_value.control_value_once;
		// rightfoot_ankle_pitch_value.control_value_total = asin(rightfoot_ankle_pitch_value.control_value_total/COM_HEIGHT);
		rightfoot_ankle_pitch = rightfoot_ankle_pitch_value.control_value_total/180.0*PI;
		//----------- roll ----------------------
		rightfoot_hip_roll_value.control_value_once = PIDleftfoot_hip_roll.calculateExpValue(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03 ;//dt = 0.03;
		rightfoot_hip_roll_value.control_value_total += rightfoot_hip_roll_value.control_value_once;
		// rightfoot_hip_roll += rightfoot_hip_roll_value.control_value_total/180.0*PI;
		rightfoot_hip_roll = rightfoot_hip_roll_value.control_value_total/180.0*PI;

        // rightfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03 ;////dt = 0.03;
		// rightfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue(kalman_rpy_[0]) ;//dt = 0.03;
		rightfoot_ankle_roll_value.control_value_once = PIDleftfoot_ankle_roll.calculateExpValue(foot_cog_y_);//dt = 0.03;
		rightfoot_ankle_roll_value.control_value_total -= rightfoot_ankle_roll_value.control_value_once;
		// rightfoot_ankle_roll_value.control_value_total = asin(rightfoot_ankle_roll_value.control_value_total/COM_HEIGHT);
		rightfoot_ankle_roll = rightfoot_ankle_roll_value.control_value_total/180.0*PI;

		if(abs(kalman_rpy_[0])> 2.5){roll_count_++;}else{roll_count_--;}
        if(abs(kalman_rpy_[1])> 2.5){pitch_count_++;}else{pitch_count_--;}
        if(roll_count_ < 0){
            rightfoot_hip_roll_value.initialize();
            rightfoot_ankle_roll_value.initialize();
            roll_count_ = 0;}
        if(pitch_count_< 0){
            rightfoot_hip_pitch_value.initialize();
            rightfoot_ankle_pitch_value.initialize();
            pitch_count_ = 0;}
		// parameterinfo->points.IK_Point_RX -= 0.5 * CoM_EPx_value.control_value_once;//CoM點控制 右腳
	}
	else if(sup_foot_ == doublefeet)
	{
		if(flag_r||flag_l)
		{ 
			leftfoot_hip_pitch_value.initialize();
			rightfoot_hip_pitch_value.initialize();
            leftfoot_ankle_pitch_value.initialize();
            leftfoot_ankle_roll_value.initialize();
			leftfoot_hip_pitch = 0;//-= rightfoot_hip_pitch_value.control_value_total/180.0*PI;
			leftfoot_hip_roll = 0;//+= rightfoot_hip_roll_value.control_value_total/180.0*PI;
			rightfoot_hip_pitch = 0;//-= rightfoot_hip_pitch_value.control_value_total/180.0*PI;
			rightfoot_hip_roll = 0;//+= rightfoot_hip_roll_value.control_value_total/180.0*PI;
			flag_r = false;
            flag_l = false;
		}
		leftfoot_hip_pitch = 0;
		rightfoot_hip_pitch = 0;
		//----------- pitch ---------------------
		leftfoot_hip_pitch_value.control_value_once = fuzzy.fuzzy_pitch_control(passfilter_pres_imu_value[(int)imu::pitch].pos, passfilter_pres_imu_value[(int)imu::pitch].vel);
		// leftfoot_hip_pitch_value.control_value_once = PIDleftfoot_stand_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		leftfoot_hip_pitch_value.control_value_total += leftfoot_hip_pitch_value.control_value_once;
		leftfoot_hip_pitch += leftfoot_hip_pitch_value.control_value_total/180.0*PI;

		rightfoot_hip_pitch_value.control_value_once = fuzzy.fuzzy_pitch_control(passfilter_pres_imu_value[(int)imu::pitch].pos, passfilter_pres_imu_value[(int)imu::pitch].vel);
		// rightfoot_hip_pitch_value.control_value_once = PIDrightfoot_stand_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		rightfoot_hip_pitch_value.control_value_total += rightfoot_hip_pitch_value.control_value_once;
		rightfoot_hip_pitch += rightfoot_hip_pitch_value.control_value_total/180.0*PI;
		
        // leftfoot_ankle_pitch_value.control_value_once = fuzzy.fuzzy_pitch_control(passfilter_pres_imu_value[(int)imu::pitch].pos, passfilter_pres_imu_value[(int)imu::pitch].vel);
		// // leftfoot_ankle_pitch_value.control_value_once = PIDleftfoot_stand_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		// leftfoot_ankle_pitch_value.control_value_total -= leftfoot_ankle_pitch_value.control_value_once;
		// leftfoot_ankle_pitch = leftfoot_ankle_pitch_value.control_value_total/180.0*PI;

		// rightfoot_ankle_pitch_value.control_value_once = fuzzy.fuzzy_pitch_control(passfilter_pres_imu_value[(int)imu::pitch].pos, passfilter_pres_imu_value[(int)imu::pitch].vel);
		// // rightfoot_ankle_pitch_value.control_value_once = PIDrightfoot_stand_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		// rightfoot_ankle_pitch_value.control_value_total -= rightfoot_ankle_pitch_value.control_value_once;
		// rightfoot_ankle_pitch = rightfoot_ankle_pitch_value.control_value_total/180.0*PI;

		// if(parameterinfo->points.IK_Point_LZ>16.3)
		// {
		// 	parameterinfo->points.IK_Point_LZ -= abs(rightfoot_hip_pitch_value.control_value_once)/3; 
		// 	parameterinfo->points.IK_Point_RZ -= abs(rightfoot_hip_pitch_value.control_value_once)/3;
		// }
        
	}
	map_roll.find("left_control_once_roll")->second.push_back(leftfoot_hip_roll_value.control_value_once);
	map_roll.find("left_control_total_roll")->second.push_back(leftfoot_hip_roll_value.control_value_total);
	map_roll.find("right_control_once_roll")->second.push_back(rightfoot_hip_roll_value.control_value_once);
	map_roll.find("right_control_total_roll")->second.push_back(rightfoot_hip_roll_value.control_value_total);
	map_roll.find("smaple_times_count")->second.push_back(foot_cog_y_);
    map_roll.find("pres_roll_pos")->second.push_back(pres_imu_value[(int)imu::roll].pos);
    map_roll.find("passfilter_pres_roll_pos")->second.push_back(passfilter_pres_imu_value[(int)imu::roll].pos);
    map_roll.find("ideal_roll_vel")->second.push_back(ideal_imu_value[(int)imu::roll].vel);
    map_roll.find("pres_roll_vel")->second.push_back(pres_imu_value[(int)imu::roll].vel);
    map_roll.find("passfilter_pres_roll_vel")->second.push_back(passfilter_pres_imu_value[(int)imu::roll].vel);
	map_roll.find("leftfoot_hip_roll")->second.push_back(leftfoot_hip_roll);
	map_roll.find("rightfoot_hip_roll")->second.push_back(rightfoot_hip_roll);
	map_roll.find("support_foot")->second.push_back(sensor.rpy_[1]);

	map_pitch.find("left_control_once_pitch")->second.push_back(leftfoot_hip_pitch_value.control_value_once);
	map_pitch.find("left_control_total_pitch")->second.push_back(leftfoot_hip_pitch_value.control_value_total);
	map_pitch.find("right_control_once_pitch")->second.push_back(rightfoot_hip_pitch_value.control_value_once);
	map_pitch.find("right_control_total_pitch")->second.push_back(rightfoot_hip_pitch_value.control_value_total);

    map_pitch.find("left_control_once_pitch_ankle")->second.push_back(leftfoot_ankle_pitch_value.control_value_once);
	map_pitch.find("left_control_total_pitch_ankle")->second.push_back(leftfoot_ankle_pitch);
	map_pitch.find("right_control_once_pitch_ankle")->second.push_back(rightfoot_ankle_pitch_value.control_value_once);
	map_pitch.find("right_control_total_pitch_ankle")->second.push_back(rightfoot_ankle_pitch);

	map_pitch.find("smaple_times_count")->second.push_back(foot_cog_x_);
    map_pitch.find("pres_pitch_pos")->second.push_back(pres_imu_value[(int)imu::pitch].pos);
    map_pitch.find("passfilter_pres_pitch_pos")->second.push_back(passfilter_pres_imu_value[(int)imu::pitch].pos);
    map_pitch.find("ideal_pitch_vel")->second.push_back(ideal_imu_value[(int)imu::pitch].vel);
    map_pitch.find("pres_pitch_vel")->second.push_back(pres_imu_value[(int)imu::pitch].vel);
    map_pitch.find("passfilter_pres_pitch_vel")->second.push_back(passfilter_pres_imu_value[(int)imu::pitch].vel);

	map_CoM.find("CoM_x_control")->second.push_back(CoM_EPx_value.control_value_once);
	map_CoM.find("new_EP_lx")->second.push_back(parameterinfo->points.IK_Point_LZ);
	map_CoM.find("new_EP_rx")->second.push_back(parameterinfo->points.IK_Point_RX);
}

void BalanceControl::InitEndPointControl()
{
	for(int i = 0; i < sizeof(butterfilter_imu)/sizeof(butterfilter_imu[0]); i++)
        butterfilter_imu[i].initialize();
	// pitch_value.initialize();
	// pitch_value_a.initialize();
	leftfoot_hip_roll_value.initialize();
	rightfoot_hip_roll_value.initialize();
	leftfoot_hip_pitch_value.initialize();
	rightfoot_hip_pitch_value.initialize();
	leftfoot_ankle_roll_value.initialize();
	rightfoot_ankle_roll_value.initialize();
	leftfoot_ankle_pitch_value.initialize();
	rightfoot_ankle_pitch_value.initialize();
	CoM_EPx_value.initialize();
	PIDCoM_x.initParam();
	PIDCoM_z.initParam();
	parameterinfo->points.IK_Point_LX = 0;
	parameterinfo->points.IK_Point_RX = 0;
	// parameterinfo->points.IK_Point_LZ = Length_Leg;
	// parameterinfo->points.IK_Point_RZ = Length_Leg;
	leftfoot_hip_roll = 0;
	rightfoot_hip_roll = 0;
	leftfoot_hip_pitch = 0;
	rightfoot_hip_pitch = 0;
	leftfoot_ankle_roll = 0;
	rightfoot_ankle_roll = 0;
	leftfoot_ankle_pitch = 0;
	rightfoot_ankle_pitch = 0;

    //kalman
    q_angle_ = 0.0001;
    q_bias_ = 0.0001;
    r_measure_ = 0.05;//0.1;//0.03;    //0.0005
	
    angle_[0] = 0.0;
	angle_[1] = 0.0;
	angle_[2] = 0.0;
	bias_[0] = 0.0;
	bias_[1] = 0.0;
	bias_[2] = 0.0;
	
	p_[0][0][0] = 0;
	p_[0][0][1] = 0;
	p_[0][1][0] = 0;
	p_[0][1][1] = 0;

	p_[1][0][0] = 0;
	p_[1][0][1] = 0;
	p_[1][1][0] = 0;
	p_[1][1][1] = 0;

	p_[2][0][0] = 0;
	p_[2][0][1] = 0;
	p_[2][1][0] = 0;
	p_[2][1][1] = 0;
	
}

void BalanceControl::control_after_ik_calculation()
{
	// compensate
	if(walkinggait.if_finish_)
	{
		Points.Thta[10] = Points.Thta[10];
		Points.Thta[11] = Points.Thta[11];
		Points.Thta[12] = Points.Thta[12];
		Points.Thta[13] = Points.Thta[13];
		Points.Thta[14] = Points.Thta[14];

		Points.Thta[16] = Points.Thta[16];
		Points.Thta[17] = Points.Thta[17];
		Points.Thta[18] = Points.Thta[18];
		Points.Thta[19] = Points.Thta[19];
		Points.Thta[20] = Points.Thta[20];
	}
    else
    {
        if(sup_foot_ == leftfoot)
        {
            Points.Thta[10] -= foot_offset_[0]*DEGREE2RADIAN;//0.5;
            Points.Thta[16] += foot_offset_[1]*DEGREE2RADIAN;//1.8;
            
            Points.Thta[10] += leftfoot_hip_roll;
            Points.Thta[11] += leftfoot_hip_pitch;
            Points.Thta[13] += leftfoot_ankle_pitch;
            Points.Thta[14] += leftfoot_ankle_roll;

            Points.Thta[16] += rightfoot_hip_roll;
            Points.Thta[17] += rightfoot_hip_pitch;
            Points.Thta[19] += rightfoot_ankle_pitch;
            Points.Thta[20] += rightfoot_ankle_roll;
        }
        else if(sup_foot_ == rightfoot)
        {
            Points.Thta[10] += foot_offset_[1]*DEGREE2RADIAN;//1.8;
            Points.Thta[16] += foot_offset_[0]*DEGREE2RADIAN;//0.5;

            Points.Thta[10] += leftfoot_hip_roll;
            Points.Thta[11] += leftfoot_hip_pitch;
            Points.Thta[13] += leftfoot_ankle_pitch;
            Points.Thta[14] += leftfoot_ankle_roll;

            Points.Thta[16] += rightfoot_hip_roll;
            Points.Thta[17] += rightfoot_hip_pitch;
            Points.Thta[19] += rightfoot_ankle_pitch;
            Points.Thta[20] += rightfoot_ankle_roll;
        }
        else if(sup_foot_ == doublefeet)
        { 
            Points.Thta[10] += leftfoot_hip_roll;
            Points.Thta[11] += leftfoot_hip_pitch;
            Points.Thta[13] += leftfoot_ankle_pitch;
            Points.Thta[14] += leftfoot_ankle_roll;

            Points.Thta[16] += rightfoot_hip_roll;
            Points.Thta[17] += rightfoot_hip_pitch;
            Points.Thta[19] += rightfoot_ankle_pitch;
            Points.Thta[20] += rightfoot_ankle_roll;
        }
    }
	
}

double BalanceControl::get_angle(double acc_angle_tmp, double gyro_angle_tmp, double dt, int i)
{
    double s, y;
    angle_[i] += (gyro_angle_tmp - bias_[i]) * dt;

    p_[i][0][0] += dt * (dt * p_[i][1][1] - p_[i][0][1] - p_[i][1][0] + q_angle_);
    p_[i][0][1] -= dt * p_[i][1][1];
    p_[i][1][0] -= dt * p_[i][1][1];
    p_[i][1][1] += q_bias_ * dt;

    s = p_[i][0][0] + r_measure_;

    k_[0] = p_[i][0][0] / s;  //k = K Gain;
    k_[1] = p_[i][1][0] / s;

    y = acc_angle_tmp - angle_[i];

    angle_[i] += k_[0] * y;
    bias_[i] += k_[1] * y;

    p_[i][0][0] -= k_[0] * p_[i][0][0];
    p_[i][0][1] -= k_[0] * p_[i][0][1];
    p_[i][1][0] -= k_[1] * p_[i][0][0];
    p_[i][1][1] -= k_[1] * p_[i][0][1];

    return angle_[i];
}

double BalanceControl::get_force(double Force_data, int i){
    force_x_[i] = force_x_[i] + force_w_;
    force_p_[i] = force_p_[i] + force_q_;
    
    force_k_[i] = force_p_[i] / (force_p_[i] + force_r_[i]);
    
    force_x_[i] = force_x_[i] + force_k_[i]*(Force_data - force_x_[i]);
    force_p_[i] = (1 - force_k_[i])*force_p_[i];
    
    return force_x_[i];
}

double BalanceControl::get_q_bias()
{
    return q_bias_;
}

PID_Controller::PID_Controller()
{
    this->Kp = 0;
    this->Ki = 0;
    this->Kd = 0;
    this->error = 0;
    this->pre_error = 0;
    this->errors = 0;
    this->errord = 0;
    this->x1c = 0;
    this->x2c = 0;
    this->x3c = 0;
    this->exp_value = 0;
    this->value = 0;
    this->pre_value = 0;
}

PID_Controller::~PID_Controller()
{
    
}

void PID_Controller::initParam()
{
    this->pre_error = 0;
    this->error = 0;
    this->errors = 0;
    this->errord = 0;
    this->exp_value = 0;
    this->value = 0;
    this->pre_value = 0;
}

void PID_Controller::setKpid(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID_Controller::setControlGoal(float x1c, float x2c, float x3c)
{
    this->x1c = x1c;
    this->x2c = x2c;
    this->x3c = x3c;
}

float PID_Controller::calculateExpValue(float value)//Expected value
{
    this->pre_value = this->value;
    this->value = value;
    this->pre_error = this->error;
    this->error = this->x1c - this->value;
    this->errors += this->error*0.1;
    if(this->pre_error == 0)
    {
        this->errord = 0;
    } 
    else
    {
        this->errord = (this->error - this->pre_error)/0.03;
    }
    this->exp_value = this->Kp*this->error + this->Ki*this->errors + this->Kd*this->errord;
    if(this->exp_value > this->upper_limit)
    {
        return this->upper_limit;
    }
    else if(this->exp_value < this->lower_limit)
    {
        return this->lower_limit;
    }
    else
    {
        return this->exp_value;
    }
}

void PID_Controller::setValueLimit(float upper_limit, float lower_limit)
{
    this->upper_limit = upper_limit;
    this->lower_limit = lower_limit;
}

float PID_Controller::getError()
{
    return this->error;
}

float PID_Controller::getErrors()
{
    return this->errors;
}

float PID_Controller::getErrord()
{
    return this->errord;
}

ButterWorthParam ButterWorthParam::set(float a1, float a2, float b1, float b2)
{
    ButterWorthParam temp;
    temp.a_[0] = a1;
    temp.a_[1] = a2;
    temp.b_[0] = b1;
    temp.b_[1] = b2;
    return temp;
}

ButterWorthFilter::ButterWorthFilter()
{

}

ButterWorthFilter::~ButterWorthFilter()
{

}

void ButterWorthFilter::initialize(ButterWorthParam param)
{
    param_ = param;
    prev_output_ = 0;
    prev_value_ = 0;
}

float ButterWorthFilter::getValue(float present_value)
{
    if(prev_output_ == 0 && prev_value_ == 0)
    {
        prev_output_ = param_.b_[0]*present_value/param_.a_[0];
        prev_value_ = present_value;
        return prev_output_;
    }
    else
    {
        prev_output_ = (param_.b_[0]*present_value + param_.b_[1]*prev_value_ - param_.a_[1]*prev_output_)/param_.a_[0];
        prev_value_ = present_value;
        return prev_output_;
    }
}

BalanceLowPassFilter::BalanceLowPassFilter()
{
	cut_off_freq_ = 1.0;
	control_cycle_sec_ = 0.008;
	prev_output_ = 0;

	alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
}

BalanceLowPassFilter::~BalanceLowPassFilter()
{

}

void BalanceLowPassFilter::initialize(double control_cycle_sec, double cut_off_frequency)
{
	cut_off_freq_ = cut_off_frequency;
	control_cycle_sec_ = control_cycle_sec;
	prev_output_ = 0;

	if(cut_off_frequency > 0)
		alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
	else
		alpha_ = 1;
}

void BalanceLowPassFilter::set_cut_off_frequency(double cut_off_frequency)
{
	cut_off_freq_ = cut_off_frequency;

	if(cut_off_frequency > 0)
	alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
	else
	alpha_ = 1;
}

double BalanceLowPassFilter::get_cut_off_frequency(void)
{
	return cut_off_freq_;
}

double BalanceLowPassFilter::get_filtered_output(double present_raw_value)
{
	prev_output_ = alpha_*present_raw_value + (1.0 - alpha_)*prev_output_;
	return prev_output_;
}

string BalanceControl::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}
 
void BalanceControl::saveData()
{
	//------roll------
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_Roll"+tmp+".csv";
    strcat(path, tmp.c_str());

    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<float>>::iterator it_roll;

	for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
	{
		savedText += it_roll->first;
		if(it_roll == --map_roll.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_roll = map_roll.begin();
	int max_size = it_roll->second.size();

	for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
	{
		if(max_size < it_roll->second.size())
            max_size = it_roll->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
        {
            if(i < it_roll->second.size())
            {
                if(it_roll == --map_roll.end())
                {
                    savedText += std::to_string(it_roll->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_roll->second[i]) + ",";
                }
            }
            else
            {
                if(it_roll == --map_roll.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
        it_roll->second.clear();

	//------pitch------
	char path2[200] = "/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_Pitch_"+tmp+".csv";
    strcat(path2, tmp.c_str());
    fp.open(path2, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_pitch;

	for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
	{
		savedText += it_pitch->first;
		if(it_pitch == --map_pitch.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_pitch = map_pitch.begin();
	max_size = it_pitch->second.size();

	for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
	{
		if(max_size < it_pitch->second.size())
            max_size = it_pitch->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
        {
            if(i < it_pitch->second.size())
            {
                if(it_pitch == --map_pitch.end())
                {
                    savedText += std::to_string(it_pitch->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_pitch->second[i]) + ",";
                }
            }
            else
            {
                if(it_pitch == --map_pitch.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
        it_pitch->second.clear();

	//------ZMP------
	char path3[200] = "/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_ZMP_"+tmp+".csv";
    strcat(path3, tmp.c_str());
    fp.open(path3, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_ZMP;

	for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
	{
		savedText += it_ZMP->first;
		if(it_ZMP == --map_ZMP.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_ZMP = map_ZMP.begin();
	max_size = it_ZMP->second.size();

	for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
	{
		if(max_size < it_ZMP->second.size())
            max_size = it_ZMP->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
        {
            if(i < it_ZMP->second.size())
            {
                if(it_ZMP == --map_ZMP.end())
                {
                    savedText += std::to_string(it_ZMP->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_ZMP->second[i]) + ",";
                }
            }
            else
            {
                if(it_ZMP == --map_ZMP.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
        it_ZMP->second.clear();

	//------CoM------
	char path4[200] = "/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_CoM_"+tmp+".csv";
    strcat(path4, tmp.c_str());
    fp.open(path4, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_CoM;

	for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
	{
		savedText += it_CoM->first;
		if(it_CoM == --map_CoM.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_CoM = map_CoM.begin();
	max_size = it_CoM->second.size();

	for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
	{
		if(max_size < it_CoM->second.size())
            max_size = it_CoM->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
        {
            if(i < it_CoM->second.size())
            {
                if(it_CoM == --map_CoM.end())
                {
                    savedText += std::to_string(it_CoM->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_CoM->second[i]) + ",";
                }
            }
            else
            {
                if(it_CoM == --map_CoM.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
        it_CoM->second.clear();

//------Accel------
	char path5[200] = "/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Sensor_data_accel"+tmp+".csv";
    strcat(path5, tmp.c_str());
    fp.open(path5, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_Accel;

	for(it_Accel = map_Accel.begin(); it_Accel != map_Accel.end(); it_Accel++)
	{
		savedText += it_Accel->first;
		if(it_Accel == --map_Accel.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_Accel = map_Accel.begin();
	max_size = it_Accel->second.size();

	for(it_Accel = map_Accel.begin(); it_Accel != map_Accel.end(); it_Accel++)
	{
		if(max_size < it_Accel->second.size())
            max_size = it_Accel->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_Accel = map_Accel.begin(); it_Accel != map_Accel.end(); it_Accel++)
        {
            if(i < it_Accel->second.size())
            {
                if(it_Accel == --map_Accel.end())
                {
                    savedText += std::to_string(it_Accel->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_Accel->second[i]) + ",";
                }
            }
            else
            {
                if(it_Accel == --map_Accel.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_Accel = map_Accel.begin(); it_Accel != map_Accel.end(); it_Accel++)
        it_Accel->second.clear();

//-----end
	name_cont_++;
}
#include "include/Walkinggait.h"
#include "ZMP.h"
#include "ZMPProcess.h"

WalkingCycle walkingcycle;
WalkingTrajectory walkingtrajectory;
kickgait_space::KickingGait kickinggait;
Force_Diff_Ctrl_base FDC;

extern BalanceControl balance;
extern InverseKinematic IK;
extern Initial init;
extern SensorDataProcess sensor;
extern Feedback_Motor feedbackmotor;


Walkinggait::Walkinggait()
{
    update_parameter_flag_ = false;
    update_walkdata_flag_ = false;
    continuous_stop_flag_ = false;
    get_parameter_flag_ = false; 
    get_walkdata_flag_ = false;
    locus_flag_ = false;
    push_data_ = false;
    delay_push_ = false;
}

Walkinggait::~Walkinggait()
{
 
}

void Walkinggait::walking_timer()
{
    //pushData();
    if(!parameterinfo->complan.walking_stop)
    {
        switch(parameterinfo->walking_mode)
		{
        case Single:
            single();
            locus_flag_ = true;
            LIPM_flag_ = true;
        	break;
        case Continuous:
            process();
            locus_flag_ = true;
            LIPM_flag_ = true;
        	break;
        case LC_up:            
            LC();
            locus_flag_ = true;
            LIPM_flag_ = true;
        	break;
        case LC_down:
            //walkingcycle.walkingkindfunction(parameterinfo->walking_mode);
            //walkingtrajectory.walkingprocess(parameterinfo->walking_mode);
            //parameterinfo->CPGalready = true;
            LC_dsp();
            locus_flag_ = true;
            LIPM_flag_ = true;
        	break;
        case Long_Jump:
            locus_flag_ = true;
            LIPM_flag_ = true;
        	break;
        case RKickB:
        case LKickB:
            kickinggait.kickingCycle(parameterinfo->walking_mode);
            parameterinfo->CPGalready = true;
            locus_flag_ = true;          
        	break;
        default:
            break;
		}
    }
    gettimeofday(&timer_start_, NULL);
}

void Walkinggait::load_parameter()
{
    int state = 0;
	int count = 0;

	for(;;)
	{
		if(state == 0)
		{
			update_parameter_flag_ = false;
			if(*(uint32_t *)init.p2h_set_hps_read_parameter_addr)
			{
				state = 1;
				continue;
			}
			else
			{
				break;
			}
		}
		else if(state == 1)
		{
			if(count <= 5)
			{
				parameter_[count] = *(uint32_t *)init.p2h_parameter_addr;
				count++;
				*(uint32_t *)init.h2p_read_parameter_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_parameter_pulse_addr = 0;
				continue;
			}
			else
			{
				update_parameter_flag_ = true;
				state = 0;
				break;
			}
		}
	}
    update_parameter();
}

void Walkinggait::update_parameter()
{

    if(update_parameter_flag_)
    {
        int parameter_cnt;
        int arr_index = 0;
        short tmp = 0;
        double tmp_arr[12] = {0.0};

        for(parameter_cnt=0; parameter_cnt<6; parameter_cnt++)
        {
            tmp = ((parameter_[parameter_cnt] & 0xFFFF0000) >> 16);
            if(tmp & 0x8000)
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF) * (-1)) / 100;
            else
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF)) / 100;

            tmp = ((parameter_[parameter_cnt] & 0x0000FFFF));
            if(tmp & 0x8000)
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF) * (-1)) / 100;
            else
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF)) / 100;
        }
        parameter_cnt = 5;
        parameterinfo->walking_mode = (parameter_[parameter_cnt] & 0xFF000000) >> 24;
        if(parameterinfo->walking_mode != 9 && parameterinfo->walking_mode != 10)
        {
            arr_index = 0;
            parameter_cnt = 1;
            parameterinfo->parameters.X_Swing_Range = tmp_arr[arr_index++];
            parameterinfo->parameters.Y_Swing_Range = tmp_arr[arr_index++];
            parameterinfo->parameters.Z_Swing_Range = tmp_arr[arr_index++];
            //COM_HEIGHT = parameterinfo->parameters.Z_Swing_Range;
            parameterinfo->parameters.Period_T = parameter_[parameter_cnt++] & 0x0000FFFF;
            parameterinfo->parameters.Period_T2 = (parameter_[parameter_cnt] & 0xFFFF0000) >> 16;
            parameterinfo->parameters.Sample_Time = (parameter_[parameter_cnt] & 0x0000FF00) >> 8;
            parameterinfo->parameters.OSC_LockRange = ((double)(parameter_[parameter_cnt++] & 0x000000FF)) / 100;

            arr_index = 6;
            parameter_cnt = 5;
            parameterinfo->parameters.BASE_Default_Z = tmp_arr[arr_index++];
            parameterinfo->parameters.X_Swing_COM = tmp_arr[arr_index++];
            parameterinfo->parameters.Y_Swing_Shift = tmp_arr[arr_index++];
            parameterinfo->parameters.BASE_LIFT_Z = tmp_arr[arr_index++];
            arr_index++;
            parameterinfo->LCBalanceOn = tmp_arr[arr_index++];
            parameterinfo->parameters.Sample_Time = parameterinfo->parameters.Period_T/15;
            if(parameterinfo->parameters.Sample_Time == 0)
            {
                motion_delay_ = 15;
            }
            else
            {
                motion_delay_ = parameterinfo->parameters.Period_T / parameterinfo->parameters.Sample_Time;
            }     
        }
        else
        {
            arr_index = 0;
            parameterinfo->parameters.Y_Swing_Range = tmp_arr[arr_index++];
            parameterinfo->parameters.Period_T      = (parameter_[0] & 0x0000FFFF) + 600;
            arr_index = 2;
            parameterinfo->parameters.Kick_Point_X  = tmp_arr[arr_index++];
            parameterinfo->parameters.Kick_Point_Y  = tmp_arr[arr_index++];
            parameterinfo->parameters.Kick_Point_Z  = tmp_arr[arr_index++];
            parameterinfo->parameters.Back_Point_X  = tmp_arr[arr_index++];
            parameterinfo->parameters.Back_Point_Z  = tmp_arr[arr_index++];
            parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch  = tmp_arr[arr_index++];
            parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch  = tmp_arr[arr_index++];
            parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch  = tmp_arr[arr_index++];
            parameterinfo->parameters.Sample_Time = parameterinfo->parameters.Period_T/30;
            if(parameterinfo->parameters.Sample_Time == 0)
            {
                motion_delay_ = 30;
            }
            else
            {
                motion_delay_ = parameterinfo->parameters.Period_T / parameterinfo->parameters.Sample_Time;
            }      
        }
        get_parameter_flag_ = true;
    }

}

void Walkinggait::load_walkdata()
{
    int state = 0;
	int count = 0;

	for(;;)
	{
		if(state == 0)
		{
			update_walkdata_flag_ = false;
			if(*(uint32_t *)init.p2h_set_hps_read_walkdata_addr)
			{
				state = 1;
				continue;
			}
			else
			{
				break;
			}
		}
		else if(state == 1)
		{
			if(count <= 2)
			{
				walkdata_[count] = *(uint32_t *)init.p2h_walkdata_addr;
				count++;
				*(uint32_t *)init.h2p_read_walkdata_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_walkdata_pulse_addr = 0;
				continue;
			}
			else
			{
				update_walkdata_flag_ = true;
				state = 0;
				break;
			}
		}
	}
    update_walkdata();
}
//讀取現在的速度
void Walkinggait::update_walkdata()
{
    if(update_walkdata_flag_)
    {
        int walkdata_cnt;
        int arr_index = 0;
        short tmp = 0;
        double tmp_arr[12] = {0.0};

        for(walkdata_cnt=0; walkdata_cnt<2; walkdata_cnt++)
        {
            tmp = ((walkdata_[walkdata_cnt] & 0xFFFF0000) >> 16);
            if(tmp & 0x8000)
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF) * (-1));
            else
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF));

            tmp = ((walkdata_[walkdata_cnt] & 0x0000FFFF));
            if(tmp & 0x8000)
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF) * (-1));
            else
                tmp_arr[arr_index++] = (double)((tmp & 0x7FFF));
        }

        arr_index = 0;
        walkdata_cnt = 2;
        parameterinfo->X = tmp_arr[arr_index++] / 1000.0;
        parameterinfo->Y = tmp_arr[arr_index++] / 1000.0;
        parameterinfo->Z = tmp_arr[arr_index++] / 1000.0;
        parameterinfo->THTA = tmp_arr[arr_index] / 180.0 * PI;
        walking_cmd_ = (walkdata_[walkdata_cnt] >> 24) & 0xFF;
        sensor_mode_ = (walkdata_[walkdata_cnt] >> 16) & 0xFF;
        get_walkdata_flag_ = true;
    }
}

void Walkinggait::calculate_point_trajectory()
{
    if(get_parameter_flag_ && get_walkdata_flag_)
    {
        if(walking_cmd_ != etChangeValue)   //walking_cmd_ = generate
        {
            if(parameterinfo->complan.walking_state == StopStep)
            {
                parameterinfo->complan.walking_state = StartStep;
                parameterinfo->complan.walking_stop = false;
                pre_walking_mode = parameterinfo->walking_mode;
            }
            else if(pre_walking_mode == Continuous)
            {
                parameterinfo->complan.walking_state = StopStep;
                ready_to_stop_ = true;
                pre_walking_mode = 0;
            }
            else 
            {
                parameterinfo->complan.walking_state = StopStep;
                pre_walking_mode = parameterinfo->walking_mode;
            }
            parameterinfo->complan.sample_point_ = 0;

            // // check walking_cmd if it is start , stop or change value
            // if(parameterinfo->complan.walking_state == StopStep)
            // {
            //     parameterinfo->complan.walking_stop = false;
            //     parameterinfo->complan.walking_state = StartStep;
            //     parameterinfo->WalkFlag = true;
            //     parameterinfo->counter = 0;
            //     parameterinfo->Repeat = true;
            // }
            // else
            // {
            //     parameterinfo->WalkFlag = false;
            //     parameterinfo->complan.walking_state = StopStep;
            //     parameterinfo->Repeat = false;
            // }
        }
        get_parameter_flag_ = false;
    }
    get_walkdata_flag_ = false;
}

void Walkinggait::pushData()
{
    if(delay_push_)
    {
        cnt++;
        if(cnt > 0)
        {
            push_data_ = false;
            delay_push_ = false;
            cnt = 0;
            IK.saveData();
            balance.ZMP_process->saveData();
            // feedbackmotor.saveData();
            saveData();
            balance.saveData();
        }  
    }        
    if(push_data_)
    {
        IK.pushData();
        // map_walk.find("l_foot_x")->second.push_back(step_point_lx_);
        // map_walk.find("r_foot_x")->second.push_back(step_point_rx_);
        // map_walk.find("l_foot_y")->second.push_back(step_point_ly_);
        // map_walk.find("r_foot_y")->second.push_back(step_point_ry_);
        // map_walk.find("l_foot_z")->second.push_back(step_point_lz_);
        // map_walk.find("r_foot_z")->second.push_back(step_point_rz_);

        map_walk.find("l_foot_x")->second.push_back(end_point_lx_);
        map_walk.find("r_foot_x")->second.push_back(end_point_rx_);
        map_walk.find("l_foot_y")->second.push_back(end_point_ly_);
        map_walk.find("r_foot_y")->second.push_back(end_point_ry_);
        map_walk.find("l_foot_z")->second.push_back(end_point_lz_);
        map_walk.find("r_foot_z")->second.push_back(end_point_rz_);

        // map_walk.find("l_foot_x")->second.push_back(lpx_);
        // map_walk.find("r_foot_x")->second.push_back(rpx_);
        // map_walk.find("l_foot_y")->second.push_back(lpy_);
        // map_walk.find("r_foot_y")->second.push_back(rpy_);
        // map_walk.find("l_foot_z")->second.push_back(lpz_);
        // map_walk.find("r_foot_z")->second.push_back(rpz_);

        // map_walk.find("l_foot_t")->second.push_back(step_point_lthta_);
        // map_walk.find("r_foot_t")->second.push_back(step_point_rthta_);
        map_walk.find("com_x")->second.push_back(px_);
        map_walk.find("com_y")->second.push_back(py_);
        map_walk.find("now_step_")->second.push_back(com_y);
        map_walk.find("ideal_zmp_x")->second.push_back(zmp_x);
        map_walk.find("ideal_zmp_y")->second.push_back(zmp_y);        
        map_walk.find("A")->second.push_back(A);
        map_walk.find("B")->second.push_back(B);
        map_walk.find("aa1")->second.push_back(aa1);
        map_walk.find("aa2")->second.push_back(aa2);
        map_walk.find("d1")->second.push_back(d1);
        map_walk.find("d2")->second.push_back(d2);
        map_walk.find("time_point_")->second.push_back(time_point_);
        // map_walk.find("case")->second.push_back(Step_Count_);
        map_walk.find("sensor_roll")->second.push_back(sensor.rpy_[0]);
        map_walk.find("sensor_pitch")->second.push_back(sensor.rpy_[1]);
        map_walk.find("sensor_yaw")->second.push_back(sensor.rpy_[2]);
        map_walk.find("foot")->second.push_back(balance.sup_foot_);   
        map_walk.find("displacement_x")->second.push_back(displacement_x);
        // map_walk.find("var_theta_")->second.push_back(var_theta_); 
        // map_walk.find("Cpz")->second.push_back(balance.change_pitch);
        // map_walk.find("Cpx")->second.push_back(balance.change_roll);         
    }
}

WalkingGaitByLIPM::WalkingGaitByLIPM()
{
    is_parameter_load_ = false;

    period_t_ = 600;// T
    sample_time_ = 15;
    time_point_ = 0;
    sample_point_ = 0;
    now_step_ = 0;
    pre_step_ = -1;
    step_ = 99999;//999;
    g_ = 980;
    step_length_ = 0;//x
    last_displacement_x = 0;
    shift_length_ = 0;//y
    last_displacement_y = 0;
    theta_ = 0;//theta
    var_theta_ = 0;
    abs_theta_ = 0;
    last_abs_theta_ = 0;
    width_size_ = 4.5;//6;
    lift_height_ = 6;//default_Z
    left_step_ = 0;
    right_step_ = 0;
    footstep_x = 0;
    footstep_y = -width_size_;
    base_x = 0;
    now_left_x_ = 0;
    now_right_x_ = 0;
    base_y = 0;
    now_left_y_ = 0;
    now_right_y_ = 0;
    last_base_x = 0;
    last_base_y = 0;
    last_theta_ = 0;
    now_width_ = 0;
    width_x = 0;
    width_y = 0;
    zmp_x = 0;
    zmp_y = 0;
    if_finish_ = false;
    plot_once_ = false;
    ready_to_stop_ = false;
    name_cont_ = 0;
    StartHeight_ = 1;
    T_DSP_ = 0;
    Step_Count_ = 0;
    Stepout_flag_X_ = false;
    Stepout_flag_Y_ = false;
    Control_Step_length_X_ = 0;
    Control_Step_length_Y_ = 0;
    com_x = 0;
    com_y = 0;
    foot_hight = 0;
    c_hight = 0;
    board_hight = 0;
    pz_ = COM_HEIGHT;
    new_com = COM_HEIGHT-(Lap + Calve - Length_Leg);
    com_y_swing = 0;
    rightfoot_shift_z = 0;

    /*步態儲存*/
    cruise_command <<
                 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;
    pre_cruise_command <<
                 0, 0,
                 0, 0,
                 0, 0;
    /*-------*/
    A=0,B=0,aa1=0,aa2=0,d1=0,d2=0;
}
WalkingGaitByLIPM::~WalkingGaitByLIPM()
{    }

void WalkingGaitByLIPM::initialize()
{
    parameterinfo->complan.time_point_ = 0;
    parameterinfo->complan.sample_point_ = 0;

    std::vector<float> temp;
	if(map_walk.empty())
	{
		map_walk["l_foot_x"] = temp;
        map_walk["l_foot_y"] = temp;
        map_walk["l_foot_z"] = temp;
        // map_walk["l_foot_t"] = temp;
        map_walk["r_foot_x"] = temp;
        map_walk["r_foot_y"] = temp;
        map_walk["r_foot_z"] = temp;
        // map_walk["r_foot_t"] = temp;
        map_walk["com_x"] = temp;
		map_walk["com_y"] = temp;
        map_walk["now_step_"] = temp;
		map_walk["ideal_zmp_x"] = temp;
		map_walk["ideal_zmp_y"] = temp;
        
		map_walk["A"] = temp;
        map_walk["B"] = temp;
        map_walk["aa1"] = temp;
        map_walk["aa2"] = temp;
        map_walk["d1"] = temp;
        map_walk["d2"] = temp;
        map_walk["time_point_"] = temp;
        // map_walk["case"] = temp;
        map_walk["foot"] = temp;
        map_walk["sensor.roll"] = temp;
		map_walk["sensor.pitch"] = temp;
		map_walk["sensor.yaw"] = temp;
        
        map_walk["displacement_x"] = temp;
        // map_walk["var_theta_"] = temp;
        map_walk["Cpz"] = temp;
        map_walk["Cpx"] = temp;
	}
} 
void WalkingGaitByLIPM::readWalkParameter()
{
    period_t_ = parameterinfo->parameters.Period_T;
    T_DSP_ = parameterinfo->parameters.OSC_LockRange;
    if(parameterinfo->LCBalanceOn && parameterinfo->walking_mode == Continuous)
    {
        if(abs(step_length_) < 4)
        {
            lift_height_ = parameterinfo->parameters.BASE_Default_Z+abs(step_length_)*0.25;
        }
        else
        {
            lift_height_ = parameterinfo->parameters.BASE_Default_Z+abs(step_length_)*0.3;
        }
    }
    else{lift_height_ = parameterinfo->parameters.BASE_Default_Z;}
    lift_height_ = parameterinfo->parameters.BASE_Default_Z;
    board_hight = parameterinfo->parameters.BASE_LIFT_Z;
    com_y_swing = parameterinfo->parameters.X_Swing_Range;
    parameterinfo->parameters.DSP = (parameterinfo->parameters.Period_T*parameterinfo->parameters.OSC_LockRange)/2;
    parameterinfo->parameters.SSP = (parameterinfo->parameters.Period_T-(parameterinfo->parameters.Period_T*parameterinfo->parameters.OSC_LockRange));
    t1 = parameterinfo->parameters.DSP* 0.001;
    t2 = (parameterinfo->parameters.DSP + parameterinfo->parameters.SSP)* 0.001;
    
    if(parameterinfo->parameters.X_Swing_COM >3)
    {
        rightfoot_shift_z = 3;
    }
    else if (parameterinfo->parameters.X_Swing_COM < 0)
    {
        rightfoot_shift_z = 0;
    }
    else
    {
        rightfoot_shift_z = parameterinfo->parameters.X_Swing_COM;
    }

    if(parameterinfo->parameters.Y_Swing_Range <= 0)
    {
        width_size_ = 4.5;
    }
    else
    {
        width_size_ = parameterinfo->parameters.Y_Swing_Range;
    }
    if(parameterinfo->complan.walking_state == StartStep)
    {
        if(parameterinfo->walking_mode == 2 ||parameterinfo->walking_mode==3 || parameterinfo->walking_mode == 0)
        {
            cruise_command <<
                    parameterinfo->X, parameterinfo->X, parameterinfo->X,
                    parameterinfo->Y, parameterinfo->Y, parameterinfo->Y,
                    parameterinfo->THTA, parameterinfo->THTA, parameterinfo->THTA;
        }
        else
        {
            cruise_command <<
                    0, 0, parameterinfo->X,
                    0, 0, parameterinfo->Y,
                    0, 0, parameterinfo->THTA;
        }
    }
    else
    {
        pre_cruise_command.rightCols(1) = cruise_command.leftCols(1);
        pre_cruise_command.leftCols(1) = pre_cruise_command.rightCols(1);
        
        cruise_command.leftCols(2) = cruise_command.rightCols(2);
        if(pre_cruise_command(0, 1) != 0 && cruise_command(0, 0) == 0)
            cruise_command(0, 1) = 0;
        else if(pre_cruise_command(0, 1)*cruise_command(0, 0) < 0)
            cruise_command(0, 0) = 0;
        
        cruise_command.rightCols(1) << parameterinfo->X, parameterinfo->Y, parameterinfo->THTA;
        // cout << cruise_command <<endl
        //     << pre_cruise_command <<endl;
    }
    
}
 
void WalkingGaitByLIPM::readWalkData()
{
    if(pre_step_ != now_step_)
    {
        step_length_ = cruise_command(0,0);//parameterinfo->X ;//+ Control_Step_length_X_;
        shift_length_ = cruise_command(1,0);//parameterinfo->Y ;//+ Control_Step_length_Y_;
        if((var_theta_ >= 0) && ((pre_step_ % 2) == 1))
        {
            var_theta_ = cruise_command(2,0);
        }
        else if((var_theta_ <= 0) && ((pre_step_ % 2) == 0))
        {
            var_theta_ = cruise_command(2,0);
        }
        abs_theta_ = fabs(var_theta_);
        single_var_theta_ = cruise_command(2,0);
        // if( ( Stepout_flag_X_ || Stepout_flag_Y_ ) && Step_Count_ >= 2)
        // {
        //     Stepout_flag_X_ = false;
        //     Stepout_flag_Y_ = false;
        //     Control_Step_length_X_ = 0;
        //     Control_Step_length_Y_ = 0;
        //     Step_Count_ = 0;
        // }
        // else if( ( Stepout_flag_X_ || Stepout_flag_Y_ ) && (Step_Count_ <= 1))
        // {
        //     if(((pre_step_%2 == 0) && (Control_Step_length_Y_ < 0))||((pre_step_%2 == 1) && (Control_Step_length_Y_ > 0)))
        //     {

        //     }
        //     else
        //     {
        //         Step_Count_ += 1;
        //         step_length_ -= Control_Step_length_X_;
        //         // shift_length_ -= Control_Step_length_Y_;
        //     }
        // }
        // else
        // {

        // }

        is_parameter_load_ = true;
    }
}
void WalkingGaitByLIPM::resetParameter()
{
    is_parameter_load_ = false;
    if_finish_ = true;
    time_point_ = 0;
    sample_point_ = 0;
    now_step_ = 0;
    pre_step_ = -1;
    step_ = 99999;//999;
    step_length_ = 0;
    last_displacement_x = 0;
    shift_length_ = 0;
    last_displacement_y = 0;
    theta_ = 0;
    abs_theta_ = 0;
    last_abs_theta_ = 0;
    left_step_ = 0;
    right_step_ = 0;
    base_x = 0;
    now_left_x_ = 0;
    now_right_x_ = 0;
    footstep_x = 0;
    footstep_y = -width_size_;
    base_y = 0;
    now_left_y_ = 0;
    now_right_y_ = 0;
    last_base_x = 0;
    last_base_y = 0;
    last_theta_ = 0;
    now_width_ = 0;
    width_x = 0;
    width_y = 0;
    displacement_x = 0;
    displacement_y = 0;
    zmp_x = 0;
    zmp_y = 0;
    StartHeight_ = 1;
    T_DSP_ = 0;
    Step_Count_ = 0;
    Stepout_flag_X_ = false;
    Stepout_flag_Y_ = false;
    Control_Step_length_X_ = 0;
    Control_Step_length_Y_ = 0;
    com_x = 0;
    com_y = 0;
    foot_hight = 0;
    c_hight = 0;
    board_hight = 0;
    pz_ = COM_HEIGHT;
    new_com = COM_HEIGHT-(Lap + Calve - Length_Leg);
    /*步態儲存*/
    cruise_command <<
                 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;
    pre_cruise_command <<
                 0, 0,
                 0, 0,
                 0, 0;
    /*-------*/
    A=0,B=0,aa1=0,aa2=0,d1=0,d2=0;
}  

void WalkingGaitByLIPM::readDSPParameter()
{
    d1 = cruise_command(0, 0)/2;
    d2 = cruise_command(0, 1)/2;

    if(now_step_ < 2)
    {
        if(parameterinfo->walking_mode == 2 ||parameterinfo->walking_mode==3 || parameterinfo->walking_mode == 0)
        {
            if(now_step_==0)
            {
                A   = 0;
                aa1 = 0;
                aa2 = 0;
                d1  = 0;
            }
            else
            {
                aa1 = 0;
                aa2 = 0;
                d2  = 0;
            }
            
        }
        else
        {
            A   = 0;
            aa1 = 0;
            aa2 = 0;
            d1  = 0;
            d2  = 0;
        }
    }
    else if(now_step_ == 2)
    {
        A   = 0;
        aa1 = 0;
        aa2 = 0;
        d1  = 0;
    }
    else if(now_step_ == step_)
    {
        aa1 = 0;
        aa2 = 0;
        d2  = 0;
    }
    else if(cruise_command(0, 1)== 0)
    {
        if(cruise_command(0, 0)!= 0)
        {
            aa1 = 0;
            aa2 = 0;
            d1 = cruise_command(0, 0)/2;
            d2 = 0;
        }
        else
        {
            aa1 = 0;
            aa2 = 0;
            d1  = 0;
            d2  = 0;
        }
    }
    else
    {
        if(pre_cruise_command(0, 1)== 0 && cruise_command(0, 0)== 0)
        {
            aa1 = 0;
            aa2 = 0;
            d1 = 0;
            d2 = cruise_command(0, 1)/2;
        }
        else
        {
            aa1 = cruise_command(0, 0)*movingZMP;
            aa2 = cruise_command(0, 1)*movingZMP;
            d1  = cruise_command(0, 0)/2;
            d2  = cruise_command(0, 1)/2;
        }
    }
    
}

void WalkingGaitByLIPM::process()
{
    readWalkParameter();    /* 步週期 單位[ms] ,T_DSP 單位[s] ,抬腳高 單位[cm] */

    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);
    sample_point_++;
    time_point_ = sample_point_ * sample_time_;
    Tc_ = sqrt(COM_HEIGHT/g_);          /* 機器人的自然週期 */

    TT_ = (double)period_t_ * 0.001;    /* 步週期 單位[s] */

    /* 步週期內時刻 [0,TT_] 單位[s] */
    t_ = (double)((time_point_ - (int)sample_time_) % period_t_ + sample_time_)/1000;

    now_step_ = (sample_point_ - 1)/(int)(period_t_ / sample_time_);

    if(now_step_ == step_)
        parameterinfo->complan.walking_state = StopStep;
    else if(now_step_ < STARTSTEPCOUNTER)
        parameterinfo->complan.walking_state = StartStep;
    else if(now_step_ > step_)
    {
        if_finish_ = true;
        plot_once_ = true;
        parameterinfo->complan.walking_stop = true;
        parameterinfo->walking_mode = 0;

    }
    else if(now_step_ == STARTSTEPCOUNTER)
    {
        parameterinfo->complan.walking_state = FirstStep;
    }
    else
    {
        parameterinfo->complan.walking_state = Repeat;
    }

    if(pre_step_ != now_step_)          /* 確認是否換步 */
    {
        if((now_step_ % 2) == 1 && now_step_ > 1)
        {
            parameterinfo->parameters.foot_flag = false;
            left_step_++;
        }
        else if((now_step_ % 2) == 0 && now_step_ > 1)
        {
            parameterinfo->parameters.foot_flag = true;
            right_step_++;
        }

        
        if((pre_step_ % 2) == 1)
        {
            now_right_x_ = footstep_x;
            now_right_y_ = footstep_y;
        }
        else if((pre_step_ % 2) == 0)
        {
            now_left_x_ = footstep_x;
            now_left_y_ = footstep_y;
        }
        else if(pre_step_ == -1)
        {
            footstep_x = 0;
            footstep_y = -width_size_;
            now_right_x_ = footstep_x;
            now_right_y_ = footstep_y;
            now_left_x_ = 0;
            now_left_y_ = width_size_;
        }
        

        last_zmp_x = zmp_x;
        last_zmp_y = zmp_y;
        zmp_x = footstep_x;
        zmp_y = footstep_y;
        last_displacement_x = displacement_x;   //上次的跨幅
        last_base_x = base_x;         //上次到達的位置
        last_displacement_y = displacement_y; //上次的Y軸位移量
        last_base_y = base_y;           //上次的Y軸位移位置
        last_theta_ = var_theta_;               //前一次的Theta變化量
        last_abs_theta_ = abs_theta_;
        is_parameter_load_ = false;
        
        // if(( Stepout_flag_X_ || Stepout_flag_Y_ ) && Step_Count_ < 2)

        readWalkData();

        if(parameterinfo->complan.walking_state == StartStep)
        {
            theta_ = 0;
            var_theta_ = 0;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = width_x;
            displacement_y = width_y;
            footstep_x += width_x;
            footstep_y += width_y;
        }
        else if(parameterinfo->complan.walking_state == StopStep)
        {
            theta_ = var_theta_;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = width_x;
            displacement_y = width_y;
            footstep_x += width_x;
            footstep_y += width_y;
        }        
        else 
        {
            theta_ = var_theta_;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = (step_length_*cos(theta_)-shift_length_*sin(theta_))+width_x;
            displacement_y = (step_length_*sin(theta_)+shift_length_*cos(theta_))+width_y;
            footstep_x += displacement_x;
            footstep_y += displacement_y;
        }

        base_x = (footstep_x + zmp_x)/2;
        base_y = (footstep_y + zmp_y)/2;
        

    }
    pre_step_ = now_step_;//步數儲存




    if(ready_to_stop_)
    {
        step_ = now_step_ + 1;
        ready_to_stop_ = false;
    }
    
    switch (parameterinfo->complan.walking_state)
    {
    case StartStep:
        // map_walk.find("case")->second.push_back(0);
        /* 初始化參數 
        base_x = 0;
        now_right_x_ = 0;
        now_right_y_ = 0;
        now_left_x_ = 0;
        now_left_y_ = 0;
        step_length_ = 0;
        last_displacement_x = 0;//上次的跨幅
        last_base_x = 0;//上次到達的位置
        last_displacement_y = 0;//上次的Y軸位移量
        last_base_y = 0;//上次的Y軸位移位置
        base_y = 0;//現在要到的Y軸位移位置
        last_theta_ = 0;//前一次的Theta量
        shift_length_ = 0;
        */

        vx0_ = wComVelocityInit(0, 0, zmp_x, TT_, Tc_);
        px_ = wComPosition(0, vx0_, zmp_x, t_, Tc_);
        vy0_ = wComVelocityInit(0, 0, zmp_y, TT_, Tc_);
        // py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);

        StartHeight_ = StartHeight_;
        if((now_step_ % 2) == 1)
        {
            py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);
            lpx_ = zmp_x;
            rpx_ = wFootPositionRepeat(now_right_x_, 0, t_, TT_, T_DSP_);
            lpy_ = zmp_y;
            rpy_ = wFootPositionRepeat(now_right_y_, 0, t_, TT_, T_DSP_);
            lpz_ = 0;
            rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);

            lpt_ = 0;
            rpt_ = 0;
        }
        else if((now_step_ % 2) == 0)
        {
            py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_)+com_y_swing*sin(PI*t_/TT_);
            lpx_ = wFootPositionRepeat(now_left_x_, 0, t_, TT_, T_DSP_);
            rpx_ = zmp_x;
            lpy_ = wFootPositionRepeat(now_left_y_, 0, t_, TT_, T_DSP_);
            rpy_ = zmp_y;
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
            rpz_ = 0;

            lpt_ = 0;
            rpt_ = 0;

        }
        break;
    case FirstStep:
        vx0_ = wComVelocityInit(0, base_x, zmp_x, TT_, Tc_);
        px_ = wComPosition(0, vx0_, zmp_x, t_, Tc_);
        vy0_ = wComVelocityInit(0, base_y, zmp_y, TT_, Tc_);
        py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);
        lpx_ = wFootPosition(now_left_x_, displacement_x, t_, TT_, T_DSP_);
        rpx_ = zmp_x;
        lpy_ = wFootPosition(now_left_y_, displacement_y-now_width_, t_, TT_, T_DSP_);
        rpy_ = zmp_y;
        lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
        rpz_ = 0;

        lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
        rpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
    
        break;
    case StopStep:
        vx0_ = wComVelocityInit(last_base_x, base_x, zmp_x, TT_, Tc_);
        px_ = wComPosition(last_base_x, vx0_, zmp_x, t_, Tc_);
        vy0_ = wComVelocityInit(last_base_y, base_y, zmp_y, TT_, Tc_);
        py_ = wComPosition(last_base_y, vy0_, zmp_y, t_, Tc_);

        if((now_step_ % 2) == 1)
        {
            lpx_ = zmp_x;            
            lpy_ = zmp_y;
            rpx_ = wFootPosition(now_right_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
            rpy_ = wFootPosition(now_right_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
            lpz_ = 0;
            rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);

            lpt_ = 0;
            rpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
        }
        else if((now_step_ % 2) == 0)
        {
            lpx_ = wFootPosition(now_left_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
            rpx_ = zmp_x;
            lpy_ = wFootPosition(now_left_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
            rpy_ = zmp_y;
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
            rpz_ = 0;

            lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            rpt_ = 0;
        }
        break;

    case Repeat:
        vx0_ = wComVelocityInit(last_base_x, base_x, zmp_x, TT_, Tc_);
        px_ = wComPosition(last_base_x, vx0_, zmp_x, t_, Tc_);
        vy0_ = wComVelocityInit(last_base_y, base_y, zmp_y, TT_, Tc_);
        py_ = wComPosition(last_base_y, vy0_, zmp_y, t_, Tc_);

        if((now_step_ % 2) == 1)
        {
            lpx_ = zmp_x;
            rpx_ = wFootPositionRepeat(now_right_x_, (last_displacement_x+displacement_x)/2, t_, TT_, T_DSP_);
            lpy_ = zmp_y;
            rpy_ = wFootPositionRepeat(now_right_y_, (last_displacement_y+displacement_y)/2, t_, TT_, T_DSP_);
            lpz_ = 0;
            rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
            if(var_theta_*last_theta_ >= 0)
            {
                lpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
                rpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            }
            else
            {
                lpt_ = 0;
                rpt_ = 0;
            } 
        }
        else if((now_step_ % 2) == 0)
        {
            lpx_ = wFootPositionRepeat(now_left_x_, (last_displacement_x+displacement_x)/2, t_, TT_, T_DSP_);
            rpx_ = zmp_x;
            lpy_ = wFootPositionRepeat(now_left_y_, (last_displacement_y+displacement_y)/2, t_, TT_, T_DSP_);
            rpy_ = zmp_y;
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
            rpz_ = 0;
            if(var_theta_*last_theta_ >= 0)
            {
                lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
                rpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
            }
            else
            {
                lpt_ = 0;
                rpt_ = 0;
            }
        }
        break;
    default:
        break;
    }

    // if(5>fabs(com_y) && 5>fabs(com_x))
    // {
    // /* 前饋控制 */
    // py_ = py_ + 0.5 * ( py_ - com_y);
    // //px_ = px_ + 0.5 * ( px_ - com_x);
    // /* --- */
    // }
    

    // if(abs(sensor.rpy_[0])>3.5)
    //    py_u = py_  - 0.2 * ( py_ - com_y);
    // else
    //    py_u = py_;   
    // if(balance.pitch_over_limit_)
    //     px_u = px_ - 0.05 * ( px_ - com_x);
    // else
    //     px_u = px_;    
    py_u = py_ + 0.3 * ( py_ - com_y);
    px_u = px_;

    Z_ctrl_ = wForceDifferenceControl(t_, TT_, T_DSP_);

    coordinate_transformation();
    coordinate_offset();
    

    if(now_step_ > step_)
    {
        step_point_lx_ = 0;
        step_point_rx_ = 0;
        step_point_ly_ = 0;
        step_point_ry_ = 0;
        step_point_lz_ = COM_HEIGHT;
        step_point_rz_ = COM_HEIGHT;
        step_point_lthta_ = 0;
        step_point_rthta_ = 0;

        Z_ctrl_ = 0;
        end_point_lx_ = 0;
        end_point_rx_ = 0;
        parameterinfo->points.IK_Point_LX 	= 0;
	    parameterinfo->points.IK_Point_RX 	= 0;
        end_point_ly_ = width_size_ - Length_Pelvis/2;
        end_point_ry_ = -width_size_ + Length_Pelvis/2;
        end_point_lz_ = step_point_lz_- (COM_HEIGHT - Length_Leg);
        end_point_rz_ = step_point_rz_- (COM_HEIGHT - Length_Leg);
        end_point_lthta_ = 0;
        end_point_rthta_ = 0;
        if_finish_ = true;
        delay_push_ = true;
        resetParameter();
        //saveData();
    }
    else
    {
        push_data_ = true; 
    }
    // parameterinfo->points.IK_Point_RX = end_point_rx_;
	// parameterinfo->points.IK_Point_RY = end_point_ry_+balance.y_offset_r;
	// parameterinfo->points.IK_Point_RZ = end_point_rz_-balance.x_offset-balance.y_offset_r;
	// parameterinfo->points.IK_Point_RThta = end_point_rthta_;
	// parameterinfo->points.IK_Point_LX = end_point_lx_;
	// parameterinfo->points.IK_Point_LY = end_point_ly_+balance.y_offset_l;
	// parameterinfo->points.IK_Point_LZ = end_point_lz_-balance.x_offset-balance.y_offset_l;
	// parameterinfo->points.IK_Point_LThta = end_point_lthta_;
}

void WalkingGaitByLIPM::LC()
{
    readWalkParameter();    /* 步週期 單位[ms] ,T_DSP 單位[s] ,抬腳高 單位[cm] */

    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);
    sample_point_++;
    time_point_ = sample_point_ * sample_time_;
    Tc_ = sqrt(COM_HEIGHT/g_);          /* 機器人的自然週期 */

    TT_ = (double)period_t_ * 0.001;    /* 步週期 單位[s] */

    /* 步週期內時刻 [0,TT_] 單位[s] */
    t_ = (double)((time_point_ - (int)sample_time_) % period_t_ + sample_time_)/1000;

    now_step_ = (sample_point_ - 1)/(int)(period_t_ / sample_time_);

    
    if(now_step_ == step_)
    {
        parameterinfo->complan.walking_state = StopStep;
    }        
    else if(now_step_ > step_)
    {
        if_finish_ = true;
        plot_once_ = true;
        parameterinfo->complan.walking_stop = true;
        parameterinfo->walking_mode = 0;

    }
    else if(now_step_ == 0)
    {
        parameterinfo->complan.walking_state = FirstStep;
        ready_to_stop_ = true;
    }
    else if(now_step_ == 1){
        parameterinfo->complan.walking_state = StopStep;
    }
        

    if(pre_step_ != now_step_)          /* 確認是否換步 */
    {
        if((now_step_ % 2) == 1 && now_step_ > 1)
        {
            left_step_++;
        }
        else if((now_step_ % 2) == 0 && now_step_ > 1)
        {
            right_step_++;
        }

        if((pre_step_ % 2) == 1)
        {
            now_right_x_ = footstep_x;
            now_right_y_ = footstep_y;
        }
        else if((pre_step_ % 2) == 0)
        {
            now_left_x_ = footstep_x;
            now_left_y_ = footstep_y;
        }
        else if(pre_step_ == -1)
        {
            footstep_x = 0;
            footstep_y = -width_size_;
            now_right_x_ = footstep_x;
            now_right_y_ = footstep_y;
            now_left_x_ = 0;
            now_left_y_ = width_size_;
        }
    
        last_zmp_x = zmp_x;
        last_zmp_y = zmp_y;
        zmp_x = footstep_x;
        zmp_y = footstep_y;
        last_displacement_x = displacement_x;   //上次的跨幅
        last_base_x = base_x;         //上次到達的位置
        last_displacement_y = displacement_y; //上次的Y軸位移量
        last_base_y = base_y;           //上次的Y軸位移位置
        last_theta_ = var_theta_;               //前一次的Theta變化量
        last_abs_theta_ = abs_theta_;
        is_parameter_load_ = false;

        readWalkData();

        if(parameterinfo->complan.walking_state == StopStep)
        {
            theta_ = 0;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = width_x;
            displacement_y = width_y;
            footstep_x += width_x;
            footstep_y += width_y;
        }        
        else 
        {
            theta_ = 0;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = (step_length_*cos(theta_)-shift_length_*sin(theta_))+width_x;
            displacement_y = (step_length_*sin(theta_)+shift_length_*cos(theta_))+width_y;
            footstep_x += displacement_x;
            footstep_y += displacement_y;
        }

        base_x = (footstep_x + zmp_x)/2;
        base_y = (footstep_y + zmp_y)/2;
    }
    pre_step_ = now_step_;//步數儲存

    if(ready_to_stop_)
    {
        step_ = now_step_ + 1;
        ready_to_stop_ = false;
    }
    
    switch (parameterinfo->complan.walking_state)
    {
    case FirstStep:
        
        vx0_ = wComVelocityInit(0, base_x, zmp_x, TT_, Tc_);
        px_ = wComPosition(0, vx0_, zmp_x, t_, Tc_);
        vy0_ = wComVelocityInit(0, base_y+com_y_swing, zmp_y, TT_, Tc_);
        py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);
        lpx_ = wFootPosition(now_left_x_, displacement_x, t_, TT_, T_DSP_);
        rpx_ = zmp_x;
        lpy_ = wFootPosition(now_left_y_, displacement_y-now_width_, t_, TT_, T_DSP_);
        rpy_ = zmp_y;
        if(displacement_x == 0)
        {
            foot_hight = 0;
        }
        else
        {
            foot_hight = ((lpx_ - now_left_x_)/displacement_x)*board_hight;
            c_hight =  (px_/base_x)*board_hight/2;
        }         
        lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_)+foot_hight;
        pz_ = COM_HEIGHT + c_hight;        
        rpz_ = 0;

        lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
        rpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
    
        break;
    case StopStep:
        parameterinfo->LCFinishFlag = true;
        vx0_ = wComVelocityInit(last_base_x, base_x, zmp_x, TT_, Tc_);
        px_ = wComPosition(last_base_x, vx0_, zmp_x, t_, Tc_);
        vy0_ = wComVelocityInit(last_base_y+com_y_swing, base_y, zmp_y, TT_, Tc_);
        py_ = wComPosition(last_base_y+com_y_swing, vy0_, zmp_y, t_, Tc_);

        lpx_ = lpx_;            
        lpy_ = zmp_y;
        rpx_ = wFootPosition(now_right_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
        rpy_ = wFootPosition(now_right_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
        if((last_displacement_x+displacement_x) == 0)
        {       
            foot_hight = 0;
        }
        else
        {
            foot_hight = ((rpx_ - now_right_x_)/(last_displacement_x+displacement_x))*board_hight ;  
            c_hight = ((px_ - last_base_x)/(base_x-last_base_x))*board_hight/2;
        }
        lpz_ = board_hight;
        rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_)+foot_hight+ rightfoot_shift_z*sin(PI*t_/TT_);
        pz_ = COM_HEIGHT + c_hight+board_hight/2;
        lpt_ = 0;
        rpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
        
    default:

        break;
    }

    py_u = py_;
    px_u = px_;
    // py_u = py_ - 0.2 * ( py_ - com_y);
    // px_u = px_ + 0.1 * ( px_ - com_x);

    Z_ctrl_ = 0;

    coordinate_transformation();
    coordinate_offset();


    if(now_step_ > step_)
    {
        final_step();
        delay_push_ = true;
    }
    else
    {
        push_data_ = true; 
    }
    // parameterinfo->points.IK_Point_RX = end_point_rx_;
	// parameterinfo->points.IK_Point_RY = end_point_ry_;
	// parameterinfo->points.IK_Point_RZ = end_point_rz_;
	// parameterinfo->points.IK_Point_RThta = end_point_rthta_;
	// parameterinfo->points.IK_Point_LX = end_point_lx_;
	// parameterinfo->points.IK_Point_LY = end_point_ly_;
	// parameterinfo->points.IK_Point_LZ = end_point_lz_;
	// parameterinfo->points.IK_Point_LThta = end_point_lthta_;
}

void WalkingGaitByLIPM::LC_dsp()
{
    readWalkParameter();    /* 步週期 單位[ms] ,T_DSP 單位[s] ,抬腳高 單位[cm] */

    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);
    sample_point_++;
    time_point_ = sample_point_ * sample_time_;
    Tc_ = sqrt(COM_HEIGHT/g_);          /* 機器人的自然週期 */

    TT_ = (double)period_t_ * 0.001;    /* 步週期 單位[s] */

    /* 步週期內時刻 [0,TT_] 單位[s] */
    t_ = (double)((time_point_ - (int)sample_time_) % period_t_ + sample_time_)/1000;

    now_step_ = (sample_point_ - 1)/(int)(period_t_ / sample_time_);

    
    if(now_step_ == step_)
    {
        parameterinfo->complan.walking_state = StopStep;
    }        
    else if(now_step_ > step_)
    {
        if_finish_ = true;
        plot_once_ = true;
        parameterinfo->complan.walking_stop = true;
        parameterinfo->walking_mode = 0;

    }
    else if(now_step_ == 0)
    {
        parameterinfo->complan.walking_state = FirstStep;
        ready_to_stop_ = true;
    }
    else if(now_step_ == 1){
        parameterinfo->complan.walking_state = StopStep;
    }
        

    if(pre_step_ != now_step_)          /* 確認是否換步 */
    {
        if((now_step_ % 2) == 1 && now_step_ > 1)
        {
            left_step_++;
        }
        else if((now_step_ % 2) == 0 && now_step_ > 1)
        {
            right_step_++;
        }

        if((pre_step_ % 2) == 1)
        {
            now_right_x_ = footstep_x;
            now_right_y_ = footstep_y;
        }
        else if((pre_step_ % 2) == 0)
        {
            now_left_x_ = footstep_x;
            now_left_y_ = footstep_y;
        }
        else if(pre_step_ == -1)
        {
            footstep_x = 0;
            footstep_y = -width_size_;
            now_right_x_ = footstep_x;
            now_right_y_ = footstep_y;
            now_left_x_ = 0;
            now_left_y_ = width_size_;
        }
    
        last_zmp_x = zmp_x;
        last_zmp_y = zmp_y;
        zmp_x = footstep_x;
        zmp_y = footstep_y;
        last_displacement_x = displacement_x;   //上次的跨幅
        last_base_x = base_x;         //上次到達的位置
        last_displacement_y = displacement_y; //上次的Y軸位移量
        last_base_y = base_y;           //上次的Y軸位移位置
        last_theta_ = var_theta_;               //前一次的Theta變化量
        last_abs_theta_ = abs_theta_;
        is_parameter_load_ = false;

        readWalkData();

        if(parameterinfo->complan.walking_state == StopStep)
        {
            theta_ = 0;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = width_x;
            displacement_y = width_y;
            footstep_x += width_x;
            footstep_y += width_y;
        }        
        else 
        {
            theta_ = 0;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = (step_length_*cos(theta_)-shift_length_*sin(theta_))+width_x;
            displacement_y = (step_length_*sin(theta_)+shift_length_*cos(theta_))+width_y;
            footstep_x += displacement_x;
            footstep_y += displacement_y;
        }
        // cout << "displacement_x: " <<displacement_x<<endl
        //      << "displacement_y: " <<displacement_y<<endl;

        base_x = (footstep_x + zmp_x)/2;
        base_y = (footstep_y + zmp_y)/2;
    }
    pre_step_ = now_step_;//步數儲存

    if(ready_to_stop_)
    {
        step_ = now_step_ + 1;
        ready_to_stop_ = false;
    }
    
    if(T_DSP_ == 0)
    {
        switch (parameterinfo->complan.walking_state)
        {
        case FirstStep:
            
            vx0_ = wComVelocityInit(0, base_x, zmp_x, TT_, Tc_);
            px_ = wComPosition(0, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(0, base_y+com_y_swing, zmp_y, TT_, Tc_);
            py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);
            lpx_ = wFootPosition(now_left_x_, displacement_x, t_, TT_, T_DSP_);
            rpx_ = zmp_x;
            lpy_ = wFootPosition(now_left_y_, displacement_y-now_width_, t_, TT_, T_DSP_);
            rpy_ = zmp_y;
            if(displacement_x == 0)
            {
                foot_hight = 0;
            }
            else
            {
                foot_hight = ((lpx_ - now_left_x_)/displacement_x)*board_hight;
                c_hight =  (px_/base_x)*board_hight/2;
            }         
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_)+foot_hight;
            pz_ = COM_HEIGHT + c_hight;        
            rpz_ = 0;

            lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            rpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
        
            break;
        case StopStep:
            parameterinfo->LCFinishFlag = true;
            vx0_ = wComVelocityInit(last_base_x, base_x, zmp_x, TT_, Tc_);
            px_ = wComPosition(last_base_x, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(last_base_y+com_y_swing, base_y, zmp_y, TT_, Tc_);
            py_ = wComPosition(last_base_y+com_y_swing, vy0_, zmp_y, t_, Tc_);

            lpx_ = lpx_;            
            lpy_ = zmp_y;
            rpx_ = wFootPosition(now_right_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
            rpy_ = wFootPosition(now_right_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
            if((last_displacement_x+displacement_x) == 0)
            {       
                foot_hight = 0;
            }
            else
            {
                foot_hight = ((rpx_ - now_right_x_)/(last_displacement_x+displacement_x))*board_hight ;  
                c_hight = ((px_ - last_base_x)/(base_x-last_base_x))*board_hight/2;
            }
            lpz_ = board_hight;
            rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_)+foot_hight+ rightfoot_shift_z*sin(PI*t_/TT_);
            pz_ = COM_HEIGHT + c_hight+board_hight/2;
            lpt_ = 0;
            rpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            
        default:

            break;
        }
    }
    else
    {
        A = zmp_x;
        B = zmp_y;
        X1=0,X2=0,X3=0;
        Y1=0,Y2=0,Y3=0;
        VX1=0,VX2=0,VX3=0;
        VY1=0,VY2=0,VY3=0;
        readDSPParameter();
        switch (parameterinfo->complan.walking_state)
        {
        case FirstStep:
            lpx_ = wFootPosition(now_left_x_, displacement_x, t_, TT_, T_DSP_);
            rpx_ = zmp_x;
            lpy_ = wFootPosition(now_left_y_, displacement_y-now_width_, t_, TT_, T_DSP_);
            rpy_ = zmp_y;
             if(displacement_x == 0)
            {
                foot_hight = 0;
            }
            else
            {
                foot_hight = ((lpx_ - now_left_x_)/displacement_x)*board_hight;
                c_hight =  (px_/base_x)*board_hight/2;
            }         
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_)+foot_hight;
            pz_ = COM_HEIGHT + c_hight;        
            rpz_ = 0;

            lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            rpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
        
            break;
        case StopStep:
            parameterinfo->LCFinishFlag = true;
            lpx_ = lpx_;            
            lpy_ = zmp_y;
            rpx_ = wFootPosition(now_right_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
            rpy_ = wFootPosition(now_right_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
            if((last_displacement_x+displacement_x) == 0)
            {       
                foot_hight = 0;
            }
            else
            {
                foot_hight = ((rpx_ - now_right_x_)/(last_displacement_x+displacement_x))*board_hight ;  
                c_hight = ((px_ - last_base_x)/(base_x-last_base_x))*board_hight/2;
            }
            lpz_ = board_hight;
            rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_)+foot_hight+ rightfoot_shift_z*sin(PI*t_/TT_);
            pz_ = COM_HEIGHT + c_hight+board_hight/2;
            lpt_ = 0;
            rpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            break;

        
        default:
            break;
        }
        L1 = A-d1;
        L2 = A-aa1-(((aa1+aa2)*t1)/(t2-t1));
        L3 = A+aa2-( ((d2-aa2)*t2)/(TT_-t2) );
        
        K1 = (d1-aa1)/(t1);
        K2 = ((aa1+aa2))/(t2-t1);
        K3 = (d2-aa2)/(TT_-t2);

        Y1T = B*(TT_-Tc_*sinh(TT_/Tc_))/t1;
        Y2T = B*(1-cosh((TT_-t1)/Tc_))-B*(TT_-t1*cosh((TT_-t1)/Tc_)-Tc_*sinh((TT_-t1)/Tc_))/t1;
        Y3T = B*t2*(1-cosh((TT_-t2)/Tc_))/t1-B*(TT_-t2*cosh((TT_-t2)/Tc_)-Tc_*sinh((TT_-t2)/Tc_))/t1;
        
        dY0 = DSP_X_velocity_0(last_base_y,base_y,TT_,Tc_,Y1T,Y2T,Y3T);//%-1*(Y1T+Y2T+Y3T)/(Tc_*sinh(period_t_/Tc_));

        X1T = WLIPM_com_X1(L1,K1,TT_,Tc_);
        X2T = WLIPM_com_X2(L2,L1,K2,K1,TT_,Tc_,t1);
        X3T = WLIPM_com_X3(L3,L2,K3,K2,TT_,Tc_,t2);

        dX0 = DSP_X_velocity_0(A-d1,A+d2,TT_,Tc_,X1T,X2T,X3T) ;//+ (Tc_^2*q/C^2)*(cosh(period_t_/Tc_)-1)/(Tc_*sinh(period_t_/Tc_));

        if(t_  <= t1)
        {
            // zmp_x_DSP = t_*(d1-aa1)/t1+(A-d1);
            // zmp_y_DSP = last_py*((t1-t_)/t1)+B*t_/t1;
                        
            X1 = WLIPM_com_X1(L1,K1,t_,Tc_);
            Y1 = B*(t_-Tc_*sinh(t_/Tc_))/t1;
        }
        else if (t_ <= t2)
        {
            // zmp_x_DSP = (aa1+aa2)*(t_-t1)/(t2-t1)+(A-aa1);
            // zmp_y_DSP = B;
            
            X1 = WLIPM_com_X1(L1,K1,t_,Tc_);
            X2 = WLIPM_com_X2(L2,L1,K2,K1,t_,Tc_,t1);
            Y1 = B*(t_-Tc_*sinh(t_/Tc_))/t1;
            Y2 = B*(1-cosh((t_-t1)/Tc_))-B*(t_-t1*cosh((t_-t1)/Tc_)-Tc_*sinh((t_-t1)/Tc_))/t1;
        }
        else
        {
            // zmp_x_DSP = (t_-t2)*(d2-aa2)/(period_t_-t2)+(A+aa2);
            // zmp_y_DSP = B-B*(t_-t2)/(period_t_-t2)+(B+width_size_*(-1)^now_step_)*(t_-t2)/(period_t_-t2);
            // last_py = zmp_y_DSP;           
            X1 = WLIPM_com_X1(L1,K1,t_,Tc_);
            X2 = WLIPM_com_X2(L2,L1,K2,K1,t_,Tc_,t1);
            X3 = WLIPM_com_X3(L3,L2,K3,K2,TT_,Tc_,t2);
            
            Y1 = B*(t_-Tc_*sinh(t_/Tc_))/t1;
            Y2 = B*(1-cosh((t_-t1)/Tc_))-B*(t_-t1*cosh((t_-t1)/Tc_)-Tc_*sinh((t_-t1)/Tc_))/t1;
            Y3 = B*t2*(1-cosh((t_-t2)/Tc_))/t1-B*(t_-t2*cosh((t_-t2)/Tc_)-Tc_*sinh((t_-t2)/Tc_))/t1;
        }
        px_= (A-d1)*cosh(t_/Tc_)+Tc_*dX0*sinh(t_/Tc_) +X1+X2+X3;     //+ (Tc_^2*q/C^2)*(1-cosh(t_/Tc_));
        py_= last_base_y*cosh(t_/Tc_)+Tc_*dY0*sinh(t_/Tc_) +Y1+Y2+Y3;

    }
    py_u = py_;
    px_u = px_;
    // cout << "px: " << px_u <<endl
    //      << "py: " << py_u <<endl
    //      << "pz: " << pz_ <<endl
    //      << "t1: " << t1 <<endl
    //      << "t2: " << t2 <<endl;
    // py_u = py_ - 0.2 * ( py_ - com_y);
    // px_u = px_ + 0.1 * ( px_ - com_x);

    Z_ctrl_ = 0;

    coordinate_transformation();
    coordinate_offset();


    if(now_step_ > step_)
    {
        final_step();
        delay_push_ = true;
    }
    else
    {
        push_data_ = true; 
    }
    // parameterinfo->points.IK_Point_RX = end_point_rx_;
	// parameterinfo->points.IK_Point_RY = end_point_ry_;
	// parameterinfo->points.IK_Point_RZ = end_point_rz_;
	// parameterinfo->points.IK_Point_RThta = end_point_rthta_;
	// parameterinfo->points.IK_Point_LX = end_point_lx_;
	// parameterinfo->points.IK_Point_LY = end_point_ly_;
	// parameterinfo->points.IK_Point_LZ = end_point_lz_;
	// parameterinfo->points.IK_Point_LThta = end_point_lthta_;
}

void WalkingGaitByLIPM::single()
{
    readWalkParameter();    /* 步週期 單位[ms] ,T_DSP 單位[s] ,抬腳高 單位[cm] */

    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);
    sample_point_++;
    time_point_ = sample_point_ * sample_time_;
    Tc_ = sqrt(COM_HEIGHT/g_);          /* 機器人的自然週期 */

    TT_ = (double)period_t_ * 0.001;    /* 步週期 單位[s] */

    /* 步週期內時刻 [0,TT_] 單位[s] */
    t_ = (double)((time_point_ - (int)sample_time_) % period_t_ + sample_time_)/1000;

    now_step_ = (sample_point_ - 1)/(int)(period_t_ / sample_time_);

    
    if(now_step_ == step_)
    {
        parameterinfo->complan.walking_state = StopStep;
    }        
    else if(now_step_ > step_)
    {
        if_finish_ = true;
        plot_once_ = true;
        parameterinfo->complan.walking_stop = true;
        parameterinfo->walking_mode = 0;

    }
    else if(now_step_ == 0)
    {
        parameterinfo->complan.walking_state = FirstStep;
        ready_to_stop_ = true;
    }
    else if(now_step_ == 1){
        parameterinfo->complan.walking_state = StopStep;
    }
        
    readWalkData();

    if(pre_step_ != now_step_)          /* 確認是否換步 */
    {
        if(single_var_theta_<=0){
            if((now_step_ % 2) == 1 && now_step_ > 1)
            {
                left_step_++;
            }
            else if((now_step_ % 2) == 0 && now_step_ > 1)
            {
                right_step_++;
            }

            if((pre_step_ % 2) == 1)
            {
                now_right_x_ = footstep_x;
                now_right_y_ = footstep_y;
            }
            else if((pre_step_ % 2) == 0)
            {
                now_left_x_ = footstep_x;
                now_left_y_ = footstep_y;
            }
            else if(pre_step_ == -1)
            {
                footstep_x = 0;
                footstep_y = -width_size_;
                now_right_x_ = footstep_x;
                now_right_y_ = footstep_y;
                now_left_x_ = 0;
                now_left_y_ = width_size_;
            }
        }else{
            if((now_step_ % 2) == 1 && now_step_ > 1)
            {
                right_step_++;
            }
            else if((now_step_ % 2) == 0 && now_step_ > 1)
            {
                left_step_++;
            }

            if((pre_step_ % 2) == 1)
            {
                now_left_x_ = footstep_x;
                now_left_y_ = footstep_y;
            }
            else if((pre_step_ % 2) == 0)
            {
                now_right_x_ = footstep_x;
                now_right_y_ = footstep_y;
            }
            else if(pre_step_ == -1)
            {
                footstep_x = 0;
                footstep_y = width_size_;
                now_right_x_ = footstep_x;
                now_right_y_ = -footstep_y;
                now_left_x_ = 0;
                now_left_y_ = width_size_;
            }
        }

        last_zmp_x = zmp_x;
        last_zmp_y = zmp_y;
        zmp_x = footstep_x;
        zmp_y = footstep_y;
        last_displacement_x = displacement_x;   //上次的跨幅
        last_base_x = base_x;         //上次到達的位置
        last_displacement_y = displacement_y; //上次的Y軸位移量
        last_base_y = base_y;           //上次的Y軸位移位置
        last_theta_ = 0;//var_theta_;               //前一次的Theta變化量
        last_abs_theta_ = abs_theta_;
        is_parameter_load_ = false;

        

        if(single_var_theta_<=0){
            if(parameterinfo->complan.walking_state == StopStep)
            {
                theta_ = single_var_theta_;
                
                now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
                width_x = -sin(theta_)*now_width_;
                width_y = cos(theta_)*now_width_;
                displacement_x = width_x;
                displacement_y = width_y;
                footstep_x += width_x;
                footstep_y += width_y;
            }        
            else 
            {
                theta_ = single_var_theta_;
                
                now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
                width_x = -sin(theta_)*now_width_;
                width_y = cos(theta_)*now_width_;
                displacement_x = (step_length_*cos(theta_)-shift_length_*sin(theta_))+width_x;
                displacement_y = (step_length_*sin(theta_)+shift_length_*cos(theta_))+width_y;
                footstep_x += displacement_x;
                footstep_y += displacement_y;
            }
        }else{
            if(parameterinfo->complan.walking_state == StopStep)
            {
                theta_ = single_var_theta_;
                
                now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
                width_x = -sin(theta_)*now_width_;
                width_y = -cos(theta_)*now_width_;
                displacement_x = width_x;
                displacement_y = width_y;
                footstep_x += width_x;
                footstep_y += width_y;
            }        
            else 
            {
                theta_ = single_var_theta_;
                
                now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
                width_x = -sin(theta_)*now_width_;
                width_y = -cos(theta_)*now_width_;
                displacement_x = (step_length_*cos(theta_)-shift_length_*sin(theta_))+width_x;
                displacement_y = (step_length_*sin(theta_)+shift_length_*cos(theta_))+width_y;
                footstep_x += displacement_x;
                footstep_y += displacement_y;
            }
        }
        base_x = (footstep_x + zmp_x)/2;
        base_y = (footstep_y + zmp_y)/2;
    }
    pre_step_ = now_step_;//步數儲存

    if(ready_to_stop_)
    {
        step_ = now_step_ + 1;
        ready_to_stop_ = false;
    }
    
    switch (parameterinfo->complan.walking_state)
    {
    case FirstStep:
        if(single_var_theta_<=0){
            vx0_ = wComVelocityInit(0, base_x, zmp_x, TT_, Tc_);
            px_ = wComPosition(0, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(0, base_y+com_y_swing, zmp_y, TT_, Tc_);
            py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);
            lpx_ = wFootPosition(now_left_x_, displacement_x, t_, TT_, T_DSP_);
            rpx_ = zmp_x;
            lpy_ = wFootPosition(now_left_y_, displacement_y-now_width_, t_, TT_, T_DSP_);
            rpy_ = zmp_y;       
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);       
            rpz_ = 0;

            lpt_ = wFootTheta(single_var_theta_, 0, t_, TT_, T_DSP_);
            rpt_ = wFootTheta(single_var_theta_, 1, t_, TT_, T_DSP_);
        }
        else{
            vx0_ = wComVelocityInit(0, base_x, zmp_x, TT_, Tc_);
            px_ = wComPosition(0, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(0, base_y+com_y_swing, zmp_y, TT_, Tc_);
            py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);
            lpx_ = zmp_x;
            rpx_ = wFootPosition(now_right_x_, displacement_x, t_, TT_, T_DSP_);
            lpy_ = zmp_y;
            rpy_ = wFootPosition(now_right_y_, displacement_y+now_width_, t_, TT_, T_DSP_);       
            lpz_ = 0;       
            rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);

            lpt_ = wFootTheta(single_var_theta_, 1, t_, TT_, T_DSP_);
            rpt_ = wFootTheta(single_var_theta_, 0, t_, TT_, T_DSP_);  
        }
        break;
    case StopStep:
        if(single_var_theta_<=0){
            parameterinfo->LCFinishFlag = true;
            vx0_ = wComVelocityInit(last_base_x, base_x, zmp_x, TT_, Tc_);
            px_ = wComPosition(last_base_x, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(last_base_y+com_y_swing, base_y, zmp_y, TT_, Tc_);
            py_ = wComPosition(last_base_y+com_y_swing, vy0_, zmp_y, t_, Tc_);

            lpx_ = zmp_x;            
            lpy_ = zmp_y;
            rpx_ = wFootPosition(now_right_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
            rpy_ = wFootPosition(now_right_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
            lpz_ = 0;
            rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);  

            lpt_ = wFootTheta(single_var_theta_, 1, t_, TT_, T_DSP_);
            rpt_ = wFootTheta(single_var_theta_, 0, t_, TT_, T_DSP_);
        }else{
                parameterinfo->LCFinishFlag = true;
            vx0_ = wComVelocityInit(last_base_x, base_x, zmp_x, TT_, Tc_);
            px_ = wComPosition(last_base_x, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(last_base_y+com_y_swing, base_y, zmp_y, TT_, Tc_);
            py_ = wComPosition(last_base_y+com_y_swing, vy0_, zmp_y, t_, Tc_);

            lpx_ = wFootPosition(now_left_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);            
            lpy_ = wFootPosition(now_left_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
            rpx_ = zmp_x;
            rpy_ = zmp_y;
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
            rpz_ = 0;  

            lpt_ = wFootTheta(single_var_theta_, 0, t_, TT_, T_DSP_);
            rpt_ = wFootTheta(single_var_theta_, 1, t_, TT_, T_DSP_);
        }
        break;
    default:

        break;
    }
    py_u = py_ + 0.6 * ( py_ - com_y);
    px_u = px_;

    Z_ctrl_ = wForceDifferenceControl(t_, TT_, T_DSP_);

    coordinate_transformation();
    coordinate_offset();


    if(now_step_ > step_)
    {
        final_step();
        delay_push_ = true;
    }
    else
    {
        push_data_ = true; 
    }
    // parameterinfo->points.IK_Point_RX = end_point_rx_;
	// parameterinfo->points.IK_Point_RY = end_point_ry_;
	// parameterinfo->points.IK_Point_RZ = end_point_rz_;
	// parameterinfo->points.IK_Point_RThta = end_point_rthta_;
	// parameterinfo->points.IK_Point_LX = end_point_lx_;
	// parameterinfo->points.IK_Point_LY = end_point_ly_;
	// parameterinfo->points.IK_Point_LZ = end_point_lz_;
	// parameterinfo->points.IK_Point_LThta = end_point_lthta_;
}

void WalkingGaitByLIPM::LC_walking()
{
    readWalkParameter();    /* 步週期 單位[ms] ,T_DSP 單位[s] ,抬腳高 單位[cm] */

    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);
    sample_point_++;
    time_point_ = sample_point_ * sample_time_;
    Tc_ = sqrt(COM_HEIGHT/g_);          /* 機器人的自然週期 */

    TT_ = (double)period_t_ * 0.001;    /* 步週期 單位[s] */

    /* 步週期內時刻 [0,TT_] 單位[s] */
    t_ = (double)((time_point_ - (int)sample_time_) % period_t_ + sample_time_)/1000;

    now_step_ = (sample_point_ - 1)/(int)(period_t_ / sample_time_);

    if(now_step_ == step_)
        parameterinfo->complan.walking_state = StopStep;
    else if(now_step_ < STARTSTEPCOUNTER)
        parameterinfo->complan.walking_state = StartStep;
    else if(now_step_ > step_)
    {
        if_finish_ = true;
        plot_once_ = true;
        parameterinfo->complan.walking_stop = true;
        parameterinfo->walking_mode = 0;

    }
    else if(now_step_ == STARTSTEPCOUNTER)
    {
        parameterinfo->complan.walking_state = FirstStep;
    }
    else
    {
        parameterinfo->complan.walking_state = Repeat;
    }
    com_z = wFootPositionZ(COM_rho_Z,t_,period_t_,T_DSP_);


    if(pre_step_ != now_step_)          /* 確認是否換步 */
    {
        if((now_step_ % 2) == 1 && now_step_ > 1)
        {
            parameterinfo->parameters.foot_flag = false;
            left_step_++;
        }
        else if((now_step_ % 2) == 0 && now_step_ > 1)
        {
            parameterinfo->parameters.foot_flag = true;
            right_step_++;
        }

        
        if((pre_step_ % 2) == 1)
        {
            now_right_x_ = footstep_x;
            now_right_y_ = footstep_y;
        }
        else if((pre_step_ % 2) == 0)
        {
            now_left_x_ = footstep_x;
            now_left_y_ = footstep_y;
        }
        else if(pre_step_ == -1)
        {
            footstep_x = 0;
            footstep_y = -width_size_;
            now_right_x_ = footstep_x;
            now_right_y_ = footstep_y;
            now_left_x_ = 0;
            now_left_y_ = width_size_;
        }
        

        last_zmp_x = zmp_x;
        last_zmp_y = zmp_y;
        zmp_x = footstep_x;
        zmp_y = footstep_y;
        last_displacement_x = displacement_x;   //上次的跨幅
        last_base_x = base_x;         //上次到達的位置
        last_displacement_y = displacement_y; //上次的Y軸位移量
        last_base_y = base_y;           //上次的Y軸位移位置
        last_theta_ = var_theta_;               //前一次的Theta變化量
        last_abs_theta_ = abs_theta_;
        is_parameter_load_ = false;
        

        readWalkData();

        if(parameterinfo->complan.walking_state == StartStep)
        {
            theta_ = 0;
            var_theta_ = 0;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = width_x;
            displacement_y = width_y;
            footstep_x += width_x;
            footstep_y += width_y;
        }
        else if(parameterinfo->complan.walking_state == StopStep)
        {
            theta_ = var_theta_;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = width_x;
            displacement_y = width_y;
            footstep_x += width_x;
            footstep_y += width_y;
        }        
        else 
        {
            theta_ = var_theta_;
            
            now_width_ = 2 * width_size_ * (-pow(-1, now_step_+1));
            width_x = -sin(theta_)*now_width_;
            width_y = cos(theta_)*now_width_;
            displacement_x = (step_length_*cos(theta_)-shift_length_*sin(theta_))+width_x;
            displacement_y = (step_length_*sin(theta_)+shift_length_*cos(theta_))+width_y;
            footstep_x += displacement_x;
            footstep_y += displacement_y;
        }

        base_x = (footstep_x + zmp_x)/2;
        base_y = (footstep_y + zmp_y)/2;
        

    }
    pre_step_ = now_step_;//步數儲存

    if(ready_to_stop_)
    {
        step_ = now_step_ + 1;
        ready_to_stop_ = false;
    }
    if(T_DSP_ == 0){
        switch (parameterinfo->complan.walking_state)
        {
        case StartStep:
            vx0_ = wComVelocityInit(0, 0, zmp_x, TT_, Tc_);
            px_ = wComPosition(0, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(0, 0, zmp_y, TT_, Tc_);
            // py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);

            StartHeight_ = StartHeight_;
            if((now_step_ % 2) == 1)
            {
                py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);
                lpx_ = zmp_x;
                rpx_ = wFootPositionRepeat(now_right_x_, 0, t_, TT_, T_DSP_);
                lpy_ = zmp_y;
                rpy_ = wFootPositionRepeat(now_right_y_, 0, t_, TT_, T_DSP_);
                lpz_ = 0;
                rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);

                lpt_ = 0;
                rpt_ = 0;
            }
            else if((now_step_ % 2) == 0)
            {
                py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_)+com_y_swing*sin(PI*t_/TT_);
                lpx_ = wFootPositionRepeat(now_left_x_, 0, t_, TT_, T_DSP_);
                rpx_ = zmp_x;
                lpy_ = wFootPositionRepeat(now_left_y_, 0, t_, TT_, T_DSP_);
                rpy_ = zmp_y;
                lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
                rpz_ = 0;

                lpt_ = 0;
                rpt_ = 0;

            }
            break;
        case FirstStep:
            vx0_ = wComVelocityInit(0, base_x, zmp_x, TT_, Tc_);
            px_ = wComPosition(0, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(0, base_y, zmp_y, TT_, Tc_);
            py_ = wComPosition(0, vy0_, zmp_y, t_, Tc_);
            lpx_ = wFootPosition(now_left_x_, displacement_x, t_, TT_, T_DSP_);
            rpx_ = zmp_x;
            lpy_ = wFootPosition(now_left_y_, displacement_y-now_width_, t_, TT_, T_DSP_);
            rpy_ = zmp_y;
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
            rpz_ = 0;

            lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            rpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
        
            break;
        case StopStep:
            vx0_ = wComVelocityInit(last_base_x, base_x, zmp_x, TT_, Tc_);
            px_ = wComPosition(last_base_x, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(last_base_y, base_y, zmp_y, TT_, Tc_);
            py_ = wComPosition(last_base_y, vy0_, zmp_y, t_, Tc_);

            if((now_step_ % 2) == 1)
            {
                lpx_ = zmp_x;            
                lpy_ = zmp_y;
                rpx_ = wFootPosition(now_right_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
                rpy_ = wFootPosition(now_right_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
                lpz_ = 0;
                rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);

                lpt_ = 0;
                rpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            }
            else if((now_step_ % 2) == 0)
            {
                lpx_ = wFootPosition(now_left_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
                rpx_ = zmp_x;
                lpy_ = wFootPosition(now_left_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
                rpy_ = zmp_y;
                lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
                rpz_ = 0;

                lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
                rpt_ = 0;
            }
            break;

        case Repeat:
            vx0_ = wComVelocityInit(last_base_x, base_x, zmp_x, TT_, Tc_);
            px_ = wComPosition(last_base_x, vx0_, zmp_x, t_, Tc_);
            vy0_ = wComVelocityInit(last_base_y, base_y, zmp_y, TT_, Tc_);
            py_ = wComPosition(last_base_y, vy0_, zmp_y, t_, Tc_);

            if((now_step_ % 2) == 1)
            {
                lpx_ = zmp_x;
                rpx_ = wFootPositionRepeat(now_right_x_, (last_displacement_x+displacement_x)/2, t_, TT_, T_DSP_);
                lpy_ = zmp_y;
                rpy_ = wFootPositionRepeat(now_right_y_, (last_displacement_y+displacement_y)/2, t_, TT_, T_DSP_);
                lpz_ = 0;
                rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
                if(var_theta_*last_theta_ >= 0)
                {
                    lpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
                    rpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
                }
                else
                {
                    lpt_ = 0;
                    rpt_ = 0;
                } 
            }
            else if((now_step_ % 2) == 0)
            {
                lpx_ = wFootPositionRepeat(now_left_x_, (last_displacement_x+displacement_x)/2, t_, TT_, T_DSP_);
                rpx_ = zmp_x;
                lpy_ = wFootPositionRepeat(now_left_y_, (last_displacement_y+displacement_y)/2, t_, TT_, T_DSP_);
                rpy_ = zmp_y;
                lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
                rpz_ = 0;
                if(var_theta_*last_theta_ >= 0)
                {
                    lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
                    rpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
                }
                else
                {
                    lpt_ = 0;
                    rpt_ = 0;
                }
            }
            break;
        default:
            break;
        }
    }
    else
    {
        A = zmp_x;
        B = zmp_y;
        X1=0,X2=0,X3=0;
        Y1=0,Y2=0,Y3=0;
        VX1=0,VX2=0,VX3=0;
        VY1=0,VY2=0,VY3=0;
        readDSPParameter();
        switch (parameterinfo->complan.walking_state)
        {
        case StartStep:
            if((now_step_ % 2) == 1)
            {
                lpx_ = zmp_x;
                rpx_ = wFootPositionRepeat(now_right_x_, 0, t_, TT_, T_DSP_);
                lpy_ = zmp_y;
                rpy_ = wFootPositionRepeat(now_right_y_, 0, t_, TT_, T_DSP_);
                lpz_ = 0;
                rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);

                lpt_ = 0;
                rpt_ = 0;
            }
            else if((now_step_ % 2) == 0)
            {
                lpx_ = wFootPositionRepeat(now_left_x_, 0, t_, TT_, T_DSP_);
                rpx_ = zmp_x;
                lpy_ = wFootPositionRepeat(now_left_y_, 0, t_, TT_, T_DSP_);
                rpy_ = zmp_y;
                lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
                rpz_ = 0;

                lpt_ = 0;
                rpt_ = 0;

            }
            break;
        case FirstStep:
            lpx_ = wFootPosition(now_left_x_, displacement_x, t_, TT_, T_DSP_);
            rpx_ = zmp_x;
            lpy_ = wFootPosition(now_left_y_, displacement_y-now_width_, t_, TT_, T_DSP_);
            rpy_ = zmp_y;
            lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
            rpz_ = 0;

            lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            rpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
        
            break;
        case StopStep:
            if((now_step_ % 2) == 1)
            {
                lpx_ = zmp_x;            
                lpy_ = zmp_y;
                rpx_ = wFootPosition(now_right_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
                rpy_ = wFootPosition(now_right_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
                lpz_ = 0;
                rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);

                lpt_ = 0;
                rpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
            }
            else if((now_step_ % 2) == 0)
            {
                lpx_ = wFootPosition(now_left_x_, (last_displacement_x+displacement_x), t_, TT_, T_DSP_);
                rpx_ = zmp_x;
                lpy_ = wFootPosition(now_left_y_, (last_displacement_y+displacement_y), t_, TT_, T_DSP_);
                rpy_ = zmp_y;
                lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
                rpz_ = 0;

                lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
                rpt_ = 0;
            }
            break;

        case Repeat:
            if((now_step_ % 2) == 1)
            {
                lpx_ = zmp_x;
                rpx_ = wFootPositionRepeat(now_right_x_, (last_displacement_x+displacement_x)/2, t_, TT_, T_DSP_);
                lpy_ = zmp_y;
                rpy_ = wFootPositionRepeat(now_right_y_, (last_displacement_y+displacement_y)/2, t_, TT_, T_DSP_);
                lpz_ = 0;
                rpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
                if(var_theta_*last_theta_ >= 0)
                {
                    lpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
                    rpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
                }
                else
                {
                    lpt_ = 0;
                    rpt_ = 0;
                } 
            }
            else if((now_step_ % 2) == 0)
            {
                lpx_ = wFootPositionRepeat(now_left_x_, (last_displacement_x+displacement_x)/2, t_, TT_, T_DSP_);
                rpx_ = zmp_x;
                lpy_ = wFootPositionRepeat(now_left_y_, (last_displacement_y+displacement_y)/2, t_, TT_, T_DSP_);
                rpy_ = zmp_y;
                lpz_ = wFootPositionZ(lift_height_, t_, TT_, T_DSP_);
                rpz_ = 0;
                if(var_theta_*last_theta_ >= 0)
                {
                    lpt_ = wFootTheta(-last_theta_, 1, t_, TT_, T_DSP_);
                    rpt_ = wFootTheta(-var_theta_, 0, t_, TT_, T_DSP_);
                }
                else
                {
                    lpt_ = 0;
                    rpt_ = 0;
                }
            }
            break;
        default:
            break;
        }
        L1 = A-d1;
        L2 = A-aa1-(((aa1+aa2)*t1)/(t2-t1));
        L3 = A+aa2-( ((d2-aa2)*t2)/(period_t_-t2) );
        
        K1 = (d1-aa1)/(t1);
        K2 = ((aa1+aa2))/(t2-t1);
        K3 = (d2-aa2)/(period_t_-t2);

        Y1T = B*(period_t_-Tc_*sinh(period_t_/Tc_))/t1;
        Y2T = B*(1-cosh((period_t_-t1)/Tc_))-B*(period_t_-t1*cosh((period_t_-t1)/Tc_)-Tc_*sinh((period_t_-t1)/Tc_))/t1;
        Y3T = B*t2*(1-cosh((period_t_-t2)/Tc_))/t1-B*(period_t_-t2*cosh((period_t_-t2)/Tc_)-Tc_*sinh((period_t_-t2)/Tc_))/t1;
        
        dY0 = DSP_X_velocity_0(last_base_y,base_y,period_t_,Tc_,Y1T,Y2T,Y3T);//%-1*(Y1T+Y2T+Y3T)/(Tc_*sinh(period_t_/Tc_));

        X1T = WLIPM_com_X1(L1,K1,period_t_,Tc_);
        X2T = WLIPM_com_X2(L2,L1,K2,K1,period_t_,Tc_,t1);
        X3T = WLIPM_com_X3(L3,L2,K3,K2,period_t_,Tc_,t2);

        dX0 = DSP_X_velocity_0(A-d1,A+d2,period_t_,Tc_,X1T,X2T,X3T) ;//+ (Tc_^2*q/C^2)*(cosh(period_t_/Tc_)-1)/(Tc_*sinh(period_t_/Tc_));

        if(t_  <= t1)
        {
            // zmp_x_DSP = t_*(d1-aa1)/t1+(A-d1);
            // zmp_y_DSP = last_py*((t1-t_)/t1)+B*t_/t1;
                        
            X1 = WLIPM_com_X1(L1,K1,t_,Tc_);
            Y1 = B*(t_-Tc_*sinh(t_/Tc_))/t1;
        }
        else if (t_ <= t2)
        {
            // zmp_x_DSP = (aa1+aa2)*(t_-t1)/(t2-t1)+(A-aa1);
            // zmp_y_DSP = B;
            
            X1 = WLIPM_com_X1(L1,K1,t_,Tc_);
            X2 = WLIPM_com_X2(L2,L1,K2,K1,t_,Tc_,t1);
            Y1 = B*(t_-Tc_*sinh(t_/Tc_))/t1;
            Y2 = B*(1-cosh((t_-t1)/Tc_))-B*(t_-t1*cosh((t_-t1)/Tc_)-Tc_*sinh((t_-t1)/Tc_))/t1;
        }
        else
        {
            // zmp_x_DSP = (t_-t2)*(d2-aa2)/(period_t_-t2)+(A+aa2);
            // zmp_y_DSP = B-B*(t_-t2)/(period_t_-t2)+(B+width_size_*(-1)^now_step_)*(t_-t2)/(period_t_-t2);
            // last_py = zmp_y_DSP;           
            X1 = WLIPM_com_X1(L1,K1,t_,Tc_);
            X2 = WLIPM_com_X2(L2,L1,K2,K1,t_,Tc_,t1);
            X3 = WLIPM_com_X3(L3,L2,K3,K2,period_t_,Tc_,t2);
            
            Y1 = B*(t_-Tc_*sinh(t_/Tc_))/t1;
            Y2 = B*(1-cosh((t_-t1)/Tc_))-B*(t_-t1*cosh((t_-t1)/Tc_)-Tc_*sinh((t_-t1)/Tc_))/t1;
            Y3 = B*t2*(1-cosh((t_-t2)/Tc_))/t1-B*(t_-t2*cosh((t_-t2)/Tc_)-Tc_*sinh((t_-t2)/Tc_))/t1;
        }
        px_= (A-d1)*cosh(t_/Tc_)+Tc_*dX0*sinh(t_/Tc_) +X1+X2+X3;     //+ (Tc_^2*q/C^2)*(1-cosh(t_/Tc_));
        py_= last_base_y*cosh(t_/Tc_)+Tc_*dY0*sinh(t_/Tc_) +Y1+Y2+Y3;

    }
    
    

    py_u = py_ + 0.3 * ( py_ - com_y); //py_u = py_;
    px_u = px_;     //px_u = px_ - 0.05 * ( px_ - com_x);

    Z_ctrl_ = wForceDifferenceControl(t_, TT_, T_DSP_);

    coordinate_transformation();
    coordinate_offset();
    

    if(now_step_ > step_)
    {
        step_point_lx_ = 0;
        step_point_rx_ = 0;
        step_point_ly_ = 0;
        step_point_ry_ = 0;
        step_point_lz_ = COM_HEIGHT;
        step_point_rz_ = COM_HEIGHT;
        step_point_lthta_ = 0;
        step_point_rthta_ = 0;

        Z_ctrl_ = 0;
        end_point_lx_ = 0;
        end_point_rx_ = 0;
        parameterinfo->points.IK_Point_LX 	= 0;
	    parameterinfo->points.IK_Point_RX 	= 0;
        end_point_ly_ = width_size_ - Length_Pelvis/2;
        end_point_ry_ = -width_size_ + Length_Pelvis/2;
        end_point_lz_ = step_point_lz_- (COM_HEIGHT - Length_Leg);
        end_point_rz_ = step_point_rz_- (COM_HEIGHT - Length_Leg);
        end_point_lthta_ = 0;
        end_point_rthta_ = 0;
        if_finish_ = true;
        delay_push_ = true;
        resetParameter();
        //saveData();
    }
    else
    {
        push_data_ = true; 
    }
    // parameterinfo->points.IK_Point_RX = end_point_rx_;
	// parameterinfo->points.IK_Point_RY = end_point_ry_+balance.y_offset_r;
	// parameterinfo->points.IK_Point_RZ = end_point_rz_-balance.x_offset-balance.y_offset_r;
	// parameterinfo->points.IK_Point_RThta = end_point_rthta_;
	// parameterinfo->points.IK_Point_LX = end_point_lx_;
	// parameterinfo->points.IK_Point_LY = end_point_ly_+balance.y_offset_l;
	// parameterinfo->points.IK_Point_LZ = end_point_lz_-balance.x_offset-balance.y_offset_l;
	// parameterinfo->points.IK_Point_LThta = end_point_lthta_;
}

void WalkingGaitByLIPM::final_step()
{
    step_point_lx_ = 0;
    step_point_rx_ = 0;
    step_point_ly_ = 0;
    step_point_ry_ = 0;
    step_point_lz_ = COM_HEIGHT;
    step_point_rz_ = COM_HEIGHT;
    step_point_lthta_ = 0;
    step_point_rthta_ = 0;

    end_point_lx_ = 0;
    end_point_rx_ = 0;
    end_point_ly_ = width_size_ - Length_Pelvis/2;
    end_point_ry_ = -width_size_ + Length_Pelvis/2;
    end_point_lz_ = step_point_lz_- (COM_HEIGHT - Length_Leg);
    end_point_rz_ = step_point_rz_- (COM_HEIGHT - Length_Leg);
    end_point_lthta_ = 0;
    end_point_rthta_ = 0;
    if_finish_ = true;
    resetParameter();
}

void WalkingGaitByLIPM::coordinate_transformation()
{
    /* 座標平移 W to B */
    step_point_lx_W_ = lpx_ - px_u;
    step_point_rx_W_ = rpx_ - px_u;
    step_point_ly_W_ = lpy_ - py_u;
    step_point_ry_W_ = rpy_ - py_u;
    step_point_lz_ = pz_ - lpz_;
    step_point_rz_ = pz_ - rpz_;
    step_point_lthta_ = 0 - lpt_;
    step_point_rthta_ = 0 - rpt_;
    /* --- */

    /* 座標旋轉 W to B */
    step_point_lx_ = (step_point_lx_W_*cos(-theta_)-step_point_ly_W_*sin(-theta_));
    step_point_ly_ = (step_point_lx_W_*sin(-theta_)+step_point_ly_W_*cos(-theta_));
    step_point_rx_ = (step_point_rx_W_*cos(-theta_)-step_point_ry_W_*sin(-theta_));
    step_point_ry_ = (step_point_rx_W_*sin(-theta_)+step_point_ry_W_*cos(-theta_));
    /* --- */    
}

void WalkingGaitByLIPM::coordinate_offset()
{
    end_point_lx_ = step_point_lx_;
    end_point_rx_ = step_point_rx_;
    end_point_ly_ = step_point_ly_ - Length_Pelvis/2;
    end_point_ry_ = step_point_ry_ + Length_Pelvis/2;
    end_point_lz_ = step_point_lz_ - (COM_HEIGHT - Length_Leg) - Z_ctrl_/2;
    end_point_rz_ = step_point_rz_ - (COM_HEIGHT - Length_Leg) + Z_ctrl_/2;
    end_point_lthta_ = step_point_lthta_;
    end_point_rthta_ = step_point_rthta_;
}

double WalkingGaitByLIPM::wComVelocityInit(double x0, double xt, double px, double t, double T)
{
    return (xt - x0*cosh(t/T) + px*(cosh(t/T)-1))/(T*sinh(t/T));
}

double WalkingGaitByLIPM::wComPosition(double x0, double vx0, double px, double t, double T)
{
    return (x0*cosh(t/T) + T*vx0*sinh(t/T) - px*(cosh(t/T)-1));
}

double WalkingGaitByLIPM::wComPosition_y(double x0, double vx0, double px, double t, double T,double Tdsp,double TT)
{
    double SSP = TT*(1-Tdsp)/2;
    double T2 = TT/2;
    if(Tdsp == 0)
        return (x0*cosh(t/T) + T*vx0*sinh(t/T) - px*(cosh(t/T)-1));
    else
    {
        if(t<T2)
            return (x0*cosh(t/T) + T*vx0*sinh(t/T) - px*(cosh(t/T)-1));
        else if (t>=T2 && t<T2+SSP)
            return (x0*cosh(t/T) + T*vx0*sinh(t/T) - px*(cosh(t/T)-1))*sin(PI*(t/(T2+SSP)));
        else
            return 0;
    }
}

double WalkingGaitByLIPM::wFootPosition(const double start, const double length, const double t, const double T, const double T_DSP)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;

    if(t>0 && t<=T*T_DSP/2)
        return start;
    else if(t>T*T_DSP/2 && t<=T*(1-T_DSP/2))
        return length*(omega*new_t-sin(omega*new_t))/(2*PI)+start;
    else
        return length+start;
}

double WalkingGaitByLIPM::wFootPositionRepeat(const double start, const double length, const double t, const double T, const double T_DSP)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;

    if(t>0 && t<=T*T_DSP/2)
        return start;
    else if(t>=T*T_DSP/2 && t<=T*(1-T_DSP/2))
        return 2*length*(omega*new_t-sin(omega*new_t))/(2*PI)+start;
    else
        return 2*length+start;
}

double WalkingGaitByLIPM::wFootPositionZUP(const double height, const double t, const double T, const double T_DSP, const int step, const int board_step, const double board_height)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;

    if(step)
    {
        if(t <= T*T_DSP/2 && board_step == 3)
        {
            return board_height;
        }
        else if(t > T*T_DSP/2 && t <= T/2)
        {
            if(board_step == 3)
                return 0.5*(height-board_height)*(1-cos(omega*new_t))+board_height;
            else
                return 0.5*height*(1-cos(omega*new_t));
        }
        else if(t > T/2 && t <= T*(1-(T_DSP/2)))
        {
            if(board_step == 1)
                return 0.5*(height-board_height)*(1-cos(omega*new_t))+board_height;
            else
                return 0.5*height*(1-cos(omega*new_t));
        }
        else if(t > T*(1-(T_DSP/2)) && board_step == 1)
        {
            return board_height;
        }
        else
        {
            return 0;
        }
    }
    else if(board_step == 2) 
    {
        return board_height;
    }
    else
    {
        return 0;
    }
}

double WalkingGaitByLIPM::wFootPositionZ(const double height, const double t, const double T, const double T_DSP)
{
    double new_T = T*(1-T_DSP); //ssp
    double new_t = t-T*T_DSP/2; //
    double omega = 2*PI/new_T;

    if(t > T*T_DSP/2 && t < T*(1-(T_DSP/2)))
        return 0.5*height*(1-cos(omega*new_t));
    else
        return 0;
}

double WalkingGaitByLIPM::wFootTheta(const double theta, bool reverse, const double t, const double T, const double T_DSP)
{
    double new_T = T*(1-T_DSP);
    double new_t = t-T*T_DSP/2;
    double omega = 2*PI/new_T;
    if(t>0 && t<=T*T_DSP/2 && !reverse)
        return 0;
    else if(t>0 && t<=T*T_DSP/2 && reverse)
        return theta;
    else if(t>T*T_DSP/2 && t<=T*(1-T_DSP/2) && !reverse)  // 0到theta
        return 0.5*theta*(1-cos(0.5*omega*(new_t)));
    else if(t>T*T_DSP/2 && t<=T*(1-T_DSP/2) && reverse)   // theta到0
        return 0.5*theta*(1-cos(0.5*omega*(new_t-new_T)));
    else if(t>T*(1-T_DSP/2) && !reverse)
        return theta;
    else if(t>T*(1-T_DSP/2) && reverse)
        return 0;    
}

double WalkingGaitByLIPM::wForceDifferenceControl(const double t, const double T, const double T_DSP)
{
   
    // FF.getZMPValue();
    double force_left_ = balance.ZMP_process->getForceLeft();
    double force_right_ = balance.ZMP_process->getForceRight();
    //FDC.setinputdata(lpx_,lpy_,rpx_,rpy_,zmp_x,zmp_y,lpt_,rpt_, force_left_, force_right_);// force_left_, force_right_
    //FDC.run();
    if (t > T * T_DSP / 2 && t < T * (1 - (T_DSP / 2))) // 單支
        return 0;
    else{ // 雙支
        // FDC.setinputdata(lpx_, lpy_, rpx_, rpy_, MPC.xt(2), MPC.yt(2), lpt_, rpt_, force_left_, force_right_);// force_left_, force_right_
        FDC.setinputdata(lpx_,lpy_,rpx_,rpy_,zmp_x,zmp_y,lpt_,rpt_, force_left_, force_right_);// force_left_, force_right_
        FDC.run();

    // //cout << "lpz_ = " << lpz_ << "  rpz_ = " << rpz_ << "\n";
    // //cout << "lpy_ = " << lpy_ << "  rpy_ = " << rpy_ << "\n";
    // //cout << "lpx_ = " << lpx_ << "  rpx_ = " << rpx_ << "\n";
    // //cout << "lpt= = " << lpt_ << "  cos(lpt) = " << cos(lpt_) << "\n";
    // cout << "zmp_x = " << px_ << "  zmp_y = " << py_ << "\n";
    // //cout << "temp_point_x = " << FDC.temp_point_x << "  temp_point_y = " << FDC.temp_point_y << "\n";
    // //cout << "temp_com_point_x = " << FDC.temp_com_point_x << "  temp_com_point_y = " << FDC.temp_com_point_y << "\n";
    // //cout << "com_pos_lx = " << FDC.com_pos_lx << "  com_pos_rx = " << FDC.com_pos_rx << "\n";
    // //cout << "com_pos_ly = " << FDC.com_pos_ly << "  com_pos_ry = " << FDC.com_pos_ry << "\n";
    // cout << "cls_lx = " << FDC.cls_lx << "  cls_ly = " << FDC.cls_ly << "\n";
    // cout << "cls_rx = " << FDC.cls_rx << "  cls_ry = " << FDC.cls_ry << "\n";
    // cout << "alpha_x = " << FDC.alpha_x << "  alpha_y = " << FDC.alpha_y << "\n";
    // cout << "alpha = " << FDC.alpha << "\n";
    // cout << "Z_ctrl = " << FDC.Z_ctrl << "\n";

        return FDC.Z_ctrl;
    }
}

double WalkingGaitByLIPM::unit_step(double x)
{
    if(x<0)
        return 0;
    else
        return 1;
}

double WalkingGaitByLIPM::sinh(double x)
{
    return (double)(exp(x)-exp(-x))/2;
}

double WalkingGaitByLIPM::cosh(double x)
{
    return (double)(exp(x)+exp(-x))/2;
}

double WalkingGaitByLIPM::WLIPM_com_X1(double L1, double K1, double t, double Tc)
{
    return L1*(1-cosh(t/Tc))+K1*(t-Tc*sinh(t/Tc));
}
double WalkingGaitByLIPM::WLIPM_com_X2(double L2,double L1, double K2, double K1, double t, double Tc, double t1)
{
    return (L2-L1)*(1-cosh((t-t1)/Tc)) + (K2-K1)*(t-t1*cosh((t-t1)/Tc)-Tc*sinh((t-t1)/Tc));
}
double WalkingGaitByLIPM::WLIPM_com_X3(double L3,double L2, double K3, double K2, double t, double Tc, double t2)
{
    return (L3-L2)*(1-cosh((t-t2)/Tc)) + (K3-K2)*(t-t2*cosh((t-t2)/Tc)-Tc*sinh((t-t2)/Tc));
}
double WalkingGaitByLIPM::DSP_X_velocity_0(double X0,double Xt, double t, double Tc, double X1T, double X2T, double X3T)
{
    return (Xt-X0*cosh(t/Tc)-X1T-X2T-X3T)/(Tc*sinh(t/Tc));
}

string WalkingGaitByLIPM::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();
    return str;
}

void WalkingGaitByLIPM::saveData()
{
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/Walking_Trajectory_"+tmp+".csv";
    strcat(path, tmp.c_str());

    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<float>>::iterator it_walk;

	for(it_walk = map_walk.begin(); it_walk != map_walk.end(); it_walk++)
	{
		savedText += it_walk->first;
		if(it_walk == --map_walk.end())
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
	it_walk = map_walk.begin();
	int max_size = it_walk->second.size();

	for(it_walk = map_walk.begin(); it_walk != map_walk.end(); it_walk++)
	{
		if(max_size < it_walk->second.size())
            max_size = it_walk->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_walk = map_walk.begin(); it_walk != map_walk.end(); it_walk++)
        {
            if(i < it_walk->second.size())
            {
                if(it_walk == --map_walk.end())
                {
                    savedText += std::to_string(it_walk->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_walk->second[i]) + ",";
                }
            }
            else
            {
                if(it_walk == --map_walk.end())
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
    for(it_walk = map_walk.begin(); it_walk != map_walk.end(); it_walk++)
        it_walk->second.clear();

    name_cont_++;
}
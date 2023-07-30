/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/

#include "include/main.h"
 

int main()
{
	
	int i=0;
	bool stop_walk = false;
	sensor.fall_Down_Flag_ = false;
	sensor.stop_Walk_Flag_ = false;

	balance.initialize(30);
	usleep(1000 * 1000);
	init.initial_system();
	usleep(1000 * 1000);
	walkinggait.if_finish_ = false;
	IK.initial_inverse_kinematic();
	walkinggait.initialize();

	//printf("initial end\n");
	cout << "initial end" << endl;
	gettimeofday(&walkinggait.timer_start_, NULL);

	int IB_count = 0; /* COM估測器 調用次數 */
	IB.setparameter(0.06,0.05);	  /* 設定CoM估測器取樣時間與截止週期 */

	/*zmp測試*/
	struct timeval zmp_start,zmp_end,use_start,use_end;
	int zmp_timer = 0,zmp_count = 0,use_timer = 0;
	bool zmp_first_time = true;
	/*-----*/
	bool feedback_angle = false;
	//------測試用延遲------//
	//usleep(500 * 1000); 	//0.5s
	//sleep(2);				//2s
	while(1)
	{ 
		/*-------------*/
		sensor.load_sensor_setting(); //balance補償([raw,pitch,com]PID,[sup,nsup]foot_offset)
		sensor.sensor_package_generate(); //建立感測器資料;回傳IMU值給IPC
		/*---讀取步態資訊---*/
		walkinggait.load_parameter();
		walkinggait.load_walkdata();
		/*-----------------*/
		/*-----------*/
		sensor.load_imu(); //獲得IMU值
		/*---壓感---*/
		sensor.load_press_left(); 
		sensor.load_press_right();
		/*----------*/
    	/*---動作串---*/
		datamodule.load_database();
		if(datamodule.motion_execute_flag_)
		{
			if(datamodule.stand_flag)
			{
				locus.set_point_by_stand();
				IK.calculate_inverse_kinematic(60);
				walkinggait.if_finish_ = false;
				datamodule.stand_flag = false;
			}
			balance.ZMP_process->resetSensor();
			datamodule.motion_execute();
			feedback_angle = true;
			cout << "do motion"<<endl;
		}
		
		/*-----------------------------------------*/
		 

		//printf(" ");
		// cout << MPC.A_tilde << endl << MPC.B_tilde << endl;
		//  cout << "-------------------------- " << endl ; 
		// usleep(500 * 1000);
		// sleep(1);
		
		/*---獲取當前步態狀態(走OR停下)---*/
		walkinggait.calculate_point_trajectory();
		/*---------------------*/

		gettimeofday(&walkinggait.timer_end_, NULL);
		walkinggait.timer_dt_ = (double)(1000000.0 * (walkinggait.timer_end_.tv_sec - walkinggait.timer_start_.tv_sec) + (walkinggait.timer_end_.tv_usec - walkinggait.timer_start_.tv_usec));

		walkinggait.balance_dt = (double)(1000000.0 * (walkinggait.timer_end_.tv_sec - walkinggait.timer_start_.tv_sec) + (walkinggait.timer_end_.tv_usec - walkinggait.timer_start_.tv_usec));


		balance.get_sensor_value();

		if (balance.two_feet_grounded_ && sensor.fall_Down_Flag_)
		{
			sensor.stop_Walk_Flag_ = true;

		}
		else
		{
			sensor.stop_Walk_Flag_ = false;
			
		}

		/*zmp測試*/
		if (zmp_first_time)
		{
			gettimeofday(&zmp_start, NULL);
			zmp_first_time = false;
		}
			gettimeofday(&zmp_end, NULL);
		
		
		zmp_timer = (double)(1000000.0 * (zmp_end.tv_sec - zmp_start.tv_sec) + (zmp_end.tv_usec - zmp_start.tv_usec));

		// if (zmp_timer>=1000000.0)//one second
		// {
		// 	balance.ZMP_process->getZMPValue();
		// 	zmp_first_time = true;
		// 	zmp_count++;
		// }

		// if (zmp_count == 30)
		// if (zmp_count == 192)
		// {
		// 	balance.ZMP_process->saveData();
		// 	// walkinggait.saveData();
		// 	zmp_count = 0;
		// }
		// cout << "count : " << zmp_count << endl;
		/*-----*/

		/*
		if(parameterinfo->complan.walking_stop){

			if(IB.first_time == 1){
			IB.saveData();
			IB.first_time = 0;
			IB.init();
			}

		}else if(IB.first_time == 0){
			
		}
		*/
		/*--------------步態------------------------*/
		if((walkinggait.timer_dt_ >= 60000.0))// && !sensor.stop_Walk_Flag_)
		{
			gettimeofday(&use_start, NULL);
			// walkinggait.setcom_pos(IB.WpB(0),IB.WpB(1));
			walkinggait.setcom_pos(IB.X(0),IB.Y(0));
			walkinggait.walking_timer();
			gettimeofday(&use_end, NULL);

			use_timer = (double)(1000000.0 * (use_end.tv_sec - use_start.tv_sec) + (use_end.tv_usec - use_start.tv_usec));
			cout << "use_time : " << use_timer << endl;

			gettimeofday(&walkinggait.timer_start_, NULL);

			
			// balance.balance_control();
			/*---馬達回授---*/
			feedbackmotor.load_motor_data_left_foot();
			feedbackmotor.load_motor_data_right_foot();
			feedbackmotor.pushData();
			read_feedback = false;
			/*-------------*/
		}
		// cout << "walk_end" << endl;
 		// printf(" ");
		// usleep(100 * 1000); 

		
		if((walkinggait.locus_flag_))
		{


			/*-----*/
			// balance.setSupportFoot();	//確認支撐腳
			// balance.endPointControl();	//末端點控制
			if(walkinggait.LIPM_flag_)
			{
				// balance.balance_control(); // 平衡控制(sensor_set)
			}
			locus.get_cpg_with_offset();  //獲取末端點
			// locus.control_by_robot_status(); //擺手&擺腰
			IK.calculate_inverse_kinematic(60);//walkinggait.motion_delay_
			locus.do_motion(); // 將目標刻度送給伺服馬達
			feedback_angle = true;

			// zmp_count++;

			/* COM估測器 測試 */
			if((walkinggait.now_step_ % 2) == 1){
				WpA_(0) = walkinggait.lpx_;
				WpA_(1) = walkinggait.lpy_;
				WpA_(2) = 0;
				BpA_(0) = walkinggait.step_point_lx_;
				BpA_(1) = walkinggait.step_point_ly_;
				BpA_(2) = -COM_HEIGHT;
				Theta_(0) = sensor.rpy_[0];
				Theta_(1) = sensor.rpy_[1];
				Theta_(2) = walkinggait.theta_;
				IB.setinputdata(WpA_,BpA_,Theta_);
			}else if((walkinggait.now_step_ % 2) == 0){
				WpA_(0) = walkinggait.rpx_;
				WpA_(1) = walkinggait.rpy_;
				WpA_(2) = 0;
				BpA_(0) = walkinggait.step_point_rx_;
				BpA_(1) = walkinggait.step_point_ry_;
				BpA_(2) = -COM_HEIGHT;
				Theta_(0) = sensor.rpy_[0];
				Theta_(1) = sensor.rpy_[1];
				Theta_(2) = walkinggait.theta_;
				IB.setinputdata(WpA_,BpA_,Theta_);
			}
			
			IB.setPrioriEstimate(MPC.xt, MPC.yt);
			IB.run();
			IB.map_com.find("desired_com_y")->second.push_back(walkinggait.py_);


			balance.ZMP_process->getZMPValue(); // ZMP_process


			if(walkinggait.if_finish_){
				IB.saveData();
				balance.ZMP_process->saveData();
				feedbackmotor.saveData();
				IB.first_time = 0;
				IB.init();
				IB.reset_Kalmen_Filter();
				walkinggait.if_finish_ = false;
				
			}else{
				// MPC.xt = IB.X;
				// MPC.yt = IB.Y;
			}

			IB_count++;
			/*-----*/
			walkinggait.LIPM_flag_ = false;
			walkinggait.locus_flag_ = false;
		}
		/*---馬達回授---*/
		feedbackmotor.load_motor_data_left_foot();
		feedbackmotor.load_motor_data_right_foot();
		if(feedback_angle)
		{
			feedbackmotor.pushData();
			feedback_angle = false;
		}
		/*-------------*/
		// if(parameterinfo->LCFinishFlag  && parameterinfo->LCBalanceOn)
		// {
		// 	i++;
		// 	if(i>290)
		// 	{
		// 		parameterinfo->LCFinishFlag = false;
		// 		parameterinfo->LCBalanceFlag = false;
		// 		balance.saveData();
		// 		IK.saveData();
		// 		i = 0;
		// 	}
		// 	else if(i>200)
		// 	{
		// 		parameterinfo->LCBalanceFlag = true;
		// 	}
		// 	if(i>90)
		// 	{
		// 		balance.setSupportFoot();
		// 		balance.balance_control();
		// 		locus.get_cpg_with_offset();
		// 		IK.calculate_inverse_kinematic(30);
		// 		locus.do_motion();
		// 	}
		// }
		// else
		// {
		// 	parameterinfo->LCFinishFlag = false;
		// }
	
	
	
	}


/*
		if(walkinggait.plot_once_ == true)
		{
			balance.saveData();
			IK.saveData();
			balance.resetControlValue();
			walkinggait.plot_once_ = false;
		}
*/	

	// clean up our memory mapping and exit
	init.Clear_Memory_Mapping();

	return( 0 );
}

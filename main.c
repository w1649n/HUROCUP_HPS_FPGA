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
	/*---系統初始化---*/
	balance.initialize(30);
	usleep(1000 * 1000);
	init.initial_system();
	usleep(1000 * 1000);
	IK.initial_inverse_kinematic();
	walkinggait.initialize();
	/*----------------*/
	/*----初始化屈膝站姿-----*/
	// locus.get_first_stand();
	// IK.calculate_inverse_kinematic(30);
	// walkinggait.if_finish_ = false;
	/*----------------------*/
	gettimeofday(&walkinggait.timer_start_, NULL);

	int IB_count = 0; /* COM估測器 調用次數 */
	IB.setparameter(0.03,0.05);	  /* 設定CoM估測器取樣時間與截止週期 */

	/*zmp測試*/
	struct timeval zmp_start,zmp_end;
	int zmp_timer = 0,zmp_count = 0;
	bool zmp_first_time = true;
	/*-----*/
	bool read_feedback = false;
	//------測試用延遲------//
	//usleep(500 * 1000); 	//0.5s
	//sleep(2);				//2s
	while(1)
	{  
		/*---動作串---*/
		datamodule.load_database();
		if(datamodule.motion_execute_flag_)
		{
			if(datamodule.stand_flag)
			{
				IK.initial_inverse_kinematic();
				walkinggait.final_step();
				locus.get_first_stand();
				IK.calculate_inverse_kinematic_forstand(30);
				walkinggait.if_finish_ = false;
				datamodule.stand_flag = false;
			}
			datamodule.motion_execute();
			read_feedback = true;
		}
		/*-----------*/
		sensor.load_imu(); //獲得IMU值
		/*---壓感---*/
		sensor.load_press_left(); 
		sensor.load_press_right();
		/*----------*/
		/*-----------------------------------------*/
		/*---馬達回授---*/
		// feedbackmotor.load_motor_data_left_foot();
		// feedbackmotor.load_motor_data_right_foot();
		// feedbackmotor.pushData();
		// read_feedback = false;
		/*-------------*/
		sensor.load_sensor_setting(); //balance補償([raw,pitch,com]PID,[sup,nsup]foot_offset)
		sensor.sensor_package_generate(); //建立感測器資料;回傳IMU值給IPC
		/*---讀取步態資訊---*/
		walkinggait.load_parameter();
		walkinggait.load_walkdata();
		/*-----------------*/
		/*---獲取當前步態狀態(走OR停下)---*/
		walkinggait.calculate_point_trajectory();
		/*---------------------*/


		gettimeofday(&walkinggait.timer_end_, NULL);
		walkinggait.timer_dt_ = (double)(1000000.0 * (walkinggait.timer_end_.tv_sec - walkinggait.timer_start_.tv_sec) + (walkinggait.timer_end_.tv_usec - walkinggait.timer_start_.tv_usec));

		balance.get_sensor_value();
		/*zmp測試*/
		// if (zmp_first_time)
		// {
		// 	gettimeofday(&zmp_start, NULL);
		// 	zmp_first_time = false;
		// }
		// 	gettimeofday(&zmp_end, NULL);
		
		
		// zmp_timer = (double)(1000000.0 * (zmp_end.tv_sec - zmp_start.tv_sec) + (zmp_end.tv_usec - zmp_start.tv_usec));

		// if (zmp_timer>=1000000.0)//one second
		// {
		// 	balance.ZMP_process->getZMPValue();
		// 	zmp_first_time = true;
		// 	zmp_count++;
		// }

		// if (zmp_count == 30)
		// {
		// 	//balance.ZMP_process->saveData();
		// 	zmp_count = 0;
		// }
		
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
		if((walkinggait.timer_dt_ >= 15000.0))// && !sensor.stop_Walk_Flag_)
		{
			walkinggait.setcom_pos(IB.WpB(0),IB.WpB(1));
			walkinggait.walking_timer();
			walkinggait.pushData();

			gettimeofday(&walkinggait.timer_start_, NULL);
			// balance.balance_control();
		}
		

 		// printf(" ");
		// usleep(100 * 1000); 
		if((walkinggait.locus_flag_))
		{
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
			IB.run();
			IB.map_com.find("desired_com_y")->second.push_back(walkinggait.py_);

			if(walkinggait.if_finish_){
				IB.saveData();
				IB.first_time = 0;
				IB.init();
				
				//walkinggait.if_finish_ = false;
			}

			IB_count++;
			//printf("walking ");
			/*-----*/
			balance.setSupportFoot();	//確認支撐腳
			balance.endPointControl();	//末端點控制
			if(walkinggait.LIPM_flag_)	
			{
				balance.balance_control();	// 平衡控制(sensor_set)
			}
			locus.get_cpg_with_offset();  //獲取末端點
			locus.control_by_robot_status(); //擺手&擺腰
			IK.calculate_inverse_kinematic(walkinggait.motion_delay_); //計算逆運動學
			IK.pushData();
			
			locus.do_motion();

			walkinggait.LIPM_flag_ = false;
			walkinggait.if_finish_ = false;
			walkinggait.locus_flag_ = false;
			read_feedback = true;
		}
		 
	}
	// clean up our memory mapping and exit
	init.Clear_Memory_Mapping();

	return( 0 );
}

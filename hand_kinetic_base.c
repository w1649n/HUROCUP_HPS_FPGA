#include "hand_kinetic_base.h"


struct Points_Struct result_package;

extern Initial init;
extern Datamodule datamodule;
extern Locus locus;

Hand_InverseKinematic::Hand_InverseKinematic()
{

}

Hand_InverseKinematic::~Hand_InverseKinematic()
{

}

void Hand_InverseKinematic::Kinetic_Main(double rp_x, double rp_y, double rp_z)
{
	recevie_point_.x = rp_x;
	recevie_point_.y = rp_y;
	recevie_point_.z = rp_z;
	//printf("receive x = %f ,receive y = %f ,receive z = %f \n", rp_x, rp_x, recevie_point_.z);
	Kinetic_Inverse();

}

void Hand_InverseKinematic::Kinetic_Inverse()
{
	printf("receive x = %f ,receive y = %f ,receive z = %f \n", recevie_point_.x, recevie_point_.y, recevie_point_.z);
	double link_[4] = { 37, 90, 0, 90 };
	double cos_d;
	init_.dof3 = 0;
	if (recevie_point_.x > 0)
	{
		init_.dof1 = atan2(recevie_point_.x, -recevie_point_.z);
	}
	else if (recevie_point_.x < 0)
	{
		init_.dof1 = -atan2(-recevie_point_.x, -recevie_point_.z);
	}
	else if (recevie_point_.x == 0)
	{
		init_.dof1 = -atan2(recevie_point_.x, -recevie_point_.z);
	}

	cos_d = ((pow(link_[1],2) + pow(link_[3],2)) - ((pow(recevie_point_.x,2) + pow(recevie_point_.z,2)) + pow(recevie_point_.y - link_[0],2))) / (2 * link_[1] * link_[3]);
	if ((1 - pow(cos_d, 2) < 0))
	{
		init_.dof4 = atan2(0, -cos_d);
	}
	else
	{
		init_.dof4 = atan2(sqrt(1 - pow(cos_d, 2)), -cos_d);
	}
	// init_.dof4 = atan2(sqrt(1 - pow(cos_d, 2)), -cos_d);
	init_.dof2 = -(atan2((recevie_point_.y - link_[0]), sqrt(pow(recevie_point_.x, 2) + pow(recevie_point_.z, 2))) - atan2(link_[3] * sin(init_.dof4), link_[1] + link_[3] * cos(init_.dof4)));

	//cout << "dof1 = " << init_.dof1*(180 / PI) << endl << "dof2 = " << init_.dof2*(180 / PI) << endl << "dof3 = " << init_.dof3*(180 / PI) << endl << "dof4 = " << init_.dof4*(180 / PI) << endl;

	// allxyz_detect-------------------------------------------------------------------------------------------------

	struct four_dof angletest;
	angletest = init_;
	Kinetic_Forward(angletest.dof1, angletest.dof2, angletest.dof3, angletest.dof4);

	double tempsum = abs(recevie_point_.x - confirm_point_.x) + abs(recevie_point_.y - confirm_point_.y) + abs(recevie_point_.z - confirm_point_.z);

	double sum = tempsum;

	final_point_.x = confirm_point_.x;
	final_point_.y = confirm_point_.y;
	final_point_.z = confirm_point_.z;

	//postive & negative test

	//+-

	Kinetic_Forward(angletest.dof1, angletest.dof2, angletest.dof3, -angletest.dof4);

	tempsum = abs(recevie_point_.x - confirm_point_.x) + abs(recevie_point_.y - confirm_point_.y) + abs(recevie_point_.z - confirm_point_.z);

	if (tempsum < sum)
	{
		sum = tempsum;
		init_.dof2 = angletest.dof2;
		init_.dof4 = -angletest.dof4;
		
		final_point_.x = confirm_point_.x;
		final_point_.y = confirm_point_.y;
		final_point_.z = confirm_point_.z;
	}

	//--

	Kinetic_Forward(angletest.dof1, -angletest.dof2, angletest.dof3, -angletest.dof4);

	tempsum = abs(recevie_point_.x - confirm_point_.x) + abs(recevie_point_.y - confirm_point_.y) + abs(recevie_point_.z - confirm_point_.z);

	if (tempsum < sum)
	{
		sum = tempsum;
		init_.dof2 = -angletest.dof2;
		init_.dof4 = -angletest.dof4;
	
		final_point_.x = confirm_point_.x;
		final_point_.y = confirm_point_.y;
		final_point_.z = confirm_point_.z;
	}

	//-+

	Kinetic_Forward(angletest.dof1, -angletest.dof2, angletest.dof3, angletest.dof4);

	tempsum = abs(recevie_point_.x - confirm_point_.x) + abs(recevie_point_.y - confirm_point_.y) + abs(recevie_point_.z - confirm_point_.z);

	if (tempsum < sum)
	{
		sum = tempsum;
		init_.dof2 = -angletest.dof2;
		init_.dof4 = angletest.dof4;
	
		final_point_.x = confirm_point_.x;
		final_point_.y = confirm_point_.y;
		final_point_.z = confirm_point_.z;
	}

	transmit_dof_.dof1 = init_.dof1;//弧度 ; 角度需乘*(180 / PI)
	transmit_dof_.dof2 = init_.dof2;
	transmit_dof_.dof3 = init_.dof3;
	transmit_dof_.dof4 = init_.dof4;

	

	printf("dof 1 = %f ,dof 2 = %f ,dof 3 = %f ,dof 4 = %f \n", (transmit_dof_.dof1*(180 / PI)), (transmit_dof_.dof2*(180 / PI)), (transmit_dof_.dof3*(180 / PI)), (transmit_dof_.dof4*(180 / PI)));
	printf("final x = %f ,final y = %f ,final z = %f \n", final_point_.x, final_point_.y, final_point_.z);

	if ((transmit_dof_.dof1*(180 / PI)) >= 150 || (transmit_dof_.dof1*(180 / PI))  <= -150 )
	{
		cout<<"dof1 over range"<<endl;
		transmit_dof_.dof1 = 0;
		transmit_dof_.dof2 = 0;
		transmit_dof_.dof3 = 0;
		transmit_dof_.dof4 = 0;
	}

	if ((transmit_dof_.dof2*(180 / PI)) >= 20 || (transmit_dof_.dof2*(180 / PI)) <= -90 )
	{
		cout << "dof2 over range" << endl;
		transmit_dof_.dof1 = 0;
		transmit_dof_.dof2 = 0;
		transmit_dof_.dof3 = 0;
		transmit_dof_.dof4 = 0;
	}

	if ((transmit_dof_.dof4*(180 / PI)) >= 135 || (transmit_dof_.dof4*(180 / PI)) <= -90 )
	{
		cout << "dof4 over range" << endl;
		transmit_dof_.dof1 = 0;
		transmit_dof_.dof2 = 0;
		transmit_dof_.dof3 = 0;
		transmit_dof_.dof4 = 0;
	}



	printf("final dof 1 = %f ,final dof 2 = %f ,final dof 3 = %f ,final dof 4 = %f \n", (transmit_dof_.dof1*(180 / PI)), (transmit_dof_.dof2*(180 / PI)), (transmit_dof_.dof3*(180 / PI)), (transmit_dof_.dof4*(180 / PI)));
	
	

	hand_package();

	if(hand_do_motion_)
	{
		locus.do_motion();
		hand_do_motion_ = false;
	}

}

void Hand_InverseKinematic::Kinetic_Forward(double dof1, double dof2, double dof3, double dof4)
{

	double link_[4] = { 37, 90, 0, 90 };
	double hand_angle[4] = { 0, 0, 0, 0 };
	hand_angle[0] = dof1;
	hand_angle[1] = dof2;
	hand_angle[2] = dof3;
	hand_angle[3] = dof4;

	Float T[16] = {1, 0, 0, 0,
				   0, 1, 0, 0,
				   0, 0, 1, 0,
				   0, 0, 0, 1};
	Float dh[16] = { 0       , PI/2, link_[0] , PI/2,
					 0       , PI/2, 0       , PI/2,
					 0       , PI/2, -link_[1], PI,
					 -link_[3], 0   , 0       , PI/2 };

	fMatrix muilt_base(T, 4, 4);
	fMatrix dh_list(dh, 4, 4);


	fMatrix all_pos(4, 4);
	//all_pos = Identity(4);
	//Float *A_base;

	fVector temparray(4);
	


	for (int i = 0; i < 4; i++)
	{
		
		fVector dhtemp = dh_list.GetRow(i);
		//A_base = GenerateTransformationMatrices(hand_angle[i], dhtemp);

		double c_theta = cos(hand_angle[i] + num2double(dhtemp, 3));
		double s_theta = sin(hand_angle[i] + num2double(dhtemp, 3));
		double c_alpha = cos(num2double(dhtemp, 1));
		double s_alpha = sin(num2double(dhtemp, 1));

		Float A_base[16] = { c_theta, -s_theta*c_alpha, s_theta*s_alpha, num2double(dhtemp, 0)*c_theta,
							 s_theta, c_theta*c_alpha, -c_theta*s_alpha, num2double(dhtemp, 0)*s_theta,
							 0, s_alpha, c_alpha, num2double(dhtemp, 2),
							 0, 0, 0, 1 };

		fMatrix A(A_base, 4, 4);

		muilt_base = muilt_base*A;

		temparray = muilt_base.GetCol(3);

		all_pos.SetRow(i, temparray);

	}

	fVector postemp = all_pos.GetRow(3);

	confirm_point_.x = num2double(postemp, 0);
	confirm_point_.y = num2double(postemp, 2);
	confirm_point_.z = num2double(postemp, 1);


}

void Hand_InverseKinematic::package_init()
{
	//printf("in init!!!!\n");
	//Locus_time = 30;
    //600 / 24 = 25
	for(int i = 0; i < 9; i++)  
		result_package.Thta[i] = PI_2;
	//給各角度正負
	result_package.P_Table[0] = 0;                     //Positive
	result_package.P_Table[1] = 0;                     //Positive
	result_package.P_Table[2] = 0;                     //Positive
	result_package.P_Table[3] = 0;                     //Positive
	result_package.P_Table[4] = 0;                     //Positive
	result_package.P_Table[5] = 0;                     //Positive
	result_package.P_Table[6] = 0;                     //Positive
	result_package.P_Table[7] = 0;                     //Positive
	result_package.P_Table[8] = 0;                     //Positive
	result_package.P_Table[9] = 0;                     //Positive
	result_package.P_Table[10] = 0;                    //Positive
	result_package.P_Table[11] = 0;                    //Positive
	result_package.P_Table[12] = 0;                    //Positive
	result_package.P_Table[13] = 1;                    //Negitive
	result_package.P_Table[14] = 1;                    //Negitive
	result_package.P_Table[15] = 0;                    //Positive
	result_package.P_Table[16] = 0;                    //Pogitive
	result_package.P_Table[17] = 1;                    //Negitive
	result_package.P_Table[18] = 1;                    //Negitive
	result_package.P_Table[19] = 0;                    //Positive
	result_package.P_Table[20] = 1;                    //Negitive
	// printf("\nR_X: %f, R_Y: %f, R_Z: %f, R_T: %f\nL_X: %f, L_Y: %f, L_Z: %f, L_T: %f\n\n",result_package.Inverse_PointR_X, result_package.Inverse_PointR_Y, result_package.Inverse_PointR_Z, result_package.Inverse_PiontR_Thta, result_package.Inverse_PointL_X, result_package.Inverse_PointL_Y, result_package.Inverse_PointL_Z, result_package.Inverse_PiontL_Thta);


	for(int k = 0 ; k<21 ; k++)
	{
			thta_base_[k] = datamodule.totalangle_[k];
	}

	int j;
	for(j = 0; j < 21; j++)//===============================================i=0>i=9
	{   
		if(result_package.P_Table[j])
		{
			angle_[j] = Max_value - (result_package.Thta[j] * PI_TO_OUTPUT + Position_Zero);
		}
		else
		{
			angle_[j] = result_package.Thta[j] * PI_TO_OUTPUT + Position_Zero;
		}
		
		output_base_[j] = thta_base_[j] - angle_[j];
		output_angle_[j] = thta_base_[j];
		output_speed_[j] = 100;
		past_thta_[j] = result_package.Thta[j];
		result_package.UThta[j] = result_package.Thta[j];
		delay_time_[j] = 256;
		past_delay_time_[j] = 256;
	}

	//for(j = 0;j<4;j++)printf("angle %d = %d   ;; ",j,angle_[j]);

}

void Hand_InverseKinematic::hand_package()
{
	package_init();
	int Motion_Delay = 30;
	int i;
	// for(i = 0; i < 21; i++)
	// {
	// 	thta_base_[i] = datamodule.totalangle_[i];
	// }

	result_package.Thta[0] = ((-1)*transmit_dof_.dof1)+PI_2;
	result_package.Thta[1] = transmit_dof_.dof2+PI_2;
	result_package.Thta[2] = transmit_dof_.dof3+PI_2;
	result_package.Thta[3] = ((-1)*transmit_dof_.dof4)+PI_2; 

	//printf("se 1 = %d ,se 2 = %d ,se 3 = %d ,base 4 = %d \n", thta_base_[0], thta_base_[1], thta_base_[2],thta_base_[3]);
	//printf("base 1 = %f ,base 2 = %f ,base 3 = %f ,base 4 = %f \n", result_package.Thta[0], result_package.Thta[1], result_package.Thta[2], result_package.Thta[3]);
	// result_package.Thta[4] = 1024;
	// result_package.Thta[5] = 2198;
	// result_package.Thta[6] = 2048;
	// result_package.Thta[7] = 2048;
	// result_package.Thta[8] = 2048;
	// result_package.Thta[9] = 2048;
	// result_package.Thta[10] = 2038;
	// result_package.Thta[11] = 1580;
	// result_package.Thta[12] = 2778;
	// result_package.Thta[13] = 2385;
	// result_package.Thta[14] = 2065;
	// result_package.Thta[15] = 2058;
	// result_package.Thta[16] = 2058;
	// result_package.Thta[17] = 2500;
	// result_package.Thta[18] = 1295;
	// result_package.Thta[19] = 1675;
	// result_package.Thta[20] = 2048;


	for(int k = 0 ; k<4 ; k++)//給各角度正負
	{
		if(result_package.Thta[k]<0)
		{
			result_package.P_Table[k] = 1;
			// result_package.Thta[k] = abs(result_package.Thta[k]);
		}
		else
		{
			result_package.P_Table[k] = 0;
		}
		
	}


	for(i = 0; i < 21; i++)
    {
        // if(result_package.P_Table[i])
        // {
        //     output_angle_[i] = (unsigned int)(Max_value - (result_package.Thta[i] * PI_TO_OUTPUT + Position_Zero));
        // }
        // else
        // {
            output_angle_[i] = (unsigned int)(result_package.Thta[i] * PI_TO_OUTPUT + Position_Zero);
        // }
        output_angle_[i] += output_base_[i];

        double different_thta;
        different_thta = fabs( past_thta_[i] - result_package.Thta[i]);
        if(different_thta > 0.0)
        {
        	delay_time_[i] = (unsigned int)(different_thta/(2*PI) * (1000/Motion_Delay) * 60 / 0.229);	// ((percent of circle(rad)) / ((delta t/1000)*60(min))) / 0.229(rpm/unit)
        }
        past_thta_[i] = result_package.Thta[i];
        output_speed_[i] = delay_time_[i]  * SPEED_TRANS;
        output_speed_[i] = output_speed_[i] * 1;
		//----------------------printf-----------------------------
        #ifdef Auto_Stand
            if(i == 10 || i == 11 || i == 12 || i == 13 || i == 14   || i == 16 || i == 17 || i == 18 || i == 19)
                printf("%d:%d ", i+1, output_angle_[i]);
            else if(i == 20)
                printf("%d:%d\n", i+1, output_angle_[i]);
        #endif
        //---------------------------------------------------------
        if(output_angle_[i] >Max_value)
        {
            output_angle_[i] = Max_value;
        }
        else if(output_angle_[i] <= 0)
        {
            output_angle_[i] = 0;
        }
        if (output_speed_[i] <= 0)
        {
            output_speed_[i] = 0;
        }
        else if(output_speed_[i] > 32767)
        {
            output_speed_[i] = 32767;
		}

		output_angle_[4] = 1024;
		output_angle_[5] = 2198;
		output_angle_[6] = 2048;
		output_angle_[7] = 2048;
		output_angle_[8] = 2048;
		output_angle_[9] = 2048;
		output_angle_[10] = 2038;
		output_angle_[11] = 1580;
		output_angle_[12] = 2778;
		output_angle_[13] = 2385;
		output_angle_[14] = 2065;
		output_angle_[15] = 2058;
		output_angle_[16] = 2058;
		output_angle_[17] = 2500;
		output_angle_[18] = 1295;
		output_angle_[19] = 1675;
		output_angle_[20] = 2048;

		output_speed_[0] = 50;
		output_speed_[1] = 50;
		output_speed_[2] = 50;
		output_speed_[3] = 50;

		*((uint32_t *)init.robot_motion_addr+(2*i+1)) = output_speed_[i];
		*((uint32_t *)init.robot_motion_addr+(2*i)) = output_angle_[i];
		// printf("32\n");
    }
	// printf("\n");
	*((uint32_t *)init.robot_motion_addr+(42)) = Motion_Delay;
	*((uint32_t *)init.robot_motion_addr+(43)) = 0x00000070;

	for(int h = 0 ; h < 21 ; h++)
	{
		printf("package = %d\n",output_angle_[h]);
	}

    unsigned short blk_size = 0;
	// Header
	hand_char_[0] = 0xFF;
	hand_char_[1] = 0xFF;
	hand_char_[2] = 0xFD;
	// Reserved
	hand_char_[3] = 0x00;
	// ID
	hand_char_[4] = 0xFE;
	// Length      The length after the Packet Length field (Instruction, Parameter, CRC fields). Packet Length = number of Parameters + 3
	hand_char_[5] = 0x2b;       //0x2b = 43(decimal)
	hand_char_[6] = 0;
	// Instruction
	hand_char_[7] = 0x83;       //write         // 0x83 = sync write
	// Parameter
	hand_char_[8] = 0x70;       // addressL             // Velocity: 0x70 = 112 Position: 0x74(hex) = 116(dec)
	hand_char_[9] = 0;

	hand_char_[10] = 0x08;      // data length(byte)
	hand_char_[11] = 0x00;


	//left hand
	for(i=0; i<4; i++)
	{
		hand_char_[12+i*9] = i+1;      //MotorID
		hand_char_[13+i*9] = output_speed_[i] & 0xFF;    //Profile Velocity      //initial value = 0
		hand_char_[14+i*9] = (output_speed_[i] >> 8) & 0xFF;
		hand_char_[15+i*9] = (output_speed_[i] >> 16) & 0xFF;
		hand_char_[16+i*9] = (output_speed_[i] >> 24) & 0xFF;
		hand_char_[17+i*9] = output_angle_[i] & 0xFF;    // positionL
		hand_char_[18+i*9] = (output_angle_[i] >> 8) & 0xFF;
		hand_char_[19+i*9] = (output_angle_[i] >> 16) & 0xFF;
		hand_char_[20+i*9] = (output_angle_[i] >> 24) & 0xFF;
	}

	blk_size = 5 + hand_char_[5];
	unsigned short lh_crc_value = update_crc(0, hand_char_, blk_size);

	//right hand
	for(i=0; i<4; i++)
	{
		hand_char_[12+i*9] = i+5;      //MotorID
		hand_char_[13+i*9] = output_speed_[i+4] & 0xFF;    //Profile Velocity      //initial value = 0
		hand_char_[14+i*9] = (output_speed_[i+4] >> 8) & 0xFF;
		hand_char_[15+i*9] = (output_speed_[i+4] >> 16) & 0xFF;
		hand_char_[16+i*9] = (output_speed_[i+4] >> 24) & 0xFF;
		hand_char_[17+i*9] = output_angle_[i+4] & 0xFF;    // positionL
		hand_char_[18+i*9] = (output_angle_[i+4] >> 8) & 0xFF;
		hand_char_[19+i*9] = (output_angle_[i+4] >> 16) & 0xFF;
		hand_char_[20+i*9] = (output_angle_[i+4] >> 24) & 0xFF;
	}

	blk_size = 5 + hand_char_[5];
	unsigned short rh_crc_value = update_crc(0, hand_char_, blk_size);

	//foot
	hand_char_[5] = 0x46; // 0x46 = 70  77-7

	hand_char_[12] = 0x09;
	hand_char_[13] = output_speed_[8] & 0xFF;
	hand_char_[14] = (output_speed_[8] >> 8) & 0xFF;
	hand_char_[15] = (output_speed_[8] >> 16) & 0xFF;
	hand_char_[16] = (output_speed_[8] >> 24) & 0xFF;
	hand_char_[17] = output_angle_[8] & 0xFF;
	hand_char_[18] = (output_angle_[8] >> 8) & 0xFF;
	hand_char_[19] = (output_angle_[8] >> 16) & 0xFF;
	hand_char_[20] = (output_angle_[8] >> 24) & 0xFF;

	//left foot
	for(i=0; i<6; i++)
	{
		hand_char_[21+i*9] = i+10;      //MotorID
		hand_char_[22+i*9] = output_speed_[i+9] & 0xFF;    //Profile Velocity      //initial value = 0
		hand_char_[23+i*9] = (output_speed_[i+9] >> 8) & 0xFF;
		hand_char_[24+i*9] = (output_speed_[i+9] >> 16) & 0xFF;
		hand_char_[25+i*9] = (output_speed_[i+9] >> 24) & 0xFF;
		hand_char_[26+i*9] = output_angle_[i+9] & 0xFF;    // positionL
		hand_char_[27+i*9] = (output_angle_[i+9] >> 8) & 0xFF;
		hand_char_[28+i*9] = (output_angle_[i+9] >> 16) & 0xFF;
		hand_char_[29+i*9] = (output_angle_[i+9] >> 24) & 0xFF;
	}

	blk_size = 5 + hand_char_[5];
	unsigned short lf_crc_value = update_crc(0, hand_char_, blk_size);

	//right foot
	for(i=0; i<6; i++)
	{
		hand_char_[21+i*9] = i+16;      //MotorID
		hand_char_[22+i*9] = output_speed_[i+15] & 0xFF;    //Profile Velocity      //initial value = 0
		hand_char_[23+i*9] = (output_speed_[i+15] >> 8) & 0xFF;
		hand_char_[24+i*9] = (output_speed_[i+15] >> 16) & 0xFF;
		hand_char_[25+i*9] = (output_speed_[i+15] >> 24) & 0xFF;
		hand_char_[26+i*9] = output_angle_[i+15] & 0xFF;    // positionL
		hand_char_[27+i*9] = (output_angle_[i+15] >> 8) & 0xFF;
		hand_char_[28+i*9] = (output_angle_[i+15] >> 16) & 0xFF;
		hand_char_[29+i*9] = (output_angle_[i+15] >> 24) & 0xFF;
	}

	blk_size = 5 + hand_char_[5];
	unsigned short rf_crc_value = update_crc(0, hand_char_, blk_size);

	*((uint32_t *)init.robot_motion_addr+(44)) = (lh_crc_value << 16) + rh_crc_value;
	*((uint32_t *)init.robot_motion_addr+(45)) = (lf_crc_value << 16) + rf_crc_value;
}


/*double *Hand_InverseKinematic::GenerateTransformationMatrices(double kinetic_angle, fVector dh)
{
	double c_theta = cos(kinetic_angle + num2double(dh, 3));
	double s_theta = sin(kinetic_angle + num2double(dh, 3));
	double c_alpha = cos(num2double(dh, 1));
	double s_alpha = sin(num2double(dh, 1));

	Float A_base[16] = { c_theta, -s_theta*c_alpha, s_theta*s_alpha , num2double(dh, 0)*c_theta,
				         s_theta, c_theta*c_alpha , -c_theta*s_alpha, num2double(dh, 0)*s_theta,
				         0      , s_alpha         , c_alpha         , num2double(dh, 2)        ,
				         0      , 0               , 0               , 1                          };


	return &A_base[0];





}*/

unsigned short Hand_InverseKinematic::update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for (j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

void Hand_InverseKinematic::load_hand_data()
{
    int state = 0;
    int count = 0;

    for(;;)
    {
        if(state == 0)
        {
            update_hand_data_flag_ = false;
            if(*(uint32_t *)init.p2h_set_hps_read_hand_data_addr)
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
            if(count <= 1)
            {
                hand_data_[count] = *(uint32_t *)init.p2h_hand_data_addr;
                count++;
                *(uint32_t *)init.h2p_read_hand_data_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_hand_data_pulse_addr = 0;
                continue;
            }
            else
            {
                update_hand_data_flag_ = true;
                state = 0;

				hand_do_motion_ = true;

                break;
            }
        }
    }
    update_hand_data();
}

void Hand_InverseKinematic::update_hand_data()
{
    if(update_hand_data_flag_)
    {
        int count = 0;
        short tmp_point = 0;


        tmp_point = (hand_data_[count] >> 16) & 0xFFFF;
		if(tmp_point&0x8000)
		{
			hand_point_[0] = (double)((tmp_point & 0x7FFF) * (-1) / 100.0);
		}
		else
		{
			hand_point_[0] = (double)((tmp_point & 0xFFFF) / 100.0);
		}       
        tmp_point = hand_data_[count++] & 0xFFFF;
		if(tmp_point&0x8000)
		{
			hand_point_[1] = (double)((tmp_point & 0x7FFF) * (-1) / 100.0);
		}
		else
		{
			hand_point_[1] = (double)((tmp_point & 0xFFFF) / 100.0);
		}       

        tmp_point = (hand_data_[count] >> 16) & 0xFFFF;
		if(tmp_point&0x8000)
		{
			hand_point_[2] = (double)((tmp_point & 0x7FFF) * (-1) / 100.0);
		}
		else
		{
			hand_point_[2] = (double)((tmp_point & 0xFFFF) / 100.0);
		}

		tmp_point = hand_data_[count] & 0xFFFF;


		printf("x = %lf y = %lf z = %lf\n",hand_point_[0],hand_point_[1],hand_point_[2]);
    }
 


}
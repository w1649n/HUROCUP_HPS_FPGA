#include "include/DataModule.h"

extern Locus locus;
extern InverseKinematic IK;
extern Initial init;

Datamodule::Datamodule()
{
	datamodule_cmd_ = 0;
		std::vector<double> temp;
		map_motor["motor_01"] = temp;
        map_motor["motor_02"] = temp;
		map_motor["motor_03"] = temp;
        map_motor["motor_04"] = temp;
		map_motor["motor_05"] = temp;
        map_motor["motor_06"] = temp;
		map_motor["motor_07"] = temp;
        map_motor["motor_08"] = temp;
		map_motor["motor_09"] = temp;
        map_motor["motor_10"] = temp;
		map_motor["motor_11"] = temp;
        map_motor["motor_12"] = temp;
		map_motor["motor_13"] = temp;
        map_motor["motor_14"] = temp;
		map_motor["motor_15"] = temp;
        map_motor["motor_16"] = temp;
		map_motor["motor_17"] = temp;
        map_motor["motor_18"] = temp;
		map_motor["motor_19"] = temp;
        map_motor["motor_20"] = temp;
		map_motor["motor_21"] = temp;

}

Datamodule::~Datamodule()
{

}

void Datamodule::load_database()
{
	int state = 0;
	int count = 0;

	for(;;)
	{
		if(state == 0)
		{
			update_database_flag_ = false;
			if(*(uint32_t *)init.p2h_set_hps_read_database_addr)
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
			if(count <= 20)
			{
				database_[count] = *(uint32_t *)init.p2h_database_addr;
				count++;
				*(uint32_t *)init.h2p_read_database_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_database_pulse_addr = 0;
				continue;
			}
			else
			{
				update_database_flag_ = true;
				motion_execute_flag_ = true;
				state = 0;
				break;
			}
		}
	}
	update_database();
}

void Datamodule::update_database()
{
	if(update_database_flag_)
	{
		int i;
		short tmp_angle = 0;
		datamodule_cmd_ = *(uint32_t *)init.p2h_pc_command_addr;

		for(i=0; i<21; i++)
		{
			switch(datamodule_cmd_)
			{
			case 2: //29儲存全直腳
				stand_flag = true;
				if(i<9) 
				{
					//將全直腳刻度存進Calculate(12~21)
					Calculate_standspeed[11-i] = (database_[i] & 0x0000FFFF);
					Calculate_standangle[11-i] = ((database_[i] & 0xFFFF0000)>> 16);
					//--------------------------
					//手部絕對刻度
					totalspeed_[i] = (database_[20-i] & 0x0000FFFF);
					totalangle_[i] = ((database_[20-i] & 0xFFFF0000)>> 16);
				}
				else
				{
					//將全直腳刻度存進Calculate(10、11)
					if(i<12)
					{
						Calculate_standspeed[11-i] = (database_[i] & 0x0000FFFF);
						Calculate_standangle[11-i] = ((database_[i] & 0xFFFF0000)>> 16);
					}
					//IK算好的腿部刻度
					totalspeed_[i] = Walking_standspeed[i];
					totalangle_[i] = Walking_standangle[i];
				}
				break;
			case 3: //motion做動作
				totalspeed_[20-i] = (database_[i] & 0x0000FFFF);
				tmp_angle = ((database_[i] & 0xFFFF0000) >> 16);

				if(tmp_angle & 0x8000)
				{
					if(i<12)//紀錄相對刻度用於計算
						Calculate_standangle[11-i] -= (tmp_angle & 0x7FFF);
					totalangle_[20-i] = totalangle_[20-i] - (tmp_angle & 0x7FFF);
					if(totalangle_[20-i] < 0)
						totalangle_[20-i] = 0;
				}
				else
				{
					if(i<12)//紀錄相對刻度用於計算
						Calculate_standangle[11-i] += (tmp_angle & 0x7FFF);
					totalangle_[20-i] = totalangle_[20-i] + (tmp_angle & 0x7fff);
					if(totalangle_[20-i] > 4095)
						totalangle_[20-i] = 4095;
				}
				break;
			case 4:	//測試全直腳(使用motion上的Lockstand)
				totalspeed_[20-i] = (database_[i] & 0x0000FFFF);
				totalangle_[20-i] = ((database_[i] & 0xFFFF0000)>> 16);
				break;
			}
		}
	}
}
void Datamodule::pushData()
{
	map_motor.find("motor_01")->second.push_back((double)totalangle_[0]);
	map_motor.find("motor_02")->second.push_back((double)totalangle_[1]);
	map_motor.find("motor_03")->second.push_back((double)totalangle_[2]);
	map_motor.find("motor_04")->second.push_back((double)totalangle_[3]);
	map_motor.find("motor_05")->second.push_back((double)totalangle_[4]);
	map_motor.find("motor_06")->second.push_back((double)totalangle_[5]);
	map_motor.find("motor_07")->second.push_back((double)totalangle_[6]);
	map_motor.find("motor_08")->second.push_back((double)totalangle_[7]);
	map_motor.find("motor_09")->second.push_back((double)totalangle_[8]);
	map_motor.find("motor_10")->second.push_back((double)totalangle_[9]);
	map_motor.find("motor_11")->second.push_back((double)totalangle_[10]);
	map_motor.find("motor_12")->second.push_back((double)totalangle_[11]);
	map_motor.find("motor_13")->second.push_back((double)totalangle_[12]);
	map_motor.find("motor_14")->second.push_back((double)totalangle_[13]);
	map_motor.find("motor_15")->second.push_back((double)totalangle_[14]);
	map_motor.find("motor_16")->second.push_back((double)totalangle_[15]);
	map_motor.find("motor_17")->second.push_back((double)totalangle_[16]);
	map_motor.find("motor_18")->second.push_back((double)totalangle_[17]);
	map_motor.find("motor_19")->second.push_back((double)totalangle_[18]);
	map_motor.find("motor_20")->second.push_back((double)totalangle_[19]);
	map_motor.find("motor_21")->second.push_back((double)totalangle_[20]);
}
void Datamodule::motion_execute()	//做motion動作
{
	//pushData();
	// saveData();
	// printf("motion exe\n");
	int i=0;
	unsigned short blk_size = 0;

	IK.packet_char_[0] = 0xff;//Header
	IK.packet_char_[1] = 0xff;//Header
	IK.packet_char_[2] = 0xfd;//Header
	IK.packet_char_[3] = 0x00;//Reserved
	IK.packet_char_[4] = 0xfe;//Packet ID
	IK.packet_char_[5] = 0x2B;//Packet Length L  =  201
	IK.packet_char_[6] = 0x00;//Packet Length H
	IK.packet_char_[7] = 0x83;//Instruction

	IK.packet_char_[8] = 0x70;//Address L
	IK.packet_char_[9] = 0x00;//Address H

	IK.packet_char_[10] = 0x08;//Data Length L
	IK.packet_char_[11] = 0x00;//Data Length H

	for(i=0; i<21; i++)
	{
		*((uint32_t *)init.robot_motion_addr+(2*i+1)) = totalspeed_[i];
		*((uint32_t *)init.robot_motion_addr+(2*i)) = totalangle_[i];
	}

	//left hand
	for(i=0; i<4; i++)
	{
		IK.packet_char_[12+i*9] = i+1;      						//MotorID
		IK.packet_char_[13+i*9] = totalspeed_[i] & 0xFF;    		//Profile Velocity      //initial value = 0
		IK.packet_char_[14+i*9] = (totalspeed_[i] >> 8) & 0xFF;
		IK.packet_char_[15+i*9] = (totalspeed_[i] >> 16) & 0xFF;
		IK.packet_char_[16+i*9] = (totalspeed_[i] >> 24) & 0xFF;
		IK.packet_char_[17+i*9] = totalangle_[i] & 0xFF;    		// positionL
		IK.packet_char_[18+i*9] = (totalangle_[i] >> 8) & 0xFF;
		IK.packet_char_[19+i*9] = (totalangle_[i] >> 16) & 0xFF;
		IK.packet_char_[20+i*9] = (totalangle_[i] >> 24) & 0xFF;
	}

	blk_size = 5 + IK.packet_char_[5];
	unsigned short lh_crc_value = IK.update_crc(0, IK.packet_char_, blk_size);

	//right hand
	for(i=0; i<4; i++)
	{
		IK.packet_char_[12+i*9] = i+5;      						//MotorID
		IK.packet_char_[13+i*9] = totalspeed_[i+4] & 0xFF;    		//Profile Velocity      //initial value = 0
		IK.packet_char_[14+i*9] = (totalspeed_[i+4] >> 8) & 0xFF;
		IK.packet_char_[15+i*9] = (totalspeed_[i+4] >> 16) & 0xFF;
		IK.packet_char_[16+i*9] = (totalspeed_[i+4] >> 24) & 0xFF;
		IK.packet_char_[17+i*9] = totalangle_[i+4] & 0xFF;    		// positionL
		IK.packet_char_[18+i*9] = (totalangle_[i+4] >> 8) & 0xFF;
		IK.packet_char_[19+i*9] = (totalangle_[i+4] >> 16) & 0xFF;
		IK.packet_char_[20+i*9] = (totalangle_[i+4] >> 24) & 0xFF;
	}

	blk_size = 5 + IK.packet_char_[5];
	unsigned short rh_crc_value = IK.update_crc(0, IK.packet_char_, blk_size);

	//foot
	IK.packet_char_[5] = 0x46; // 0x46 = 70  77-7

	IK.packet_char_[12] = 0x09;
	IK.packet_char_[13] = totalspeed_[8] & 0xFF;
	IK.packet_char_[14] = (totalspeed_[8] >> 8) & 0xFF;
	IK.packet_char_[15] = (totalspeed_[8] >> 16) & 0xFF;
	IK.packet_char_[16] = (totalspeed_[8] >> 24) & 0xFF;
	IK.packet_char_[17] = totalangle_[8] & 0xFF;
	IK.packet_char_[18] = (totalangle_[8] >> 8) & 0xFF;
	IK.packet_char_[19] = (totalangle_[8] >> 16) & 0xFF;
	IK.packet_char_[20] = (totalangle_[8] >> 24) & 0xFF;

	//left foot
	for(i=0; i<6; i++)
	{
		IK.packet_char_[21+i*9] = i+10;      						//MotorID
		IK.packet_char_[22+i*9] = totalspeed_[i+9] & 0xFF;    		//Profile Velocity      //initial value = 0
		IK.packet_char_[23+i*9] = (totalspeed_[i+9] >> 8) & 0xFF;
		IK.packet_char_[24+i*9] = (totalspeed_[i+9] >> 16) & 0xFF;
		IK.packet_char_[25+i*9] = (totalspeed_[i+9] >> 24) & 0xFF;
		IK.packet_char_[26+i*9] = totalangle_[i+9] & 0xFF;    		// positionL
		IK.packet_char_[27+i*9] = (totalangle_[i+9] >> 8) & 0xFF;
		IK.packet_char_[28+i*9] = (totalangle_[i+9] >> 16) & 0xFF;
		IK.packet_char_[29+i*9] = (totalangle_[i+9] >> 24) & 0xFF;
	}

	blk_size = 5 + IK.packet_char_[5];
	unsigned short lf_crc_value = IK.update_crc(0, IK.packet_char_, blk_size);

	//right foot
	for(i=0; i<6; i++)
	{
		IK.packet_char_[21+i*9] = i+16;      						//MotorID
		IK.packet_char_[22+i*9] = totalspeed_[i+15] & 0xFF;			//Profile Velocity      //initial value = 0
		IK.packet_char_[23+i*9] = (totalspeed_[i+15] >> 8) & 0xFF;
		IK.packet_char_[24+i*9] = (totalspeed_[i+15] >> 16) & 0xFF;
		IK.packet_char_[25+i*9] = (totalspeed_[i+15] >> 24) & 0xFF;
		IK.packet_char_[26+i*9] = totalangle_[i+15] & 0xFF;    		// positionL
		IK.packet_char_[27+i*9] = (totalangle_[i+15] >> 8) & 0xFF;
		IK.packet_char_[28+i*9] = (totalangle_[i+15] >> 16) & 0xFF;
		IK.packet_char_[29+i*9] = (totalangle_[i+15] >> 24) & 0xFF;
	}

	blk_size = 5 + IK.packet_char_[5];
	unsigned short rf_crc_value = IK.update_crc(0, IK.packet_char_, blk_size);

	*((uint32_t *)init.robot_motion_addr+(42)) = 30;
	*((uint32_t *)init.robot_motion_addr+(43)) = 0x00000070;
	*((uint32_t *)init.robot_motion_addr+(44)) = (lh_crc_value << 16) + rh_crc_value;
	*((uint32_t *)init.robot_motion_addr+(45)) = (lf_crc_value << 16) + rf_crc_value;

	//printf("motion_execute ");

	locus.do_motion();
	IK.initial_inverse_kinematic();

	datamodule_cmd_ = 0;
	update_database_flag_ = false;
	motion_execute_flag_ = false;
}

void Datamodule::set_stand()	//設定站姿參數
{
/*開機初始站姿*/
	int i=0;
	for(i=0; i<21; i++)
	{
		if(i==12||i==18)
			totalspeed_[i] = 80;
		else
			totalspeed_[i] = 40;
	}
	totalangle_[0] = 3072;
	totalangle_[1] = 1898;
	totalangle_[2] = 2048;
	totalangle_[3] = 2048;
	totalangle_[4] = 1024;
	totalangle_[5] = 2198;
	totalangle_[6] = 2048;
	totalangle_[7] = 2048;
	totalangle_[8] = 2048;

	totalangle_[9] = 2048;
	totalangle_[10] = 2028;
	totalangle_[11] = 1652;
	totalangle_[12] = 2737;
	totalangle_[13] = 2410;
	totalangle_[14] = 2047;
	totalangle_[15] = 2068;
	totalangle_[16] = 2058;
	totalangle_[17] = 2410;
	totalangle_[18] = 1337;
	totalangle_[19] = 1674;
	totalangle_[20] = 2047;
	//-------------------------------------------------
	//---------------stand上半身初始值------------------
	for(i=0; i<21; i++)
	{
		if(i==12||i==18)
			Walking_standspeed[i] = 80;
		else
			Walking_standspeed[i] = 40;
	}
	Walking_standangle[0] = 3072;
	Walking_standangle[1] = 1898;
	Walking_standangle[2] = 2048;
	Walking_standangle[3] = 2048;
	Walking_standangle[4] = 1024;
	Walking_standangle[5] = 2198;
	Walking_standangle[6] = 2048;
	Walking_standangle[7] = 2048;
	Walking_standangle[8] = 2048;
	//---------------stand下半身初始值------------------
	Walking_standangle[9] = 2048;
	Walking_standangle[10] = 2058;
	Walking_standangle[11] = 1970;
	Walking_standangle[12] = 2078;
	Walking_standangle[13] = 2058;
	Walking_standangle[14] = 2048;
	Walking_standangle[15] = 2048;
	Walking_standangle[16] = 2058;
	Walking_standangle[17] = 2165;
	Walking_standangle[18] = 2048;
	Walking_standangle[19] = 2063;
	Walking_standangle[20] = 2048;
	//-------------------------------------------------
	//----------------站直初始值------------------------
	for(i=0;i<12;i++)
	{
		if(i==2||i==8)
		{
			Calculate_standspeed[i] = 80;
		}else
		{
			Calculate_standspeed[i] = 40;
		}
		
	}
	Calculate_standangle[0] = 2048;//10_motor
	Calculate_standangle[1] = 2058;//11_motor
	Calculate_standangle[2] = 1970;//12_motor
	Calculate_standangle[3] = 2078;//13_motor
	Calculate_standangle[4] = 2058;//14_motor
	Calculate_standangle[5] = 2048;//15_motor
	Calculate_standangle[6] = 2048;//16_motor
	Calculate_standangle[7] = 2058;//17_motor
	Calculate_standangle[8] = 2165;//18_motor
	Calculate_standangle[9] = 2048;//19_motor
	Calculate_standangle[10] = 2063;//20_motor
	Calculate_standangle[11] = 2048;//21_motor
	//-------------------------------------------------
	motion_execute();
}

string Datamodule::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();
    return str;
}

void Datamodule::saveData()
{
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/data_motor"+tmp+".csv";
    strcat(path, tmp.c_str());

	
    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<double>>::iterator it_motor;

	for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
	{
		savedText += it_motor->first;
		if(it_motor == --map_motor.end())
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
	it_motor = map_motor.begin();
	int max_size = it_motor->second.size();

	for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
	{
		if(max_size < it_motor->second.size())
            max_size = it_motor->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
        {
            if(i < it_motor->second.size())
            {
                if(it_motor == --map_motor.end())
                {
                    savedText += std::to_string(it_motor->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_motor->second[i]) + ",";
                }
            }
            else
            {
                if(it_motor == --map_motor.end())
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
    for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
        it_motor->second.clear();

    name_cont_++;

}
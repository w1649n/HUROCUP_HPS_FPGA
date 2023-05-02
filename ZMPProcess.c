#include "include/ZMPProcess.h"

extern SensorDataProcess sensor;
ZMPProcess::ZMPProcess()
{
	ZMP_kg_offset = new int*[8];
	ZMP_kg_offset[0] = new int[5]{0,40,80,120,160};
	ZMP_kg_offset[1] = new int[5]{0,155,310,465,620};
	ZMP_kg_offset[2] = new int[5]{0,114,228,342,456};
	ZMP_kg_offset[3] = new int[5]{0,86,172,258,344};
	ZMP_kg_offset[4] = new int[5]{0,102,204,306,408};
	ZMP_kg_offset[5] = new int[5]{0,132,264,396,528};
	ZMP_kg_offset[6] = new int[5]{0,135,270,405,540};
	ZMP_kg_offset[7] = new int[5]{0,135,270,405,540};
	ZMP_kg_table = new int[5]{0,1,2,3,4};
	name_cont_ = 0;
	initialize();
}

ZMPProcess::~ZMPProcess()
{
	for(int i = 0; i < 8; i++)delete[] ZMP_kg_offset[i];
	delete[] ZMP_kg_offset;
	delete[] ZMP_kg_table;
}

void ZMPProcess::initialize()
{
	ZMP.initialize();
	for(int i = 0; i < 8; i++)sensor_offset_data_sum[i] = 0;
	for(int i = 0; i < 8; i++)origen_sensor_data[i] = 0;
	for(int i = 0; i < 8; i++)sensor_force[i] = 0;
	sensor_offset_data_count = 0;
	force_left = 0;
	force_right = 0;
	torque_left_x = 0;
	torque_left_y = 0;

    std::vector<float> temp;
	if(map_zmp.empty())
	{
		map_zmp["P1"] = temp;
		map_zmp["P2"] = temp;
		map_zmp["P3"] = temp;
		map_zmp["P4"] = temp;
		map_zmp["R_zmp_x"] = temp;
        map_zmp["R_zmp_y"] = temp;
        map_zmp["R_force"] = temp;
		map_zmp["L_zmp_x"] = temp;
        map_zmp["L_zmp_y"] = temp;
        map_zmp["L_force"] = temp;
		map_zmp["D_zmp_x"] = temp;
		map_zmp["D_zmp_y"] = temp;
        map_zmp["D_force"] = temp;
	}

}

void ZMPProcess::getSensorValue()
{
	for(int count=0; count<4; count++)
	{
	origen_sensor_data[count] = sensor.press_left_[count];
	}
	for(int count=0; count<4; count++)
	{
	origen_sensor_data[count+4] = sensor.press_right_[count];
	}
}

void ZMPProcess::digital2KGProcess()
{
	double 	ZMP_S[8];	
    for(int press_count = 0;press_count < 8;press_count++)
	{
		if(origen_sensor_data[press_count]<0)
		{
			origen_sensor_data[press_count] = 0;
		}
		ZMP_S[press_count] = 0;
		for(int kg_count = 0; kg_count < 4; kg_count++)
		{
			if ((origen_sensor_data[press_count] >= ZMP_kg_offset[press_count][kg_count]) && (origen_sensor_data[press_count] < ZMP_kg_offset[press_count][kg_count+1]))
			{
				if((ZMP_kg_offset[press_count][kg_count+1] - ZMP_kg_offset[press_count][kg_count]) >= 0)
				{
					ZMP_S[press_count] = (double)ZMP_kg_table[kg_count] + (double)((ZMP_kg_table[kg_count+1]-ZMP_kg_table[kg_count])*((double)(origen_sensor_data[press_count] - ZMP_kg_offset[press_count][kg_count])/(double)(ZMP_kg_offset[press_count][kg_count+1] - ZMP_kg_offset[press_count][kg_count])));
                    break;
                }
			}
            else if(kg_count == 3 && origen_sensor_data[press_count] >= ZMP_kg_offset[press_count][kg_count+1])
            {
                ZMP_S[press_count] = 5;
            }
		}
	}
    double sensor_digital_	        = ZMP_S[0]+ZMP_S[1]+ZMP_S[2]+ZMP_S[3] + ZMP_S[4]+ZMP_S[5]+ZMP_S[6]+ZMP_S[7];
	double sensor_digital_left_	    = ZMP_S[0]+ZMP_S[1]+ZMP_S[2]+ZMP_S[3];
	double sensor_digital_right_	= ZMP_S[4]+ZMP_S[5]+ZMP_S[6]+ZMP_S[7];

    force_left   = sensor_digital_left_;//*9.8;
    force_right  = sensor_digital_right_;

    for(int i = 0; i < 8; i++)
    {
        sensor_force[i] = ZMP_S[i];
    }
    torque_left_x = ((ZMP_S[0]+ZMP_S[1]) - (ZMP_S[2]+ZMP_S[3]))*SINGLE_FOOT_WEIGHT_X/100.0*9.8;
    torque_left_y = ((ZMP_S[0]+ZMP_S[3]) - (ZMP_S[1]+ZMP_S[2]))*SINGLE_FOOT_WEIGHT_EQUAL_Y/100.0*9.8;
    if(sensor_digital_ > 0.2)
	{
		ZMP.feet_pos.y = (double)((((ZMP_S[0]+ZMP_S[3]-ZMP_S[5]-ZMP_S[6])*DOUBLE_FEET_WEIGHT_FAR_Y) + ((ZMP_S[1]+ZMP_S[2]-ZMP_S[4]-ZMP_S[7]) * DOUBLE_FEET_WEIGHT_NEAR_Y) )/(double)sensor_digital_);
		ZMP.feet_pos.x = (double)((ZMP_S[0]+ZMP_S[1]+ZMP_S[4]+ZMP_S[5]-ZMP_S[2]-ZMP_S[3]-ZMP_S[6]-ZMP_S[7]) * DOUBLE_FEET_WEIGHT_X )/(double)sensor_digital_;
	}
	else
	{
		ZMP.feet_pos.y = 0;
		ZMP.feet_pos.x = 0;
	}
    if(sensor_digital_left_ > 0.2)
	{
		ZMP.left_pos.y = ((ZMP_S[0]+ZMP_S[3]-ZMP_S[1]-ZMP_S[2]) * SINGLE_FOOT_WEIGHT_EQUAL_Y)/sensor_digital_left_;
		ZMP.left_pos.x = ((ZMP_S[0]+ZMP_S[1]-ZMP_S[2]-ZMP_S[3]) * SINGLE_FOOT_WEIGHT_X)/sensor_digital_left_;
	}
	else
	{
		ZMP.left_pos.y = 0;
		ZMP.left_pos.x = 0;
	}
    if(sensor_digital_right_ > 0.2)
	{
		ZMP.right_pos.y = ((ZMP_S[4]+ZMP_S[7]-ZMP_S[5]-ZMP_S[6]) * SINGLE_FOOT_WEIGHT_EQUAL_Y)/sensor_digital_right_;
		ZMP.right_pos.x = ((ZMP_S[4]+ZMP_S[5]-ZMP_S[6]-ZMP_S[7]) * SINGLE_FOOT_WEIGHT_X)/sensor_digital_right_;			
	}
	else
	{ 
		ZMP.right_pos.y = 0;
		ZMP.right_pos.x = 0;		
	}

	map_zmp.find("P1")->second.push_back(origen_sensor_data[0]);
	map_zmp.find("P2")->second.push_back(origen_sensor_data[1]);
	map_zmp.find("P3")->second.push_back(origen_sensor_data[2]);
	map_zmp.find("P4")->second.push_back(origen_sensor_data[3]);
	map_zmp.find("R_zmp_x")->second.push_back(ZMP.right_pos.y);
    map_zmp.find("R_zmp_y")->second.push_back(ZMP.right_pos.x);
    map_zmp.find("R_force")->second.push_back(sensor_digital_left_);
	map_zmp.find("L_zmp_x")->second.push_back(ZMP.left_pos.y);
    map_zmp.find("L_zmp_y")->second.push_back(ZMP.left_pos.x);
    map_zmp.find("L_force")->second.push_back(sensor_digital_right_);
	map_zmp.find("D_zmp_x")->second.push_back(ZMP.feet_pos.y);
    map_zmp.find("D_zmp_y")->second.push_back(ZMP.feet_pos.x);
    map_zmp.find("D_force")->second.push_back(sensor_digital_);

}

void ZMPProcess::setpSensorDataOffset(void *origen_sensor_data)
{
	for(int i = 0; i < 8; i++)
	{
		sensor_offset_data_sum[i] += ((int*)origen_sensor_data)[i];
	}
	sensor_offset_data_count++;
}

void ZMPProcess::setpOrigenSensorData(void *origen_sensor_data)
{
	if(sensor_offset_data_count == 0)
	{
		for(int i = 0; i < 8; i++)
		{
			this->origen_sensor_data[i] = ((int*)origen_sensor_data)[i];
		}
	}
	else
	{
		for(int i = 0; i < 8; i++)
		{
			this->origen_sensor_data[i] = ((int*)origen_sensor_data)[i] - sensor_offset_data_sum[i]/sensor_offset_data_count;
		}
	}
}

ZMPParam ZMPProcess::getZMPValue()
{
    getSensorValue();
	digital2KGProcess();
    return this->ZMP;
}

float ZMPProcess::getForceLeft()
{
    return this->force_left;
}

float ZMPProcess::getForceRight()
{
    return this->force_right;
}

double *ZMPProcess::getpSensorForce()
{
    return this->sensor_force;
}

int *ZMPProcess::getpOrigenSensorData()
{
    return this->origen_sensor_data;
}

float ZMPProcess::getTorqueLeftX()
{
    return this->torque_left_x;
}

float ZMPProcess::getTorqueLeftY()
{
    return this->torque_left_y;
}

void ZMPProcess::saveData()
{
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/ZMP_Trajectory_"+tmp+".csv";
    strcat(path, tmp.c_str());

    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<float>>::iterator it_zmp;

	for(it_zmp = map_zmp.begin(); it_zmp != map_zmp.end(); it_zmp++)
	{
		savedText += it_zmp->first;
		if(it_zmp == --map_zmp.end())
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
	it_zmp = map_zmp.begin();
	int max_size = it_zmp->second.size();

	for(it_zmp = map_zmp.begin(); it_zmp != map_zmp.end(); it_zmp++)
	{
		if(max_size < it_zmp->second.size())
            max_size = it_zmp->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_zmp = map_zmp.begin(); it_zmp != map_zmp.end(); it_zmp++)
        {
            if(i < it_zmp->second.size())
            {
                if(it_zmp == --map_zmp.end())
                {
                    savedText += std::to_string(it_zmp->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_zmp->second[i]) + ",";
                }
            }
            else
            {
                if(it_zmp == --map_zmp.end())
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
    for(it_zmp = map_zmp.begin(); it_zmp != map_zmp.end(); it_zmp++)
        it_zmp->second.clear();

    name_cont_++;
}
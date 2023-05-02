#include "include/Feedback_Motor.h"

extern Initial init;
extern Walkinggait walkinggait;
Feedback_Motor::Feedback_Motor()
{
    name_cont_ = 0;
	std::vector<double> temp;
    if(map_feedback.empty())
    {   
        map_feedback["feedback_10"] = temp;
        map_feedback["feedback_11"] = temp;
        map_feedback["feedback_12"] = temp;
        map_feedback["feedback_13"] = temp;
        map_feedback["feedback_14"] = temp;
        map_feedback["feedback_15"] = temp;
        map_feedback["feedback_16"] = temp;
        map_feedback["feedback_17"] = temp;
        map_feedback["feedback_18"] = temp;
        map_feedback["feedback_19"] = temp;
        map_feedback["feedback_20"] = temp;
        map_feedback["feedback_21"] = temp;
        map_feedback["time_point_"] = temp;
        map_feedback["update_motor_data_left_foot_flag_"] = temp;
        map_feedback["update_motor_data_right_foot_flag_"] = temp;
    }
}

Feedback_Motor::~Feedback_Motor()
{

}

void Feedback_Motor::load_motor_data_left_foot()
{
    int state = 0;
    int count = 0;

    for(;;)
    {
        if(state == 0)
        {
            update_motor_data_left_foot_flag_ = false;
            if(*(uint32_t *)init.p2h_set_hps_read_motor_data_leftfoot_addr)
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
                motor_data_left_foot_[count] = *(uint32_t *)init.p2h_motor_data_leftfoot_addr;
                count++;
                *(uint32_t *)init.h2p_read_motor_data_leftfoot_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_motor_data_leftfoot_pulse_addr = 0;
                continue;
            }
            else
            {
                update_motor_data_left_foot_flag_ = true;
                state = 0;
                break;
            }
        }
    }
    update_motor_data_left_foot();
}

void Feedback_Motor::update_motor_data_left_foot()
{
    if(update_motor_data_left_foot_flag_)
    {
        //printf("\n data :%d , %d \n",motor_data_left_foot_[0],motor_data_left_foot_[1]);
    }
}
void Feedback_Motor::load_motor_data_right_foot()
{
    int state = 0;
    int count = 0;

    for(;;)
    {
        if(state == 0)
        {
            update_motor_data_right_foot_flag_ = false;
            if(*(uint32_t *)init.p2h_set_hps_read_motor_data_rightfoot_addr)
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
                motor_data_right_foot_[count] = *(uint32_t *)init.p2h_motor_data_rightfoot_addr;
                count++;
                *(uint32_t *)init.h2p_read_motor_data_rightfoot_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_motor_data_rightfoot_pulse_addr = 0;
                continue;
            }
            else
            {
                update_motor_data_right_foot_flag_ = true;
                state = 0;
                break;
            }
        }
    }
    update_motor_data_right_foot();
}

void Feedback_Motor::update_motor_data_right_foot()
{
    if(update_motor_data_right_foot_flag_)
    {
        //printf("\n data :%d , %d \n",motor_data_right_foot_[0],motor_data_right_foot_[1]);
    }
}

void Feedback_Motor::pushData()
{
    map_feedback.find("feedback_10")->second.push_back(motor_data_left_foot_[0]);
    map_feedback.find("feedback_11")->second.push_back(motor_data_left_foot_[1]);
    map_feedback.find("feedback_12")->second.push_back(motor_data_left_foot_[2]);
    map_feedback.find("feedback_13")->second.push_back(motor_data_left_foot_[3]);
    map_feedback.find("feedback_14")->second.push_back(motor_data_left_foot_[4]);
    map_feedback.find("feedback_15")->second.push_back(motor_data_left_foot_[5]);
    map_feedback.find("feedback_16")->second.push_back(motor_data_right_foot_[0]);
    map_feedback.find("feedback_17")->second.push_back(motor_data_right_foot_[1]);
    map_feedback.find("feedback_18")->second.push_back(motor_data_right_foot_[2]);
    map_feedback.find("feedback_19")->second.push_back(motor_data_right_foot_[3]);
    map_feedback.find("feedback_20")->second.push_back(motor_data_right_foot_[4]);
    map_feedback.find("feedback_21")->second.push_back(motor_data_right_foot_[5]);
    map_feedback.find("time_point_")->second.push_back(walkinggait.time_point_);
    map_feedback.find("time_point_")->second.push_back(update_motor_data_left_foot_flag_);
    map_feedback.find("time_point_")->second.push_back(update_motor_data_right_foot_flag_);
}

string Feedback_Motor::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();
    return str;
}

void Feedback_Motor::saveData()
{
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/motor_feedback"+tmp+".csv";
    strcat(path, tmp.c_str());

	
    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<double>>::iterator it_motor;

	for(it_motor = map_feedback.begin(); it_motor != map_feedback.end(); it_motor++)
	{
		savedText += it_motor->first;
		if(it_motor == --map_feedback.end())
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
	it_motor = map_feedback.begin();
	int max_size = it_motor->second.size();

	for(it_motor = map_feedback.begin(); it_motor != map_feedback.end(); it_motor++)
	{
		if(max_size < it_motor->second.size())
            max_size = it_motor->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_motor = map_feedback.begin(); it_motor != map_feedback.end(); it_motor++)
        {
            if(i < it_motor->second.size())
            {
                if(it_motor == --map_feedback.end())
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
                if(it_motor == --map_feedback.end())
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
    for(it_motor = map_feedback.begin(); it_motor != map_feedback.end(); it_motor++)
        it_motor->second.clear();

    name_cont_++;

}
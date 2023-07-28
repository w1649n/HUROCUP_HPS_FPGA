#include "include/Feedback_Motor.h"

extern Initial init;

Feedback_Motor::Feedback_Motor()
{

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
        int count = 0;
        // printf("\n data :%d , %d \n",motor_data_left_foot_[0],motor_data_left_foot_[1]);
    }
}
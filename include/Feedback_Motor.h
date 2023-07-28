#ifndef FEEDBACK_MOTOR_H_
#define FEEDBACK_MOTOR_H_
#include <stdio.h>
#include <string.h>
#include "Initial.h"

class Feedback_Motor
{
    public:
        Feedback_Motor();
        ~Feedback_Motor();
        void load_motor_data_left_foot();
        void update_motor_data_left_foot();
        int motor_data_left_foot_[6];
    private:
        bool update_motor_data_left_foot_flag_;
};

#endif
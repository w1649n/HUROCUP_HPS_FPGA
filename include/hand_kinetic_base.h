#ifndef HAND_KINETIC_BASE_H_
#define HAND_KINETIC_BASE_H_

#include <math.h>
#include <stdio.h>
#include <iostream>
#include "fVector.h"
#include "fMatrix.h"
#include <stdlib.h>

#include "Initial.h"

#include "Inverse_Kinematic.h"

using namespace std;


/******************* Define******************************/
#define PI      3.1415926535897932384626433832795   //pi
#define PI_2    1.5707963267948966192313216916398   //pi/2
#define PI_3_2  4.7123889803846898576939650749193   //3*pi/2
#define PI_3    1.0471975511965977461542144610932   //pi/3
#define PI_TO_OUTPUT 651.8986469044032953093479120548 //360/(2pi)*4096/360
#define PI_6	PI / 6.0

struct three_axis_coordinate
{
	double x;
	double y;
	double z;
};

struct  four_dof
{
	double dof1;
	double dof2;
	double dof3;
	double dof4;
};

class Hand_InverseKinematic
{
public:
	Hand_InverseKinematic();
	~Hand_InverseKinematic();

	void Kinetic_Main(double rp_x, double rp_y, double rp_z);
	void Kinetic_Inverse();
	void Kinetic_Forward(double dof1, double dof2, double dof3, double dof4);
	double *GenerateTransformationMatrices(double kinetic_angle, fVector dh);
	unsigned short update_crc(unsigned short, unsigned char *, unsigned short);
	void load_hand_data();
	void update_hand_data();

	void package_init();
	void hand_package();

	struct three_axis_coordinate recevie_point_;
	struct three_axis_coordinate confirm_point_;
	struct three_axis_coordinate final_point_;
	struct four_dof init_;
	struct four_dof transmit_dof_;

	unsigned int hand_data_[2];
	double hand_point_[3];
	unsigned char hand_char_[203];



private:
	bool update_hand_data_flag_;
	int angle_[21];
    int output_base_[21];
    int thta_base_[21];
    double past_thta_[21];
    double delay_time_[21];
    double past_delay_time_[21];
    unsigned int output_speed_[21];
    unsigned int output_angle_[21];

	bool hand_do_motion_;
};

#endif
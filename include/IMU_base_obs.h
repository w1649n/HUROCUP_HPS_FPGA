

#ifndef  __IMU_BASE_OBS_H__
#define  __IMU_BASE_OBS_H__


#include <iostream>
#include <Eigen/Dense>
#include <cmath>       /* isnan, sqrt */
#include <map>
#include <fstream>
#include <vector>
#include "Initial.h"

using namespace Eigen;

class IMU_base_obs {

private:
	double alpha;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* input */
    Vector3d WpA,BpA,Theta;
    /* output */
    Vector3d WpB;
	Vector3d WvB;

    Matrix<double, 3, 3> WRB;	/* Rotation Matrix B to W */
	Vector3d WpB_hat;			/* state_hat */
	Vector3d WvB_hat;			/* state_hat */

	bool first_time;

    IMU_base_obs();

    //Sampling time = 1.0/Sampling Frequency
	double dt;
	double T_cutoff;

    /** @fn void setparameter(double dtt)
	 *  @brief sets the discretization of the Floating Base Estimater
	 *  @param dtt sampling time in seconds
	 */
    void setparameter(double dtt) {
		dt = dtt;
	}

	void setparameter(double dtt,double T_cut) {
		dt = dtt;
		T_cutoff = T_cut;
	}

    void setinputdata(Vector3d WpA_,Vector3d BpA_,Vector3d Theta_) {
		WpA = WpA_;
        BpA = BpA_;
        Theta = Theta_;
	}

    /**
	 *  @fn void init()
	 *  @brief Initializes the Base Estimator
     *  @details
     *   Initializes:  State-Error Covariance  P, State x, Linearization Matrices for process and measurement models Acf, Lcf, Hf and rest class variables
	*/
	void init();
    void run();
    void estimatePosition();
	void estimateVelocity();
	void saveState();
    /** @brief Computes Rotation Matrix from Euler Angles according to YPR convention
	 *  @param angles_ 3D Vector with Roll-Pitch-Yaw
	 *  @return  3x3 Rotation in SO(3) group
	 */
	inline Matrix3d getRotationMatrix(
			Vector3d angles_) {
		Matrix3d res, Rz, Ry, Rx;
		
		if(angles_(0)>180||angles_(1)>180||angles_(2)>180)
		{
			angles_ = Vector3d::Zero();
		}
		angles_(0) = angles_(0)*PI/180.0f;
		angles_(1) = angles_(1)*PI/180.0f;
		angles_(2) = angles_(2);
		

		/*
		angles_(1) = angles_(1);
		angles_(2) = angles_(2);
		angles_(3) = angles_(3);
		*/

		Rz = Matrix3d::Zero();
		Rz(0, 0) = cos(angles_(2));
		Rz(0, 1) = -sin(angles_(2));
		Rz(1, 0) = sin(angles_(2));
		Rz(1, 1) = cos(angles_(2));
		Rz(2, 2) = 1.000;

		Ry = Matrix3d::Zero();
		Ry(0, 0) = cos(angles_(1));
		Ry(0, 2) = sin(angles_(1));
		Ry(1, 1) = 1.000;
		Ry(2, 0) = -sin(angles_(1));
		Ry(2, 2) = cos(angles_(1));

		Rx = Matrix3d::Zero();
		Rx(0, 0) = 1.00;
		Rx(1, 1) = cos(angles_(0));
		Rx(1, 2) = -sin(angles_(0));
		Rx(2, 1) = sin(angles_(0));
		Rx(2, 2) = cos(angles_(0));

		// YAW PITCH ROLL Convention for right handed counterclowise coordinate systems
		res = Rz * Ry * Rx;

		return res;
	}

	/* 存檔用 */
    string DtoS(double value);
    void saveData();
    std::map<std::string, std::vector<float>> map_com;
    int name_cont_;
	/* ---- */
};

#endif
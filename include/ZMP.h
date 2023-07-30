#ifndef  __ZMP_H__
#define  __ZMP_H__

#include <iostream>
#include <Eigen/Dense>
#include <cmath>       
#include <map>
#include <fstream>
#include <vector>
#include "Initial.h"

using namespace Eigen;

#define DAMPING_GAIN 0.08   
#define WEIGHT 4  // (kg)
#define foot_W 5.15  //(cm)
#define foot_L 3.5 

class Force_Diff_Ctrl_base
{
private:
   
    /* input */
    double L_force, R_force;
    
    double x1, x2, x3, y1, y2, y3;

    void ZMP_Foot_force_difference_control();

public:
    Force_Diff_Ctrl_base()
    {}
    ~Force_Diff_Ctrl_base()
    {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* input */
    double Lfx, Lfy, Rfx, Rfy, Zmpx, Zmpy;
    double lpt, rpt;
    /* output */
    double Z_ctrl;

    Vector2d v;
    double alpha, alpha_x, alpha_y;
    double Ref_left_f, Ref_right_f;
    double l_com_x, l_com_y, r_com_x, r_com_y;
    double cls_lx, cls_ly, cls_rx, cls_ry;
    double temp_com_point_lx, temp_com_point_ly, temp_point_lx, temp_point_ly;
    double temp_com_point_rx, temp_com_point_ry, temp_point_rx, temp_point_ry;
    double com_pos_lx, com_pos_ly, com_pos_rx, com_pos_ry;
    double temp_lx, temp_ly, temp_rx, temp_ry;

    void setinputdata(double Lfx_, double Lfy_, double Rfx_, double Rfy_, double Zmpx_, double Zmpy_, double lpt_, double rpt_, double L_force_, double R_force_)
    {
        Lfx = Lfx_;
        Lfy = Lfy_;
        Lfy = Lfy_;
        Rfx = Rfx_;
        Rfy = Rfy_;
        Zmpx = Zmpx_;
        Zmpy = Zmpy_;
        lpt = lpt_;
        rpt = rpt_;
        L_force = L_force_;
        R_force = R_force_;

    }

    void run();
    void ZMP_Distributur();
    void closest_point_left();
    void closest_point_right();
    //void Coordinate_conversion(int side, double fx_, double fy_, double com_x_, double com_y_, double theta);
    void alpha_point();

};


#endif /* __ZMP_H__ */


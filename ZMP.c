#include <iostream>
#include <math.h>
#include <complex>
#include"include/ZMP.h"

using namespace std;
 
void Force_Diff_Ctrl_base::run()
{
    ZMP_Distributur();
    ZMP_Foot_force_difference_control();
}

void Force_Diff_Ctrl_base::ZMP_Distributur()
{
    closest_point_right();
    closest_point_left();
    alpha_point();

    alpha = abs(sqrt(pow((cls_lx - alpha_x), 2) + pow((cls_ly - alpha_y), 2)) / sqrt(pow((cls_lx - cls_rx), 2) + pow((cls_ly - cls_ry), 2)));
    if (alpha > 1)
    {
        alpha = 1;
    }

    Ref_left_f = -alpha * WEIGHT;
    Ref_right_f = -(1 - alpha) * WEIGHT;

}

void Force_Diff_Ctrl_base::ZMP_Foot_force_difference_control()
{
    Z_ctrl = ((L_force - R_force) - (Ref_left_f - Ref_right_f)) * DAMPING_GAIN;
    /*
    if (L_force == 0 || R_force == 0)
    {
        Z_ctrl = 0;
    }
    else
    {
        Z_ctrl = ((L_force - R_force) - (Ref_left_f - Ref_right_f)) * DAMPING_GAIN;
    }
    */
}

void Force_Diff_Ctrl_base::closest_point_left()  //0=left 1=right
{
    temp_point_lx = Zmpx - Lfx;
    temp_point_ly = Zmpy - Lfy;

    temp_com_point_lx = temp_point_lx * cos(-lpt) - temp_point_ly * sin(-lpt);
    temp_com_point_ly = temp_point_lx * sin(-lpt) + temp_point_ly * cos(-lpt);

    if (temp_com_point_lx < -foot_W && temp_com_point_ly > foot_L)
    {
        com_pos_lx = -foot_W;
        com_pos_ly = foot_L;
    }
    else if (temp_com_point_lx < -foot_W && temp_com_point_ly < -foot_L)
    {
        com_pos_lx = -foot_W;
        com_pos_ly = -foot_L;
    }
    else if (temp_com_point_lx < -foot_W)
    {
        com_pos_lx = -foot_W;
        com_pos_ly = temp_com_point_ly;
    }
    else if (temp_com_point_lx > foot_W && temp_com_point_ly > foot_L)
    {
        com_pos_lx = foot_W;
        com_pos_ly = foot_L;
    }
    else if (temp_com_point_lx > foot_W && temp_com_point_ly < -foot_L)
    {
        com_pos_lx = foot_W;
        com_pos_ly = -foot_L;
    }
    else if (temp_com_point_lx > foot_W)
    {
        com_pos_lx = foot_W;
        com_pos_ly = temp_com_point_ly;
    }
    else if (temp_com_point_ly > foot_L)
    {
        com_pos_lx = temp_com_point_lx;
        com_pos_ly = foot_L;
    }
    else if (temp_com_point_ly < -foot_L)
    {
        com_pos_lx = temp_com_point_lx;
        com_pos_ly = -foot_L;
    }
    else
    {
        if (temp_com_point_lx > 0)
        {
            temp_lx = foot_W - temp_com_point_lx;
        }
        else
        {
            temp_lx = foot_W + temp_com_point_lx;
        }
        if (temp_com_point_ly > 0)
        {
            temp_ly = foot_L - temp_com_point_ly;
        }
        else
        {
            temp_ly = foot_L + temp_com_point_ly;
        }
        if (temp_lx < temp_ly)
        {
            if (temp_com_point_lx > 0)
            {
                com_pos_lx = foot_W;
                com_pos_ly = temp_com_point_ly;
            }
            else
            {
                com_pos_lx = -foot_W;
                com_pos_ly = temp_com_point_ly;
            }
        }
        else
        {
            if (temp_com_point_ly > 0)
            {
                com_pos_lx = temp_com_point_lx;
                com_pos_ly = foot_L;
            }
            else
            {
                com_pos_lx = temp_com_point_lx;
                com_pos_ly = -foot_L;
            }
        }
    }

    temp_lx = com_pos_lx * cos(lpt) - com_pos_ly * sin(lpt);
    temp_ly = com_pos_lx * sin(lpt) + com_pos_ly * cos(lpt);
    cls_lx = temp_lx + Lfx;
    cls_ly= temp_ly + Lfy;
}
void Force_Diff_Ctrl_base::closest_point_right()  //0=left 1=right
{
    temp_point_rx = Zmpx - Rfx;
    temp_point_ry = Zmpy - Rfy;

    temp_com_point_rx = temp_point_rx * cos(-rpt) - temp_point_ry * sin(-rpt);
    temp_com_point_ry = temp_point_rx * sin(-rpt) + temp_point_ry * cos(-rpt);

    if (temp_com_point_rx < -foot_W && temp_com_point_ry > foot_L)
    {
        com_pos_rx = -foot_W;
        com_pos_ry = foot_L;
    }
    else if (temp_com_point_rx < -foot_W && temp_com_point_ry < -foot_L)
    {
        com_pos_rx = -foot_W;
        com_pos_ry = -foot_L;
    }
    else if (temp_com_point_rx < -foot_W)
    {
        com_pos_rx = -foot_W;
        com_pos_ry = temp_com_point_ry;
    }
    else if (temp_com_point_rx > foot_W && temp_com_point_ry > foot_L)
    {
        com_pos_rx = foot_W;
        com_pos_ry = foot_L;
    }
    else if (temp_com_point_rx > foot_W && temp_com_point_ry < -foot_L)
    {
        com_pos_rx = foot_W;
        com_pos_ry = -foot_L;
    }
    else if (temp_com_point_rx > foot_W)
    {
        com_pos_rx = foot_W;
        com_pos_ry = temp_com_point_ry;
    }
    else if (temp_com_point_ry > foot_L)
    {
        com_pos_rx = temp_com_point_rx;
        com_pos_ry = foot_L;
    }
    else if (temp_com_point_ry < -foot_L)
    {
        com_pos_rx = temp_com_point_rx;
        com_pos_ry = -foot_L;
    }
    else
    {
        if (temp_com_point_rx > 0)
        {
            temp_rx = foot_W - temp_com_point_rx;
        }
        else
        {
            temp_rx = foot_W + temp_com_point_rx;
        }
        if (temp_com_point_ry > 0)
        {
            temp_ry = foot_L - temp_com_point_ry;
        }
        else
        {
            temp_ry = foot_L + temp_com_point_ry;
        }
        if (temp_rx < temp_ry)
        {
            if (temp_com_point_rx > 0)
            {
                com_pos_rx = foot_W;
                com_pos_ry = temp_com_point_ry;
            }
            else
            {
                com_pos_rx = -foot_W;
                com_pos_ry = temp_com_point_ry;
            }
        }
        else
        {
            if (temp_com_point_ry > 0)
            {
                com_pos_rx = temp_com_point_rx;
                com_pos_ry = foot_L;
            }
            else
            {
                com_pos_rx = temp_com_point_rx;
                com_pos_ry = -foot_L;
            }
        }
    }

    temp_rx = com_pos_rx * cos(rpt) - com_pos_ry * sin(rpt);
    temp_ry = com_pos_rx * sin(rpt) + com_pos_ry * cos(rpt);
    cls_rx = temp_rx + Rfx;
    cls_ry = temp_ry + Rfy;
}

//void Force_Diff_Ctrl_base::Coordinate_conversion(int side, double fx_, double fy_, double com_x_, double com_y_, double theta)  //0=left 1=right
//{
//  temp_x = com_x_ * cos(lpt) - com_y_ * sin(theta);
//    temp_y = com_x_ * sin(lpt) + com_y_ * cos(theta);
//
//    if (side == 0) //0=left 1=right
//    {
//        cls_lx = temp_x + fx_;
//        cls_ly = temp_y + fy_;
//    }
//    else
//    {
//        cls_rx = temp_x + fx_;
//        cls_ry = temp_y + fy_;
//    }


//}

void  Force_Diff_Ctrl_base::alpha_point()
{
    x1 = cls_lx;
    y1 = cls_ly;
    x2 = cls_rx;
    y2 = cls_ry;

    x3 = Zmpx;
    y3 = Zmpy;

    v << (x1 - x2),
        (y1 - y2);

    alpha_y = ((x3 - x2) * (x1 - x2) * (y1 - y2) + y3 * pow((y1 - y2), 2) + y2 * pow((x1 - x2), 2)) / pow(v.norm(), 2);
    alpha_x = ((x1 - x2) * x2 * (y1 - y2) + (x1 - x2) * (x1 - x2) * (alpha_y - y2)) / ((x1 - x2) * (y1 - y2));


    if (x1 == x2)
    {
        alpha_x = x1;
    }

    if (y1 == y2)
    {
        alpha_x = x3;
    }

}




#include "include/IMU_base_obs.h"

IMU_base_obs::IMU_base_obs(){
    /* initialization */
    first_time = 1;
    name_cont_ = 0;                   /* 檔名序號 */
    init();
        /* 存檔用 */
    std::vector<float> temp;
	if(map_com.empty())
	{
		map_com["real_com_x"] = temp;
        map_com["real_com_y"] = temp;
        map_com["real_com_z"] = temp;
        map_com["K_com_x"] = temp;
        map_com["K_com_y"] = temp;
        map_com["K_com_x_v"] = temp;
        map_com["K_com_y_v"] = temp;
        map_com["K_com_z_v"] = temp;
        map_com["K_zmp_x"] = temp;
        map_com["K_zmp_y"] = temp;
        map_com["desired_com_y"] = temp;
	}
    /* ----*/
}

void IMU_base_obs::init() {
    WpA = Vector3d::Zero();
    BpA = Vector3d::Zero();
    WpB = Vector3d::Zero();
    WpB_hat = Vector3d::Zero();
    WvB = Vector3d::Zero();
    WvB_hat = Vector3d::Zero();
    WRB = Matrix<double, 3, 3>::Zero();

}

void IMU_base_obs::initial_Kalmen_Filter(
        Matrix<double, 3, 3> A_,
        Matrix<double, 1, 3> C_){

    A = A_;
    C << 1, 0, 0;

    P_x << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    cout << "P_X" << endl;
    P_y << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;

    cout << "P_y" << endl;

    Q << 0.0001, 0, 0,
         0, 0.0001, 0,
         0, 0, 0.0001;
    cout << "Q" << endl;
    R = 0.05;
    X_.setZero();
    Y_.setZero();

}

void IMU_base_obs::reset_Kalmen_Filter(){
    P_x << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    P_y << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;         

    X_.setZero();
    Y_.setZero();
    X.setZero();
    Y.setZero();
}

void IMU_base_obs::Kalmen_Filter(){
    /* 求x軸狀態 */
    // X_.noalias() = F * X + G * r;
    P_.noalias() = A * P_x * A.transpose() + Q;
    K.noalias() = P_ * C.transpose()/((C * P_ * C.transpose()) + R );
    X.noalias() = X_ + K * (WpB(0) - C * X_);
    P_x.noalias() = (Matrix<double, 3, 3>::Identity(3,3) - K * C) * P_;
    /* --- */

    /* 求y軸狀態 */
    P_.noalias() = A * P_y * A.transpose() + Q;
    K.noalias() = P_ * C.transpose()/((C * P_ * C.transpose()) + R );
    Y.noalias() = Y_ + K * (WpB(1) - C * Y_);
    P_y.noalias() = (Matrix<double, 3, 3>::Identity(3,3) - K * C) * P_;
    /* --- */
}

void IMU_base_obs::estimatePosition(){
    WRB = getRotationMatrix(Theta);
    WpB.noalias() = WRB * BpA;
    WpB.noalias() = WpA - WpB;
}

void IMU_base_obs::estimateVelocity(){
    alpha = dt/T_cutoff;
    WvB = alpha*(WpB-WpB_hat)/dt+(1-alpha)*WvB_hat;
}

void IMU_base_obs::saveState(){
    WpB_hat = WpB;
}

void IMU_base_obs::run(){
    estimatePosition();
    // estimateVelocity();
    Kalmen_Filter();
    saveState();

    /* 存檔用 */
    map_com.find("real_com_x")->second.push_back(WpB(0));
    map_com.find("real_com_y")->second.push_back(WpB(1));
    map_com.find("real_com_z")->second.push_back(WpB(2));
    map_com.find("K_com_x")->second.push_back(X(0));
    map_com.find("K_com_y")->second.push_back(Y(0));
    map_com.find("K_com_x_v")->second.push_back(X(1));
    map_com.find("K_com_y_v")->second.push_back(Y(1));
    map_com.find("K_com_z_v")->second.push_back(WvB(2));
    map_com.find("K_zmp_x")->second.push_back(X(2));
    map_com.find("K_zmp_y")->second.push_back(Y(2));

    
    //map_com.find("desired_com_y")->second.push_back(WpA(1)-BpA(1));
    /* ----*/
    
}

void IMU_base_obs::saveData()
{
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/CoM_Trajectory_"+tmp+".csv";
    strcat(path, tmp.c_str());

    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<float>>::iterator it_com;

	for(it_com = map_com.begin(); it_com != map_com.end(); it_com++)
	{
		savedText += it_com->first;
		if(it_com == --map_com.end())
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
	it_com = map_com.begin();
	int max_size = it_com->second.size();

	for(it_com = map_com.begin(); it_com != map_com.end(); it_com++)
	{
		if(max_size < it_com->second.size())
            max_size = it_com->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_com = map_com.begin(); it_com != map_com.end(); it_com++)
        {
            if(i < it_com->second.size())
            {
                if(it_com == --map_com.end())
                {
                    savedText += std::to_string(it_com->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_com->second[i]) + ",";
                }
            }
            else
            {
                if(it_com == --map_com.end())
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
    for(it_com = map_com.begin(); it_com != map_com.end(); it_com++)
        it_com->second.clear();

    name_cont_++;
}
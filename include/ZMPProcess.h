#ifndef ZMP_PROCESS_H_
#define ZMP_PROCESS_H_
#include "DefineDataStruct.h"
#include "Sensor.h"

#include <map>
#include <fstream>
#include <vector>




class ZMPProcess
{
public:
    ZMPProcess();
    ~ZMPProcess();
    void initialize();
    void setpSensorDataOffset(void *origen_sensor_data);
    void setpOrigenSensorData(void *origen_sensor_data);
    ZMPParam getZMPValue();
    float getForceLeft();
    float getForceRight();
    float getTorqueLeftX();
    float getTorqueLeftY();
    double *getpSensorForce();
    int *getpOrigenSensorData();
    double sensor_force[8];
    int origen_sensor_data[8];

    

    void saveData();
    std::map<std::string, std::vector<float>> map_zmp;
    int name_cont_;


private:
    void getSensorValue();
    void digital2KGProcess();
    ZMPParam ZMP;
    float force_left;
    float force_right;
    float torque_left_x;
    float torque_left_y;
    int sensor_offset_data_sum[8];
    int sensor_offset_data_count;
    int **ZMP_kg_offset;
    int *ZMP_kg_table;
};
#endif 
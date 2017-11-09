#include "common_include.h"
#include "config.h"

#define TEST_CSV_READER

using namespace MSCKF_MINE;

int main(int argc, char *argv[])
{
    /*test the config file*/
    Config::setParameterFile("../config/config.yaml");
    string sequence_dir = Config::get<string>("sequence_dir");
    string imu_path = sequence_dir + "imu0/data.csv";
    string camera_path = sequence_dir + "cam0/data.csv";
    cout << "sequence_dir = " << sequence_dir << endl;
    cout << "imu_path = " << imu_path << endl;

    /*test the csv.h file*/

//    CSVReader<7> imu_data("/home/m/ws/src/msckf_mine/datasets/MH_01_easy/mav0/imu0/data.csv");
//    imu_data.read_header(ignore_extra_column, "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]");
//    double imu_time, wx, wy, wz, ax, ay, az;

//    while (1)
//    {
////        imu_data.read_row(imu_time, wx, wy, wz, ax, ay, az);
////        cout << imu_time << " " << wx << " " << wy << " " << wz << endl;

//    }


    return 0;
}


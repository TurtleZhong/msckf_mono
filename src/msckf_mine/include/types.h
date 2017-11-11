#ifndef TYPES_H
#define TYPES_H
#include "common_include.h"
#include "config.h"

namespace MSCKF_MINE
{
class IMU
{
public:
    /*
         * timestamp [ns],
         * w_RS_S_x [rad s^-1],  w_RS_S_y [rad s^-1],   w_RS_S_z [rad s^-1],
         * a_RS_S_x [m s^-2],    a_RS_S_y [m s^-2],     a_RS_S_z [m s^-2]
         */
    double time_stamp;
    double wx;
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
};

class CAMERA
{
public:
    double time_stamp;
    string img_name;
};

class IMU_PARAM
{
public:
    IMU_PARAM()
    {
        g          = Config::get<double>("g");
        sigma_ac   = Config::get<double>("accelerometer_noise_density");    // [ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
        sigma_gc   = Config::get<double>("gyroscope_noise_density");        // [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )
        sigma_wac  = Config::get<double>("3.0000e-3");                      // [ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
        sigma_wgc  = Config::get<double>("1.9393e-05");                     // [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )
    }

    double g;
    double sigma_ac;
    double sigma_gc;
    double sigma_wac;
    double sigma_wgc;

};

}

#endif // TYPES_H

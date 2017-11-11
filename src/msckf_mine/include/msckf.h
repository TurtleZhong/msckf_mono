#ifndef MSCKF_H
#define MSCKF_H
#include "common_include.h"
#include "converter.h"
#include "camera.h"
#include "types.h"

using namespace std;
using namespace cv;
using namespace Eigen;


namespace MSCKF_MINE
{

class MSCKF
{
public:
    MSCKF();
    ~MSCKF();

    /*constructer*/
    MSCKF(const VectorXd &state, const MatrixXd &P, const Vector3d &Acc, const Vector3d &Gyro, double &dt );

    /*EKF propagation variables member*/
    VectorXd mState;
    MatrixXd mConvariance;

    /*Camera*/
    Camera mCAMParams;

    /*IMU params*/
    IMU_PARAM mIMUParams;

    /*accelerometer and gyroscope measurement*/
    Vector3d mAccMeasurement;       /*[m s^-2]*/
    Vector3d mGyroMeasurement;      /*[rad s^-1]*/

    Vector3d mGravity;

    double mdt;

    Vector3d mbg;
    Vector3d mba;

    Matrix4d BigOmega( const Vector3d &w );
    Matrix3d skewMatrix(const Vector3d &v);



};



}




#endif // MSCKF_H

#ifndef MSCKF_H
#define MSCKF_H
#include "common_include.h"
#include "converter.h"
#include "camera.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "frame.h"
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
    MatrixXd mCovariance;

    /*Camera*/
    Camera mCAMParams;

    /*IMU params*/
    IMU_PARAM mIMUParams;

    /*accelerometer and gyroscope measurement*/
    Vector3d mAccPrev;       /*[m s^-2]*/
    Vector3d mGyroPrev;      /*[rad s^-1]*/

    Vector3d mGravity;

    double mdt;

    Vector3d mbg;
    Vector3d mba;

    Matrix4d BigOmega( const Vector3d &w );
    Matrix3d skewMatrix(const Vector3d &v);

    void propagateIMU( Vector3d linear_acceleration, Vector3d angular_velocity);


    Matrix4d    calcOmegaMatrix(const Vector3d &w);
    Quaterniond calcDeltaQuaternion(const Vector3d &mGyroPrev, const Vector3d curr_w, double &dt);

    /*Augmentation cf. P.52*/
    void Augmentation();

    /*update step*/




    /*ORB Feature Parts*/
    ORB_PARAM orbParam;
    ORBextractor* mpORBextractor;
    Mat mImage; /*it should be noted that the image was undistorted in function unDistorImage()*/
    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;

    void unDistortImage();
    void extractFeatures();

    Frame frame;
    void ConstructFrame(const Mat &im, const double &timeStamp);



};



}




#endif // MSCKF_H

#include "msckf.h"
#include "config.h"
#include <Eigen/Dense>

namespace MSCKF_MINE
{

MSCKF::MSCKF()
{

}

MSCKF::MSCKF(const VectorXd &state, const MatrixXd &P, const Vector3d &Acc, const Vector3d &Gyro, double &dt)
{
    /*initializing all the variables*/
    /*actually we need to check the num of the variaables*/
    this->mState = state;
    this->mConvariance = P;
    this->mAccPrev = Acc;
    this->mGyroPrev = Gyro;
    this->mdt = dt;

    mGravity = Vector3d(0.0, 0.0, Config::get<double>("g"));

}

MSCKF::~MSCKF()
{

}


Matrix4d MSCKF::BigOmega( const Vector3d &w)
{
    //constructing big omega matrix
     Matrix4d W= Matrix4d::Zero();
     W.block<3,3>(0,0)  = -1*skewMatrix(w) ;
     W.block<1,3>(3,0)  =  -w.transpose();
     W.block<3,1>(0,3)  =  w;
     return W;

}

Matrix3d MSCKF::skewMatrix( const Vector3d &v)
{

    Matrix3d skewV;
    skewV << 0, -v(2) , v(1),
            v(2), 0 , -v(0),
            -v(1), v(0),  0;

    return skewV;

}


void MSCKF::processIMU(Vector3d linear_acceleration, Vector3d angular_velocity)
{
    /*Here we use the formulation in P.48 - P.52*/
    /*c.f. Monocular Visual Inertial Odometry on a Mobile Device*/

    Matrix3d d_R, pre_R;
    Vector3d s_hat, y_hat, tmp_vel, tmp_pos;

    /* step1: get the state from the mState.
     * It should be noted that the order of the state
     * quaternion position velocity gyro_bias acc_bias
     *     4         3        3         3        3
     */
    Vector4d spatial_quaternion = mState.segment(0, 4);
    Vector3d spatial_position   = mState.segment(4, 3);
    Vector3d spatial_velocity   = mState.segment(7, 3);
    Vector3d gyro_bias          = mState.segment(10,3);
    Vector3d acce_bias          = mState.segment(13,3);

    Quaterniond spa_q(spatial_quaternion);
    Matrix3d spatial_rotation = spa_q.matrix();

    /*step2: get the measurement of actual acc and w*/

    Vector3d curr_w = angular_velocity - gyro_bias;
    Vector3d curr_a = linear_acceleration - acce_bias;

    /* step3: caculate the qB(l+1)B(l) or RB(l+1)B(l) which represents the rotation from Bl to B(l+1)
     * method: the fourth order Runge-Kutta
     */
    d_R = calcDeltaQuaternion(mGyroPrev, curr_w, mdt).matrix();

    /* step4: calculate s_hat and y_hat cf. P49*/

    s_hat = 0.5 * dt * (dR.transpose() * curr_a + mAccPrev);
    y_hat = 0.5 * dt * s_hat;

    /* step5: update q v p*/
    pre_R = spatial_rotation;
    spatial_rotation = spatial_rotation * d_R.transpose(); /*from B(l+1) to G*/
    spatial_quaternion = Quaterniond(spatial_rotation).coeffs() /*w x y z*/;

    tmp_vel = spatial_velocity + spatial_rotation * s_hat + mGravity * mdt;
    tmp_pos = spatial_position + spatial_velocity * mdt + spatial_rotation * y_hat + 0.5 * mGravity * mdt * mdt;

    spatial_velocity = tmp_vel;
    spatial_position = tmp_pos;

    mState.segment(0,4) = spatial_quaternion;
    mState.segment(4,3) = spatial_position;
    mState.segment(7,3) = spatial_velocity;

    /*step6: update the mAccPrev and mGyroPrev*/
    this->mAccPrev  = curr_a;
    this->mGyroPrev = curr_w;

    /*step6: covariance P.50*/





}

Matrix4d MSCKF::calcOmegaMatrix(const Vector3d &w)
{
    Matrix4d W= Matrix4d::Zero();
    W.block<3,3>(0,0)  = -1*skewMatrix(w) ;
    W.block<1,3>(3,0)  =  -w.transpose();
    W.block<3,1>(0,3)  =  w;
    return W;
}



Quaterniond MSCKF::calcDeltaQuaternion(const Vector3d &mGyroPrev, const Vector3d curr_w, double &dt)
{
    Vector4d q0(0.0, 0,0, 0.0, 1.0);
    Vector4d k1, k2, k3, k4, d_q;

    k1 = 0.5 * calcOmegaMatrix(mGyroPrev) * q0;
    k2 = 0.5 * calcOmegaMatrix((mGyroPrev + curr_w) / 2) * (q0 + 0.5 * k1 * dt);
    k3 = 0.5 * calcOmegaMatrix((mGyroPrev + curr_w) / 2) * (q0 + 0.5 * k2 * dt);
    k4 = 0.5 * calcOmegaMatrix(curr_w) * (q0 + k3 * dt);

    d_q = q0 + ( (k1 + 2*k2 + 2*k3 + k4) * dt ) / 6.0;

    d_q = d_q / d_q.norm();

    return Quaterniond(d_q);

}





}

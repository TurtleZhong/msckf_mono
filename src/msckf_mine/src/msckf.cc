#include "msckf.h"
#include "config.h"
#include <Eigen/Dense>

namespace MSCKF_MINE
{


MSCKF::MSCKF()
{
    mpORBextractor = new ORBextractor(orbParam.nFeatures,orbParam.scaleFactor, orbParam.nLevels,orbParam.iniThFAST,orbParam.minThFAST);
    mbReset = true;
}

MSCKF::MSCKF(const VectorXd &state, const MatrixXd &P, const Vector3d &Acc, const Vector3d &Gyro, double &dt)
{
    /*initializing all the variables*/
    /*actually we need to check the num of the variaables*/
    this->mState = state;
    this->mCovariance = P;
    this->mAccPrev = Acc;
    this->mGyroPrev = Gyro;
    this->mdt = dt;

    mGravity = Vector3d(0.0, 0.0, Config::get<double>("g"));

    /*Construct the mpORBextractor*/
    mpORBextractor = new ORBextractor(orbParam.nFeatures,orbParam.scaleFactor, orbParam.nLevels,orbParam.iniThFAST,orbParam.minThFAST);
    mbReset = true;

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


void MSCKF::propagateIMU(Vector3d linear_acceleration, Vector3d angular_velocity)
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

    s_hat = 0.5 * mdt * (d_R.transpose() * curr_a + mAccPrev);
    y_hat = 0.5 * mdt * s_hat;

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

    /*step6: covariance P.50 & P.88*/
    /* |theta(l+1) |   |I3      03  03      phi_qbg  phi_qba|
     * |  pos(l+1) |   |phi_pq  I3  I3*mdt  phi_pbg  phi_pba|
     * |  vel(l+1) | = |phi_vq  03  I3      phi_vbg  phi_vba| + Q(noise)
     * |   bg(l+1) |   |03      03  03      I3       03     |
     * |   ba(l+1) |   |03      03  03      03       I3     |
     *
     * for details please cf.P88
     */

    MatrixXd phi = MatrixXd::Identity(15,15);
    phi.block<3,3>(3,0) = -1.0 * skewMatrix(pre_R * y_hat);  /*phi_pq*/
    phi.block<3,3>(6,0) = -1.0 * skewMatrix(pre_R * s_hat);  /*phi_vq*/

    phi.block<3,3>(3,6) = Matrix3d::Identity() * mdt;

    phi.block<3,3>(0,9) = -0.5 * mdt * (pre_R + spatial_rotation); /*phi_qbg*/
    phi.block<3,3>(6,9) = 0.25 * mdt * mdt * (skewMatrix(spatial_rotation * curr_a) * (pre_R + spatial_rotation));/*phi_qvbg*/
    phi.block<3,3>(3,9) = 0.5 * mdt * phi.block<3,3>(6,9);/*phi_qpbg*/

    phi.block<3,3>(0,12) = 0.5 * mdt * (pre_R + spatial_rotation);
    Matrix3d phi_vba = -0.5 * mdt * (pre_R + spatial_rotation) + 0.25 * mdt * mdt * (skewMatrix(spatial_rotation * curr_a) * (pre_R + spatial_rotation));
    phi.block<3,3>(6,12) = phi_vba;
    phi.block<3,3>(3,12) = 0.5 * mdt * phi_vba;

    Matrix<double, 15, 15> Nc;
    Matrix<double, 15, 1> Nc_diag;
    Nc_diag <<
               mIMUParams.sigma_gc*mIMUParams.sigma_gc * Vector3d(1, 1, 1),Vector3d(0, 0, 0),
            mIMUParams.sigma_ac*mIMUParams.sigma_ac * Vector3d(1, 1, 1),
            mIMUParams.sigma_wgc*mIMUParams.sigma_wgc * Vector3d(1, 1, 1),
            mIMUParams.sigma_wac*mIMUParams.sigma_wac * Vector3d(1, 1, 1);

    Nc = Nc_diag.asDiagonal();

    MatrixXd Qd = 0.5 * mdt * phi * Nc * phi.transpose() + Nc;
    mCovariance.block<15,15>(0,0) = phi * mCovariance.block<15,15>(0,0) * phi.transpose() + Qd;

    /* step7: update the PIC and PCI
     * may be some error!
     * we know that the covariance is structed as |PII PIC|
     *                                            |PCI PCC|
     * I -> Imu
     * C -> Camera
     */
    mCovariance.block(0, 15, 15, mCovariance.cols() - 15) = phi * mCovariance.block(0, 15, 15, mCovariance.cols() - 15);
    mCovariance.block(15, 0, mCovariance.rows() - 15, 15) = mCovariance.block(0, 15, 15, mCovariance.cols() - 15).transpose();

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
    Vector4d q0;
    q0 << 0.0, 0,0, 0.0, 1.0;
    Vector4d k1, k2, k3, k4, d_q;

    k1 = 0.5 * calcOmegaMatrix(mGyroPrev) * q0;
    k2 = 0.5 * calcOmegaMatrix((mGyroPrev + curr_w) / 2) * (q0 + 0.5 * k1 * dt);
    k3 = 0.5 * calcOmegaMatrix((mGyroPrev + curr_w) / 2) * (q0 + 0.5 * k2 * dt);
    k4 = 0.5 * calcOmegaMatrix(curr_w) * (q0 + k3 * dt);

    d_q = q0 + ( (k1 + 2*k2 + 2*k3 + k4) * dt ) / 6.0;

    d_q = d_q / d_q.norm();

    return Quaterniond(d_q);

}

void MSCKF::Augmentation()
{
    // when an image is captured, the current body quaternion, position and velocity are added to the
    // state vector and the covariance is augmented accordingly.

    /*step1: augmente the state vector*/

    int stateSize = mState.rows();
    mState.conservativeResize(stateSize + 10);

    mState.segment<10>(stateSize) = mState.head(10);

    /*step2: augmente the covariance
     * It should be noted that in step 1 we have add an image
     */
    int N = (mState.size() - 16) / 10; // N = number of image

    MatrixXd Jpi = MatrixXd::Zero(9, mState.size()-(N+1));
    MatrixXd I9 = MatrixXd::Identity(9,9);
    Jpi.block<9,9>(0,0) = I9;

    int covSize = mCovariance.rows();
    mCovariance.conservativeResize(covSize+9, covSize+9); /*since we have add an image, so the covariance need to be augmented*/

    mCovariance.block(0, covSize,covSize,9) = mCovariance.block(0,0,covSize,covSize)*Jpi.transpose();

    mCovariance.block(covSize,0,9,covSize) = mCovariance.block(0, covSize,covSize,9).transpose();
    mCovariance.block(covSize,covSize,9,9) = Jpi*mCovariance.block(0, covSize,covSize,9);

}

void MSCKF::Marginalizefilter()
{
    /*step 1: marginal the state vector*/
    mState.conservativeResize(16,1);

    /*step 2: marginal the cavariance*/
    mCovariance.conservativeResize(15,15);
}


/*ORB_parts*/


void MSCKF::imageComing(const Mat &image, const double timestamp)
{
    mImage = image.clone();
    mTimeStamp = timestamp;
}




void MSCKF::unDistortImage()
{
    cv::Mat tmp = mImage.clone();
    cv::undistort(tmp, mImage, mCAMParams.getK(), mCAMParams.getD(), cv::Mat());
    //    cv::imshow("undistort", mImage);
    //    cv::waitKey(0);

}

void MSCKF::extractFeatures()
{
    /*step1: --> undistort the image*/
    unDistortImage();
    /*step2: --> extract the features*/
    ORBextractor *pOrbExtractor = new ORBextractor(orbParam.nFeatures, orbParam.scaleFactor, orbParam.nLevels,
                                                   orbParam.iniThFAST, orbParam.minThFAST);
    (*pOrbExtractor)(mImage,cv::Mat(),mvKeys,mDescriptors);

}


void MSCKF::ConstructFrame(const Mat &im, const double &timeStamp)
{

    frame = Frame(im, timeStamp, mpORBextractor);
    mImage = im.clone();
}

void MSCKF::ConstructFrame(bool reset)
{
    if(!reset)
    {
        mpORBextractor->setFeatureNum(Config::get<int>("ORBextractor.nFeaturesInit"));
        frame = Frame(mImage, mTimeStamp, mpORBextractor);
    }

    else
    {
        mpORBextractor->setFeatureNum(Config::get<int>("ORBextractor.nFeatures"));
        frame = Frame(mImage, mTimeStamp, mpORBextractor);
    }
}


void MSCKF::RunFeatureMatching()
{
    if(mvFeatures.size() < 50)
    {
        /*it means that the number of tracked feature is too small */
        mvFeatures.clear();
        mvFeaturesIdx.clear();
        Marginalizefilter();
        mbReset = true;         /*which means we need to rest the filter*/
    }

    mvLostFeatures.clear();
    mvLostFeatureCamIdx.clear();


    /*step 1: Construct the frame*/
    this->ConstructFrame(mbReset);

    /*we need to know if the status is true or not */
    if(!mbReset)
    {
        /*in this case, the last frame is not empty so we can run the feature matching*/
        /*the most inportment part is frame.matchesID
         */

        ORBmatcher orbMatcher(0.7);
        orbMatcher.MatcheTwoFrames(frame, feedframe, false);

        /*after run we have got the matchesID and below are matches's data structure
         * id of feedframe, id of currframe */

        map<int,int> matches = Converter::swapMatchesId(frame.matchesId);
        map<int,int>::iterator matches_iter;

        for(int j = 0; j < mvFeaturesIdx.size(); j++)
        {
            int feedId = mvFeaturesIdx[j](0); /*key*/
            matches_iter = matches.find(feedId);
            if(matches_iter!=matches.end())
            {
                /*feature were tracked!*/
                int M = mvFeatures[j].rows();
                int N = mvFeatures[j].cols();
                mvFeatures[j].conservativeResize(M,N+1);
                mvFeatures[j](0,N) = frame.mvKeys[matches_iter->second].pt.x;
                mvFeatures[j](1,N) = frame.mvKeys[matches_iter->second].pt.y;

            }
            else
            {
                /*feature were not tracked, we need remove it and augment the lostfeatures*/
                mvLostFeatures.push_back(mvFeatures[j]);

                mvLostFeatureCamIdx.push_back(mvFeaturesIdx[j](1));

                mvFeatures.erase(mvFeatures.begin() + j);
                mvFeaturesIdx.erase(mvFeaturesIdx.begin() + j);

            }


        }



    }
    else
    {
        /* since the mbReset is true, there are two cases
         * case1: the first frame
         * case2: the mvFeatures.size() < 50 which means the tracked features is two small
         */
        feedframe = Frame(frame);

        /*step 2: AugmentNewFeatures*/

        AugmentNewFeatures();
        mbReset = false;
    }
}

/**
 * @brief MSCKF::AugmentNewFeatures-> used in RunFeatureMatching();
 */
void MSCKF::AugmentNewFeatures()
{
    mvFeatures    = vector<MatrixXd>(frame.mvKeys.size(), MatrixXd::Zero(2,1));
    mvFeaturesIdx = vector<Vector2i>(frame.mvKeys.size(), Vector2i::Zero());

    /*since the filter is reseted, we need add all the features to the feature manager vector*/

    for(int i = 0; i < frame.mvKeys.size(); i++)
    {
        mvFeatures[i](0,0) = frame.mvKeys[i].pt.x;
        mvFeatures[i](1,0) = frame.mvKeys[i].pt.y;
        mvFeaturesIdx[i]   = Vector2i(i, int(frame.mnId));
    }
}




}

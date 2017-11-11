#ifndef MSCKF_H
#define MSCKF_H
#include "common_include.h"
#include "converter.h"

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

    /*EKF propagation variables member*/
    VectorXd mState;
    MatrixXd mConvariance;



};



}




#endif // MSCKF_H

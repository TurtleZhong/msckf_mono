#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


int main(int argc, char *argv[])
{
    VectorXd state(16);
    Vector4d q;
    Vector3d p;
    Vector3d v;
    Vector3d bg;
    Vector3d ba;
    q << 1.0, 0.0, 0.0, 0.0;
    p << 3.0, 2.0, 1.0;
    v << 1.23, 2.23, 3.21;
    bg << 0, 2.23, 3.21;
    ba << 0.12, 2.2, 3.21;
    state.segment(0,4)  = Vector4d(1.0, 0.0, 0.0, 0.0);
    state.segment(4,3)  = Vector3d(3.0, 2.0, 1.0);
    state.segment(7,3)  = Vector3d(1.23, 2.23, 3.21);
    state.segment(10,3) = Vector3d(0, 2.23, 3.21);
    state.segment(13,3) = Vector3d(1.2, 0.2, 0.2);

    cout << "--state = \n" << state << endl;

    int Xsize = state.rows();
    state.conservativeResize(Xsize + 10);

    //appending the current body pose
    state.segment<10>(Xsize) = state.head(10);

    cout << "--after.size = \n" << state.rows() << endl;



    return 0;
}

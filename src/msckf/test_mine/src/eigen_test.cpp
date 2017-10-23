#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;





Vector3d xyzFromQuaternion(const Quaterniond &quaternion) {
    return quaternion.matrix().eulerAngles(2, 1, 0).reverse();
}

Quaterniond quaternionFromXYZ(const Vector3d &vector) {
    return AngleAxisd(vector[2], Vector3d::UnitZ()) *
            AngleAxisd(vector[1], Vector3d::UnitY()) *
            AngleAxisd(vector[0], Vector3d::UnitX());
}





int main(int argc, char *argv[])
{


    Matrix3d R = Matrix3d::Identity();
    cout << "R = \n" << R << endl;

    Quaterniond q = Quaterniond(R);
    cout << "q = \n" << q.coeffs() << endl;

    Quaterniond q1{1,0,0,0};
    cout << "q1 = \n" << q1.coeffs() << endl;

    Vector2d vec2d = Vector2d{1.0,2.3};
    cout << "vec2d = \n" << vec2d << endl;

    Vector2d vec2d_1 = Vector2d(0.5,0.7);
    cout << "vec2d_1 = \n" << vec2d_1 << endl;


    /*angleAxisd <--> Quaternion*/

    Eigen::AngleAxisd rotation_vector(M_PI/4, Vector3d(0,0,1));
    Eigen::Quaterniond q2(rotation_vector);

    cout << "rotation_vector = \n" << rotation_vector.matrix() << endl;


    Vector3d xyz = xyzFromQuaternion(q2);
    cout << "angleAxisd <-- Quaternion\n" << xyz << endl;

    Quaterniond q3 = quaternionFromXYZ(Vector3d(0,0,M_PI/4));
    cout << "anglexAxisd --> Quaternion\n" << q3.coeffs() << endl;
    cout << "M_PI/4 = " << M_PI/4 << endl;

    cout << "q3.inverse() = \n" << q3.inverse().coeffs() << endl;


    /**/
    VectorXd v(4);
    v << 1 ,2 ,3 , 4, 5, 6;
    cout << v << endl;




    return 0;
}

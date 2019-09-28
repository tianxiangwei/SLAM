#include <iostream>
#include "eigen3/Eigen/Dense"
#include <math.h>
using namespace std;
using namespace Eigen;

int main()
{

    double roll = -0.0092940517;
    double pitch = -0.23466094;
    double yaw = -0.07634446;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    //Eigen::Quaterniond q = rollAngle * yawAngle  * pitchAngle;
    //四元数转换顺序矩阵相乘顺序不变
    Eigen::Quaterniond q = pitchAngle * yawAngle  * rollAngle;
    cout << "quaterniod-->" << "x:" << q.x() << " y:" << q.y() << 
        " z:" << q.z() << " w:" << q.w();
    //验证矩阵方式转换
    


    /*Matrix3d n(3, 3);
    n << 1, 2, 3,
         2, 3, 4,
         5, 5, 7;
    Vector3d p(3, 1, 2);
    Vector3d q = n * p;
    std::cout << q.transpose() << std::endl;
    MatrixXf m(4,4);
    m<< 1,2,3,4,
        5,6,7,8,
        9,10,11,12,
        13,14,15,16;
    cout<<"Block in the middle"<<endl;
    cout<<m.block<2,2>(0,1)<<endl<<endl;
    for(int i = 1;i <= 3;++i)
    {
        cout<<"Block of size "<<i<<"x"<<i<<endl;
        cout<<m.block(0,0,i,i)<<endl<<endl;
    }

    Array22f m;
    m<< 1,2,
        3,4;
    Array44f a = Array44f::Constant(0.6);
    cout<<"Here is the array a:"<<endl<<a<<endl<<endl;
    a.block<2,2>(1,1) = m;
    cout<<"Here is now a with m copied into its central 2x2 block:"<<endl<<a<<endl<<endl;
    cout<<"a.block(2,1,2,3)"<<endl<<a.block(2,1,2,3)<<endl<<endl;
    a.block(0,0,2,3) = a.block(2,1,2,3);
    cout<<"Here is now a with bottom-right 2x3 block copied into top-left 2x2 block:"<<endl<<a<<endl<<endl;*/
}

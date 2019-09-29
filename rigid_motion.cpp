#include <iostream>
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "utility.h"
using namespace std;
using namespace Eigen;

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R)
{
    //不需要知道旋转顺序吗？
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    std::cout << "y:" << y << " p:" << p << " r:" << r << std::endl;

    return ypr;
}

int main()
{

    double roll = -0.0092940517;
    double pitch = -0.23466094;
    double yaw = -0.07634446;

    std::cout << "initail roll:" << roll << " pitch:"
    << pitch << " yaw:" << yaw << std::endl;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    //Eigen::Quaterniond q = yawAngle * pitchAngle  * rollAngle;
    //四元数转换顺序矩阵相乘顺序不变
    Eigen::Quaterniond q = pitchAngle * yawAngle  * rollAngle;
    cout << "quaterniod q-->" << "x:" << q.x() << " y:" << q.y() << 
        " z:" << q.z() << " w:" << q.w() << endl;

    std::cout << "" << std::endl;
    Eigen::Matrix3d R_trix = q.toRotationMatrix();
    std::cout << "R_matrix:" << R_trix << std::endl;
    std::cout << "" << std::endl;


    double w = sqrt(1+ R_trix(0, 0) + R_trix(1, 1) + R_trix(2, 2)) / 2;
    double x= (R_trix(2,1) - R_trix(1,2))/ 4 * w;
    double y= (R_trix(0,2) - R_trix(2,0))/ 4 * w;
    double z= (R_trix(1,0) - R_trix(0,1))/ 4 * w;

    /*double w = sqrt(1- R_trix(0, 0) + R_trix(1, 1) - R_trix(2, 2));
    double x= (R_trix(0,2) - R_trix(2,0))/ w;
    double y= (R_trix(0,2) - R_trix(2,0))/ 4 * w;
    double z= (R_trix(1,0) - R_trix(0,1))/ 4 * w;*/
    //有些误差，什么原因？
    std::cout << "quterniod x-->" << x << " y:" << y << 
    " z:" << z << " w:" << w << std::endl; 



    //roll pitch yaw -- 0 1 2
    //无论eulaerAngles参数如何变化，都无法转回原来的欧拉角? 欧拉角的奇异性导致的吗？
    Eigen::Vector3d euler_angels = q.matrix().eulerAngles(0, 1, 2);
    cout << "roll:" << euler_angels.x() << " pitch:" << euler_angels.y() 
    << " yaw:" << euler_angels.z() << endl;

    double roll1 = euler_angels.x();
    double pitch1 = euler_angels.y();
    double yaw1 = euler_angels.z();
    //得到的欧拉角不等于原始的欧拉角，这里生成的四元素也不等于原来的四元素
    Eigen::AngleAxisd rollAngle1(roll1, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle1(pitch1, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle1(yaw1, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q1 = pitchAngle1 * yawAngle1  * rollAngle1;
    cout << "quaterniod q1-->" << "x:" << q1.x() << " y:" << q1.y() << 
        " z:" << q1.z() << " w:" << q1.w() << endl;

    
    //转回欧拉角也不对
    Eigen::Vector3d ypr = R2ypr(R_trix);
    Quaterniond q_x = Utility::deltaQ(ypr);
    //验证矩阵方式转换
    Matrix3d R_roll;
    Matrix3d R_yaw;
    Matrix3d R_pitch;
    R_roll << 1, 0, 0,
              0, cos(roll), -sin(roll),
              0, sin(roll), cos(roll);
    R_yaw << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
    R_pitch << cos(pitch), 0, sin(pitch),
                0, 1, 0,
                -sin(pitch), 0, cos(pitch);
    //构造旋转矩阵的方式
    Matrix3d R = R_pitch * R_yaw * R_roll;
    std::cout << "the same R_trix:" << R << std::endl;
    Eigen::Quaterniond q2 = Quaterniond(R);
    cout << "the same quaterniod-->" << "x:" << q2.x() << " y:" << q2.y() << 
        " z:" << q2.z() << " w:" << q2.w() << endl;


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

#include <iostream>
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "utility.h"
using namespace std;
using namespace Eigen;

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R)
{
    //旋转顺序是yaw-->pitch-->roll，通过旋转矩阵求的欧拉角
    //R_yaw*R_pitch*R_roll
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

    cout << "eulerAngles ==> Quaterniond" << endl;
    std::cout << "initail roll:" << roll << " pitch:"
    << pitch << " yaw:" << yaw << std::endl;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    //Eigen::Quaterniond q = yawAngle * pitchAngle  * rollAngle;
    //四元数转换顺序矩阵相乘顺序不变
    //YZX
    Eigen::Quaterniond q = pitchAngle * yawAngle  * rollAngle;
    cout << "quaterniod q-->" << "x:" << q.x() << " y:" << q.y() << 
        " z:" << q.z() << " w:" << q.w() << endl;

    std::cout << "" << std::endl;
    Eigen::Matrix3d R_trix = q.toRotationMatrix();
    std::cout << "R_matrix:" << R_trix << std::endl;
    std::cout << "" << std::endl;

    //验证矩阵方式转换,通过以下方式得到的四元数和使用Eigen的AngleAxisd得到四元数相同
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
    //构造旋转矩阵的方式 (顺规)
    //欧拉角构造旋转矩阵的顺序 谁先旋转谁的矩阵在左边，因此以下是pitch-->yaw-->roll的顺序
    Matrix3d R_result;
    R_result << cos(pitch) * cos(yaw), -cos(pitch) * sin(yaw) * cos(roll) + sin(pitch) * sin(roll), cos(pitch) * sin(yaw) * sin(roll)+sin(pitch) * cos(roll),
                sin(yaw), cos(yaw) * cos(roll), -cos(yaw) * sin(roll),
                -sin(pitch) * cos(yaw), sin(pitch)*sin(yaw) * cos(roll)+cos(pitch) * sin(roll), -sin(pitch) * sin(yaw) * sin(roll)+cos(pitch) * cos(roll);
    cout << "the same R_result:" << R_result << std::endl;

    Matrix3d R = R_pitch * R_yaw * R_roll;
    std::cout << "the same R_trix:" << R << std::endl;
    Eigen::Quaterniond q2 = Quaterniond(R);
    cout << "the same quaterniod-->" << "x:" << q2.x() << " y:" << q2.y() << 
        " z:" << q2.z() << " w:" << q2.w() << endl;

    cout << endl;
    cout << endl;
    cout << "Quaterniond ==> R" <<endl;
    //从四元素到旋转矩阵
    //R(q) = [
    //        1-2y^2-2z^2  2xy-2zw,      2xz+2yw
    //        2xy+2zw      1-2x^2-2z^2   2yz-2xw
    //        2xz-2yq      2yz+2xw       1-2x^2-2y^2
    //       ]
    Matrix3d R1;
    double x = q2.x();
    double y = q2.y();
    double z = q2.z();
    double w = q2.w();
    R1 << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 *y*w, 2*x*y+2*z*w,1-2*x*x-2*z*z, 2*y*z-2*x*w, 2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y;
    cout << "quaterniond to R:" << R1 << endl; 

   cout << "R ==> Quaterniond" <<endl;
   //由旋转矩阵转换到四元数
    w = sqrt(1+ R_trix(0, 0) + R_trix(1, 1) + R_trix(2, 2)) / 2;
    x= (R_trix(2,1) - R_trix(1,2))/ 4 * w;
    y= (R_trix(0,2) - R_trix(2,0))/ 4 * w;
    z= (R_trix(1,0) - R_trix(0,1))/ 4 * w;
    //有些误差，什么原因？
    std::cout << "R to quaterniond: x-->" << x << " y:" << y << 
    " z:" << z << " w:" << w << std::endl; 
    cout << endl;
    cout << endl;

    cout << "R ==> eulaerAngles" << endl;
    //通过旋转矩阵求欧拉角的方法
    //从R_result的构造反求获得
    //三角函数重要公式 x=atan2(sin(x), cos(x));
    std::cout << "R(1, 0):" << R_result(1, 0) << std::endl;
    double yaw_result = asin(R_result(1, 0));
    cout << "R(1, 0):" << sin(-yaw_result) << std::endl;
    cout << "yaw:" << yaw_result << std::endl;
   std::cout << "sin(yaw_result):" << sin(yaw_result) << std::endl;
   
   //sin(-p)=-sin(p) cos(-p)=cos(p)
   //注意使用以下方式求解是不对的 
   //R(1, 1) = cos(yaw) *cos(roll)
   std::cout << "cos(yaw_result):" << cos(yaw_result) << std::endl;
   std::cout << "cos(-yaw_result):" << cos(-yaw_result) << std::endl;double roll_value = R_result(1, 1) / cos(yaw_result);
   std::cout << "roll_value:" << roll_value << std::endl;
   double roll_result = acos(R_result(1, 1) / cos(yaw_result));
   std::cout << "roll:" << roll_result << std::endl;

    //这种方式求解是正确的
    //atan2(cos(yaw)*sin(roll), cos(yaw)*cos(roll)),同时消掉cos(yaw);
    std::cout <<"roll:" << atan2(-R_result(1, 2), R_result(1, 1)) << std::endl;
    std::cout << "pitch" << atan2(-R_result(2, 0), R_result(0, 0)) << std::endl;

    cout << endl;
    cout <<endl;
    cout << "R==>AngleAxisd" << endl;
    //使用旋转矩阵转旋转向量的方式
    //方法1 
    double theta = acos((R_trix(0, 0) + R_trix(1,1) + R_trix(2,2) - 1) / 2);
    cout << "Angle:" << theta << endl;
    //1/2*sin(theta)  [ r32 - r23
    //                  r13 - r31
    //                  r21 - r12
    //                ]
    double x1 = 1 / (2 * sin(theta));
    //cout << "x1:" << x1 << endl;

    Vector3d r = Vector3d(R_trix(2,1) - R_trix(1, 2), R_trix(0,2) - R_trix(2,0), R_trix(1,0)-R_trix(0,1)) * x1;
    cout << "Rotation Vector:" << r.transpose() << endl;
    //Eigen 从旋转矩阵到旋转向量
    AngleAxisd V1;
    V1.fromRotationMatrix(R_trix);
    cout << "Angle:" << V1.angle() << endl;
    cout << "Rotation Vector:" << V1.axis().transpose() << endl;

    //使用如下公式可以从轴角转换成旋转矩阵
    cout << "AngleAxisd ==> R" << endl;
    double rx = r(0);
    double ry=r(1);
    double rz=r(2);
    double cos_x = cos(theta);
    double sin_x = sin(theta);
    Matrix3d R_axis;
    R_axis << rx*rx*(1-cos_x)+cos_x, rx*ry*(1-cos_x)-rz*sin_x, rx*rz*(1-cos_x)+ry*sin_x,
            rx*ry*(1-cos_x)+rz*sin_x, ry*ry*(1-cos_x)+cos_x, ry*rz*(1-cos_x)-rx*sin_x,
            rx*rz*(1-cos_x)-ry*sin_x, ry*rz*(1-cos_x)+rx*sin_x, rz*rz*(1-cos_x)+cos_x;
   cout << "Rottion:"<<R_axis<< endl;

    //使用Eigen可以从轴角直接转换成旋转矩阵 
    cout << "Rotation_vector1" << endl << V1.matrix() << endl;
    //方法2
    AngleAxisd V2;
    V2= R_trix;
    cout << "Rotation_vector2" << endl << V2.matrix() << endl;
    //方法3
    AngleAxisd V3(R_trix);
    cout << "Rotation_vector3" << endl << V3.matrix() << endl;

    Eigen::Quaterniond q3 = Quaterniond(V3.matrix());
    cout << "the same quaterniod-->" << "x:" << q3.x() << " y:" << q3.y() << 
        " z:" << q3.z() << " w:" << q3.w() << endl;
 
    //使用四元数来对旋转向量进行赋值
    //方法1
    AngleAxisd V5;
    V5 = q;
    cout << "Rotation_vector5" << endl << V5.matrix() << endl;
 
    //3.2 
    AngleAxisd V6(q);
    cout << "Rotation_vector6" << endl << V6.matrix() << endl;
    

    cout << endl;
    cout << endl;
    cout <<"Eigen 四元素到欧拉角转换不对" <<endl;
    cout << "Eigen quatoniond <==> eulaerAngles" << endl;
    //roll pitch yaw -- 0 1 2
    //无论eulaerAngles参数如何变化，都无法转回原来的欧拉角? 欧拉角的奇异性导致的吗？
    Eigen::Vector3d euler_angels = R_trix.eulerAngles(1, 2, 0);
    cout << "roll:" << euler_angels.x() << " pitch:" << euler_angels.y() 
    << " yaw:" << euler_angels.z() << endl;

    //just test
    //无法从四元素恢复回欧拉角?
    cout << endl;
    cout << "just test eigen aulerAngles" << endl;
    Eigen::Vector3f  test_angle(roll, pitch, yaw);
    cout << "roll:" << test_angle(0) << " pitch:" << test_angle(1) << 
        " yaw:" << test_angle(2)<< endl;

    Eigen::Quaterniond q_test = Eigen::AngleAxisd(test_angle[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(test_angle[1], Eigen::Vector3d::UnitY()) * 
    Eigen::AngleAxisd(test_angle[2], Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d RXYZ = q_test.toRotationMatrix();
    Eigen::Vector3d EulerAngleZYX = RXYZ.eulerAngles(2, 1, 0);

    cout << "roll:" << EulerAngleZYX(0) << " pitch:" << EulerAngleZYX(1) << 
        " yaw:" << EulerAngleZYX(2)<< endl;
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

    cout << endl;
    //欧拉角是yaw-->pitch-->roll顺规，通过以下公式进行:
    //旋转矩阵到欧拉角,然后欧拉角到四元素的转换
    //Eigen::Vector3d ypr = R2ypr(R_trix);
    //Quaterniond q_x = Utility::deltaQ(ypr);
    cout << endl;
}
